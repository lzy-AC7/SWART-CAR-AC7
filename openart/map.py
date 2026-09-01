import sensor
import time
import struct
import image
from machine import UART
from pyb import LED

red = LED(1)
red.on()

COLS = 14
ROWS = 10

ROI_X, ROI_Y, ROI_W, ROI_H = 16, 21, 288, 198
MAP_ROI = (ROI_X, ROI_Y, ROI_W, ROI_H)

# 颜色阈值
THRESH_WALL = (0, 100, -10, 50, -70, 20)
THRESH_BOX = (0, 100, -40, 0, 50, 127)
THRESH_BOMB = (0, 100, 20, 127, 0, 127)
THRESH_GOAL = (0, 100, 60, 127, -80, -50)
THRESH_HEAD = (0, 100, -128, -30, -128, 20)
THRESH_BODY = (0, 100, -128, -45, 10, 127)

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.set_auto_gain(False)
sensor.skip_frames(time=2000)

uart = UART(12, baudrate=115200)

# ------------------ 圈外提亮参数 ------------------
CENTER_CX = 160
CENTER_CY = 120
CENTER_R = 100
OUTSIDE_BRIGHT = 40  # 圈外提亮强度
# --------------------------------------------------

# 创建【圈外提亮遮罩】(圈外为 OUTSIDE_BRIGHT，圈内抠空为 0)
bright_mask = image.Image(320, 240, sensor.RGB565, copy_to_fb=False)
bright_mask.draw_rectangle(
    0,
    0,
    320,
    240,
    color=(OUTSIDE_BRIGHT, OUTSIDE_BRIGHT, OUTSIDE_BRIGHT),
    fill=True,
)
bright_mask.draw_circle(CENTER_CX, CENTER_CY, CENTER_R, color=(0, 0, 0), fill=True)

# 状态变量
out_px = out_py = None
keep_sending_pos = False


def in_range(stat, threshold):
    if stat.l_mean() == 0 and stat.a_mean() == 0:
        return False
    return (threshold[0] <= stat.l_mean() <= threshold[1] and
            threshold[2] <= stat.a_mean() <= threshold[3] and
            threshold[4] <= stat.b_mean() <= threshold[5])


def recognize_single_map(img, cell_w, cell_h):
    current_map = [['-' for _ in range(COLS)] for _ in range(ROWS)]
    for r in range(ROWS):
        for c in range(COLS):
            tx = int(max(0, min(319, ROI_X + c * cell_w + 1)))
            ty = int(max(0, min(239, ROI_Y + r * cell_h + 1)))
            tw = int(max(1, min(cell_w - 2, 320 - tx)))
            th = int(max(1, min(cell_h - 2, 240 - ty)))

            test_roi = (tx, ty, tw, th)
            stat = img.get_statistics(roi=test_roi)

            if in_range(stat, THRESH_WALL):
                current_map[r][c] = '#'
            elif in_range(stat, THRESH_BOX):
                current_map[r][c] = '$'
            elif in_range(stat, THRESH_BOMB):
                current_map[r][c] = '*'
            elif in_range(stat, THRESH_GOAL):
                current_map[r][c] = '.'

            img.draw_rectangle(test_roi, color=(0, 255, 0), thickness=1)
    return current_map


def send_position():
    global out_px, out_py
    if out_px is not None:
        val_x = int(out_px * 1000)
        val_y = int(out_py * 1000)
        data_bytes = struct.pack('<2H', val_x, val_y)
        checksum = sum(data_bytes) & 0xFF
        frame = b'\xAC' + data_bytes + struct.pack('<B', checksum)
        uart.write(frame)


def draw_debug_info(img, cell_w, cell_h, current_search_roi):
    img.draw_rectangle(MAP_ROI, color=(255, 255, 255), thickness=1)
    for i in range(1, COLS):
        vx = int(ROI_X + i * cell_w)
        img.draw_line(vx, ROI_Y, vx, ROI_Y + ROI_H, color=(120, 120, 120))
    for i in range(1, ROWS):
        vy = int(ROI_Y + i * cell_h)
        img.draw_line(ROI_X, vy, ROI_X + ROI_W, vy, color=(120, 120, 120))

    if current_search_roi != MAP_ROI:
        img.draw_rectangle(current_search_roi, color=(0, 0, 255), thickness=1)

    if out_px is not None:
        f_px_pixel = int(out_px * cell_w + ROI_X)
        f_py_pixel = int(out_py * cell_h + ROI_Y)
        img.draw_cross(f_px_pixel, f_py_pixel, color=(255, 255, 0), size=5, thickness=1)

    # 参考线
    img.draw_circle(CENTER_CX, CENTER_CY, CENTER_R, color=(255, 0, 0), thickness=1)


keep_sending_pos = True
red.off()

while True:
    img = sensor.snapshot()

    # --- 图像光照处理（仅圈外提亮） ---
    img.add(bright_mask)

    cell_w = ROI_W / COLS
    cell_h = ROI_H / ROWS

    # --- 动态 roi ---
    if out_px is not None and out_py is not None:
        curr_c = int(out_px)
        curr_r = int(out_py)

        min_c = max(0, curr_c - 1)
        max_c = min(COLS - 1, curr_c + 1)
        min_r = max(0, curr_r - 1)
        max_r = min(ROWS - 1, curr_r + 1)

        search_x = int(ROI_X + min_c * cell_w)
        search_y = int(ROI_Y + min_r * cell_h)
        search_w = int((max_c - min_c + 1) * cell_w)
        search_h = int((max_r - min_r + 1) * cell_h)

        search_roi = (search_x, search_y, search_w, search_h)
    else:
        search_roi = MAP_ROI

    # --- 玩家定位 ---
    h_blobs = img.find_blobs([THRESH_HEAD], roi=search_roi, pixels_threshold=100, area_threshold=100, merge=True)
    b_blobs = img.find_blobs([THRESH_BODY], roi=search_roi, pixels_threshold=100, area_threshold=100, merge=True)
    if h_blobs and b_blobs:
        h = max(h_blobs, key=lambda b: b.pixels())
        b = max(b_blobs, key=lambda b: b.pixels())

        px, py = (h.cx() + b.cx()) / 2.0, (h.cy() + b.cy()) / 2.0
        raw_pc, raw_pr = (px - ROI_X) / cell_w, (py - ROI_Y) / cell_h

        if ((out_px is None) or (abs(raw_pc - out_px) < 1 and abs(raw_pr - out_py) < 1)):
            out_px, out_py = raw_pc, raw_pr
    else:
        out_px, out_py = None, None

    # --- 命令接收与逻辑 ---
    if uart.any():
        cmd = uart.read().decode().strip()
        uart.write(f"{cmd}\n")
        print(f"{cmd}\n 已应答")

        if '1' in cmd:
            keep_sending_pos = False
            last_map = None
            consecutive_count = 0
            max_attempts = 10  # 最多抓拍10次，防止因画面抖动卡死
            attempt = 0

            print("开始地图识别，等待连续2次一致...")
            while consecutive_count < 2 and attempt < max_attempts:
                attempt += 1
                img = sensor.snapshot()
                img.add(bright_mask)  # 使用圈外提亮增强

                curr_map = recognize_single_map(img, cell_w, cell_h)

                if curr_map == last_map:
                    consecutive_count += 1
                else:
                    consecutive_count = 1
                    last_map = curr_map

                print(f"地图识别尝试 {attempt}/{max_attempts}, 连续一致次数: {consecutive_count}")

            # 组装最终地图字符列表
            final_map = ["".join(line) for line in last_map]

            # 串口发送地图
            for line in final_map:
                uart.write(f"{line}")
                # print(line)
            uart.write("\n")
            time.sleep_ms(10)
            print("===== 地图识别完毕并已发送 =====")

        elif '2' in cmd:
            time.sleep_ms(10)
            keep_sending_pos = False
        elif '0' in cmd:
            time.sleep_ms(10)
            keep_sending_pos = True

    # --- 发送坐标 ---
    if keep_sending_pos:
        send_position()

    # --- 调试画面 ---
    # draw_debug_info(img, cell_w, cell_h, search_roi)
