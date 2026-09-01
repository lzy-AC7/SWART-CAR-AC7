import sensor
import time
import struct
import image
from machine import UART
from pyb import LED


# ----------------- 逆透视数学矩阵引擎 -----------------
def solve_8x8(A, B):
    """高斯消元法求解 8x8 线性方程组 A*x = B"""
    N = 8
    for i in range(N):
        max_row = i
        for r in range(i + 1, N):
            if abs(A[r][i]) > abs(A[max_row][i]):
                max_row = r
        A[i], A[max_row] = A[max_row], A[i]
        B[i], B[max_row] = B[max_row], B[i]

        pivot = A[i][i]
        if abs(pivot) < 1e-12:
            continue

        for j in range(i, N):
            A[i][j] /= pivot
        B[i] /= pivot

        for r in range(N):
            if r != i:
                factor = A[r][i]
                for j in range(i, N):
                    A[r][j] -= factor * A[i][j]
                B[r] -= factor * B[i]
    return B


def get_perspective_transform(src_pts, dst_pts):
    """计算 4 点对应的透视变换参数 (a, b, c, d, e, f, g, h)"""
    A = []
    B = []
    for i in range(4):
        x, y = src_pts[i]
        u, v = dst_pts[i]
        A.append([x, y, 1, 0, 0, 0, -x * u, -y * u])
        B.append(u)
        A.append([0, 0, 0, x, y, 1, -x * v, -y * v])
        B.append(v)
    return solve_8x8(A, B)


def transform_point(H, x, y):
    """使用变换参数 H 将坐标 (x, y) 转换为映射后的坐标 (u, v)"""
    a, b, c, d, e, f, g, h = H
    denom = g * x + h * y + 1.0
    if abs(denom) < 1e-9:
        return 0.0, 0.0
    u = (a * x + b * y + c) / denom
    v = (d * x + e * y + f) / denom
    return u, v


# ------------------------------------------------------


def clamp(val, min_val, max_val):
    return max(min_val, min(val, max_val))


def get_safe_roi(cx, cy, w, h, img_w=320, img_h=240):
    sx = int(cx - w / 2)
    sy = int(cy - h / 2)
    sx = max(0, min(sx, img_w - 1))
    sy = max(0, min(sy, img_h - 1))
    sw = int(min(w, img_w - sx))
    sh = int(min(h, img_h - sy))
    return (sx, sy, sw, sh)


red = LED(1)
red.on()

COLS = 14
ROWS = 10

# ----------------- 逆透视四角点配置 -----------------
# 请在 OpenMV IDE 中点击实际图像，填入地图四个角点的像素坐标
# 顺序必须严格为：[左上, 右上, 右下, 左下]
MAP_CORNERS = [(16, 21), (303, 21), (303, 218), (17, 219)]

# 目标地图网格坐标范围
DST_CORNERS = [(0.0, 0.0), (float(COLS), 0.0), (float(COLS), float(ROWS)), (0.0, float(ROWS))]

# 计算双向映射矩阵 (只需在启动时计算一次，不占运行时间)
H_img2map = get_perspective_transform(MAP_CORNERS, DST_CORNERS)  # 原图像素 -> 地图坐标
H_map2img = get_perspective_transform(DST_CORNERS, MAP_CORNERS)  # 地图坐标 -> 原图像素

# 计算角点在原图中的包围盒，作为全局搜索区域
min_x = int(clamp(min(p[0] for p in MAP_CORNERS), 0, 319))
max_x = int(clamp(max(p[0] for p in MAP_CORNERS), 0, 319))
min_y = int(clamp(min(p[1] for p in MAP_CORNERS), 0, 239))
max_y = int(clamp(max(p[1] for p in MAP_CORNERS), 0, 239))
MAP_BOUNDING_ROI = (min_x, min_y, max(1, max_x - min_x), max(1, max_y - min_y))
FULL_ROI = (0, 0, 320, 240)

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

# 圈外提亮参数
CENTER_CX, CENTER_CY, CENTER_R = 160, 120, 100
OUTSIDE_BRIGHT = 40

bright_mask = image.Image(320, 240, sensor.RGB565, copy_to_fb=False)
bright_mask.draw_rectangle(0, 0, 320, 240, color=(OUTSIDE_BRIGHT, OUTSIDE_BRIGHT, OUTSIDE_BRIGHT), fill=True)
bright_mask.draw_circle(CENTER_CX, CENTER_CY, CENTER_R, color=(0, 0, 0), fill=True)

# 状态变量
out_px = out_py = None
keep_sending_pos = False
frames_lost = 0


def in_range(stat, threshold):
    if stat.l_mean() == 0 and stat.a_mean() == 0:
        return False
    return (threshold[0] <= stat.l_mean() <= threshold[1] and
            threshold[2] <= stat.a_mean() <= threshold[3] and
            threshold[4] <= stat.b_mean() <= threshold[5])


def get_cell_roi(r, c):
    """计算倾斜视角下，地图(r, c)网格在原图中对应的检测采样框"""
    map_pts = [(c + 0.15, r + 0.15), (c + 0.85, r + 0.15), (c + 0.85, r + 0.85), (c + 0.15, r + 0.85)]
    xs, ys = [], []
    for mc, mr in map_pts:
        ix, iy = transform_point(H_map2img, mc, mr)
        xs.append(ix)
        ys.append(iy)

    tx = int(clamp(min(xs), 0, 319))
    ty = int(clamp(min(ys), 0, 239))
    tw = max(1, min(int(max(xs) - min(xs)), 320 - tx))
    th = max(1, min(int(max(ys) - min(ys)), 240 - ty))
    return (tx, ty, tw, th)


def recognize_single_map(img):
    current_map = [['-' for _ in range(COLS)] for _ in range(ROWS)]
    for r in range(ROWS):
        for c in range(COLS):
            test_roi = get_cell_roi(r, c)
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


def draw_debug_info(img, current_search_roi):
    # 1. 绘制四个角点围成的倾斜区域
    for i in range(4):
        p1 = (int(MAP_CORNERS[i][0]), int(MAP_CORNERS[i][1]))
        p2 = (int(MAP_CORNERS[(i + 1) % 4][0]), int(MAP_CORNERS[(i + 1) % 4][1]))
        img.draw_line(p1[0], p1[1], p2[0], p2[1], color=(255, 255, 255), thickness=1)

    # 2. 绘制透视网格线 (将网格投影回原图)
    for i in range(1, COLS):
        x1, y1 = transform_point(H_map2img, i, 0)
        x2, y2 = transform_point(H_map2img, i, ROWS)
        img.draw_line(int(x1), int(y1), int(x2), int(y2), color=(100, 100, 100))
    for i in range(1, ROWS):
        x1, y1 = transform_point(H_map2img, 0, i)
        x2, y2 = transform_point(H_map2img, COLS, i)
        img.draw_line(int(x1), int(y1), int(x2), int(y2), color=(100, 100, 100))

    # 3. 动态搜索框
    if current_search_roi != MAP_BOUNDING_ROI and current_search_roi != FULL_ROI:
        img.draw_rectangle(current_search_roi, color=(0, 0, 255), thickness=1)

    # 4. 交叉标记（将算出的逆透视坐标转回原图像素绘制出来，验证精度）
    if out_px is not None:
        f_px_pixel, f_py_pixel = transform_point(H_map2img, out_px, out_py)
        img.draw_cross(int(f_px_pixel), int(f_py_pixel), color=(255, 255, 0), size=5, thickness=1)

    print(out_px, out_py)
    img.draw_circle(CENTER_CX, CENTER_CY, CENTER_R, color=(255, 0, 0), thickness=1)


keep_sending_pos = True
red.off()

while True:
    img = sensor.snapshot()
    img.add(bright_mask)

    # 1. 动态生成原图像素空间下的 ROI
    if out_px is not None and out_py is not None:
        # 将上一帧地图坐标转回原图像素位置，以此为中心建立局部 ROI
        last_px, last_py = transform_point(H_map2img, out_px, out_py)
        search_roi = get_safe_roi(last_px, last_py, 60, 60)  # 局部搜索框大小
    else:
        search_roi = MAP_BOUNDING_ROI

    # 2. 玩家定位观测
    h_blobs = img.find_blobs([THRESH_HEAD], roi=search_roi, pixels_threshold=100, area_threshold=100, merge=True)
    b_blobs = img.find_blobs([THRESH_BODY], roi=search_roi, pixels_threshold=100, area_threshold=100, merge=True)

    # 局部搜索找不到时，回退到包围盒全图搜索
    if search_roi != MAP_BOUNDING_ROI and not (h_blobs or b_blobs):
        search_roi = MAP_BOUNDING_ROI
        h_blobs = img.find_blobs([THRESH_HEAD], roi=search_roi, pixels_threshold=100, area_threshold=100, merge=True)
        b_blobs = img.find_blobs([THRESH_BODY], roi=search_roi, pixels_threshold=100, area_threshold=100, merge=True)

    valid_target_found = False
    raw_pc = raw_pr = 0.0

    if h_blobs or b_blobs:
        if h_blobs and b_blobs:
            h = max(h_blobs, key=lambda b: b.pixels())
            b = max(b_blobs, key=lambda b: b.pixels())
            px, py = (h.cx() + b.cx()) / 2.0, (h.cy() + b.cy()) / 2.0
        elif h_blobs:
            h = max(h_blobs, key=lambda b: b.pixels())
            px, py = float(h.cx()), float(h.cy())
        else:
            b = max(b_blobs, key=lambda b: b.pixels())
            px, py = float(b.cx()), float(b.cy())

        # 【核心逆透视计算】：将原图像素坐标 (px, py) 瞬间转换为纠正透视后的地图坐标 (temp_pc, temp_pr)
        temp_pc, temp_pr = transform_point(H_img2map, px, py)

        if 0 <= temp_pc <= COLS and 0 <= temp_pr <= ROWS:
            valid_target_found = True
            raw_pc = clamp(temp_pc, 0, COLS - 0.001)
            raw_pr = clamp(temp_pr, 0, ROWS - 0.001)

    # 3. 坐标状态更新
    if valid_target_found:
        frames_lost = 0
        out_px, out_py = raw_pc, raw_pr
    else:
        if out_px is not None:
            frames_lost += 1
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
            max_attempts = 10
            attempt = 0

            print("开始地图识别...")
            while consecutive_count < 2 and attempt < max_attempts:
                attempt += 1
                img = sensor.snapshot()
                img.add(bright_mask)
                curr_map = recognize_single_map(img)
                if curr_map == last_map:
                    consecutive_count += 1
                else:
                    consecutive_count = 1
                    last_map = curr_map

            final_map = ["".join(line) for line in last_map]
            for line in final_map:
                uart.write(f"{line}")
            uart.write("\n")
            time.sleep_ms(10)

        elif '2' in cmd:
            time.sleep_ms(10)
            keep_sending_pos = False
        elif '0' in cmd:
            time.sleep_ms(10)
            keep_sending_pos = True

    # --- 发送坐标 ---
    if keep_sending_pos:
        send_position()

    # 调试显示
    # draw_debug_info(img, search_roi)
