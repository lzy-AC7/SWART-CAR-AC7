import sensor
import time
import tf
import gc
from machine import UART
from pyb import LED

red = LED(1)
red.on()

# ===================== 核心配置 =====================
MODEL1_PATH = "7.30_0.75_128.tflite"
LABEL1_PATH = "/sd/labels.txt"

MODEL2_PATH = "7.6_num_0.75_96.tflite"
LABEL2_PATH = "/sd/labels_number.txt"

UART_BAUDRATE = 115200
MAX_TOTAL_TIMES = 5  # 最大尝试次数 (防止一直无法连续两次相同而卡死)

CROP_ROI = (7, 7, 315, 235)
# ====================================================

# 1. 硬件初始化
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.set_auto_gain(False)
sensor.set_vflip(True)
time.sleep_ms(20)
sensor.set_hmirror(True)
sensor.set_auto_exposure(False, exposure_us=500)
sensor.skip_frames(time=2000)

uart = UART(12, baudrate=UART_BAUDRATE)


def load_labels(path):
    try:
        with open(path, "r") as f:
            return [line.strip() for line in f if line.strip()]
    except OSError as e:
        print(f"文件读取失败: {e}")
        return []


# ==================== 【初始化时静态加载双模型】 ====================
print("正在初始化内存并加载双模型...")
gc.collect()

try:
    print(f"正在加载模型1: {MODEL1_PATH}")
    net1 = tf.load(MODEL1_PATH)
    labels1 = load_labels(LABEL1_PATH)

    print(f"正在加载模型2: {MODEL2_PATH}")
    net2 = tf.load(MODEL2_PATH)
    labels2 = load_labels(LABEL2_PATH)

    print("===== 双模型同时加载成功！ =====")
    print(f"加载后当前可用 RAM: {gc.mem_free()} 字节")
except Exception as e:
    print(f"❌ 严重错误：模型加载失败！可能是内存不足或文件不存在。详情: {e}")
    while True:
        # 如果加载失败，红灯快闪报错并死机
        red.toggle()
        time.sleep_ms(100)

# =========================================================================

# 3. 运行状态变量
is_running = 0  # 0:待机, 1:模型1运行, 2:模型2运行
active_net = None
active_labels = []

current_count = 0
best_label = None
best_conf = 0.0
last_label = None
consecutive_count = 0  # 连续相同识别次数计数器

print("系统已就绪，等待串口指令 (0 或 1)...")
red.off()

while True:
    img = sensor.snapshot()

    # --------------------- 指令接收与即时应答 ---------------------
    if uart.any():
        raw_data = uart.read().decode().strip()
        if raw_data:
            uart.write(f"{raw_data}\n")
            time.sleep_ms(10)
            print(f"收到指令: {raw_data}，已应答")

            success = False
            if "0" in raw_data:
                active_net = net1
                active_labels = labels1
                is_running = 1
                success = True
                print("→ 瞬间切换至模型1")
            elif "1" in raw_data:
                active_net = net2
                active_labels = labels2
                is_running = 2
                success = True
                print("→ 瞬间切换至模型2")

            if success:
                # 重置识别计数器和状态
                current_count = 0
                best_label = None
                best_conf = 0.0
                last_label = None
                consecutive_count = 0
                print("开始执行识别任务...")
            else:
                is_running = 0
                print("无效指令，保持待机模式")

    if not is_running or active_net is None:
        continue

    # --------------------- 识别逻辑 ---------------------
    current_count += 1
    try:
        classify_results = tf.classify(
            active_net,
            img,
            roi=CROP_ROI,
            min_scale=1.0,
            scale_mul=0.9,
            x_overlap=0.5,
            y_overlap=0.5,
        )

        # ====== 处理分类结果 ======
        for res in classify_results:
            output = res.output()
            if len(output) != len(active_labels):
                continue
            max_conf = max(output)
            max_idx = output.index(max_conf)
            now_label = active_labels[max_idx]

            # 打印输出供调试用，包含准确率
            print(f"[{current_count}/{MAX_TOTAL_TIMES}] 识别结果: {now_label} (准确率: {max_conf*100:.1f}%)")

            # 记录这段时间内准确率最高的标签，用作超时备选
            if max_conf > best_conf:
                best_conf = max_conf
                best_label = now_label

            # 连续相同逻辑判断
            if now_label == last_label:
                consecutive_count += 1
            else:
                consecutive_count = 1
                last_label = now_label

            # 如果连续2次识别结果一致，立刻通过串口发送
            if consecutive_count >= 2:
                uart.write(f"{now_label}\n")
                time.sleep_ms(10)
                print(f"满足条件(连续2次识别为 {now_label}) → 输出: {now_label}")
                is_running = 0
                break

        # 如果尝试了 MAX_TOTAL_TIMES 次依然没有连续两次相同的，输出其中置信度最高的一次
        if is_running and current_count >= MAX_TOTAL_TIMES:
            final_out = best_label if best_label else "None"
            uart.write(f"{final_out}\n")
            time.sleep_ms(10)
            print(f"达到最大尝试上限，未能连续相同 → 降级输出最优结果: {final_out}")
            is_running = 0

        if not is_running:
            print("===== 识别任务结束，等待下一次指令 =====")

    except Exception as e:
        print("识别运行异常:", e)
        is_running = 0
