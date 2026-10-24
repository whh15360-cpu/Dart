import sensor
import time
import math
import struct
from pyb import UART

# ==================== 1. 串口初始化 ====================
uart = UART(3, 115200)          # 按实际接线修改

# ==================== 2. 参数预设（只算一次） ====================
ROI_X, ROI_Y = 100, 60
ROI_W, ROI_H = 120, 120         # 硬件裁剪窗口

CENTER_X = 60.0
CENTER_Y = 60.0

FOV_DEG = 70.0
F_PX = 228.5                    # 等效像素焦距

ANGLE_SCALE = 100
ANGLE_LIMIT = 32767

THRESHOLDS = [(180, 255)]       # 高亮块阈值
MIN_PIXELS = 5
MAX_PIXELS = 10000   # 76280

# ==================== 3. 解算 + 打包 + 发送 ====================


def calculate_and_send(cx, cy, uart):
    """输入目标质心 (cx, cy)，输出 9 字节协议帧并返回角度值"""
    dx = cx - CENTER_X
    dy = CENTER_Y - cy           # 图像 Y 向下，物理 Y 向上

    # 几何角度解算
    yaw = math.degrees(math.atan2(dx, F_PX))
    pitch = math.degrees(math.atan2(dy, F_PX))

    # 缩放为整型并钳位
    yaw_int = int(yaw * ANGLE_SCALE)
    pitch_int = int(pitch * ANGLE_SCALE)

    if yaw_int > ANGLE_LIMIT:
        yaw_int = ANGLE_LIMIT
    if yaw_int < -ANGLE_LIMIT:
        yaw_int = -ANGLE_LIMIT
    if pitch_int > ANGLE_LIMIT:
        pitch_int = ANGLE_LIMIT
    if pitch_int < -ANGLE_LIMIT:
        pitch_int = -ANGLE_LIMIT

    head1, head2, status, tail = 0x55, 0xAA, 0x01, 0x0D
    confidence = 80

    checksum = (
        head1 + head2
        + (yaw_int & 0xFF) + ((yaw_int >> 8) & 0xFF)
        + (pitch_int & 0xFF) + ((pitch_int >> 8) & 0xFF)
        + status
        + confidence
    ) & 0xFF

    packet = struct.pack(
        "<BBhhBBBB",
        head1, head2,
        yaw_int, pitch_int,
        status, confidence, checksum, tail
    )
    uart.write(packet)
    # 在发送的同时，把封好的字节包转成 Hex 字符串打印到电脑 IDE 终端
    # print("Send Hex:", packet.hex())

    # 【修改点 1】：返回解算出的实际角度，供终端显示使用
    return yaw, pitch


def send_lost(uart):
    """丢靶数据包"""
    confidence = 75
    head1, head2, status, tail = 0x55, 0xAA, 0x00, 0x0D
    yaw_int = 0
    pitch_int = 0
    checksum = (
        head1 + head2
        + (yaw_int & 0xFF) + ((yaw_int >> 8) & 0xFF)
        + (pitch_int & 0xFF) + ((pitch_int >> 8) & 0xFF)
        + status
        + confidence
    ) & 0xFF
    uart.write(struct.pack("<BBhhBBBB",
                           head1, head2, yaw_int, pitch_int,
                           status, confidence, checksum, tail))

# ==================== 4. 传感器初始化 ====================


sensor.reset()
sensor.set_pixformat(sensor.GRAYSCALE)      # 单通道，最快
sensor.set_framesize(sensor.QVGA)           # 320x240 基础
sensor.set_framebuffers(3)                  # 双缓冲流水线
sensor.ioctl(sensor.IOCTL_SET_READOUT_WINDOW, (ROI_X, ROI_Y, ROI_W, ROI_H))
sensor.skip_frames(time=500)
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)
sensor.set_auto_exposure(False, exposure_us=500)

# ==================== 5. 主循环 ====================

clock = time.clock()

last_print = time.ticks_ms()

current_yaw = 0.0
current_pitch = 0.0
target_found = False

while True:
    clock.tick()
    img = sensor.snapshot()

    try:
        blobs = img.find_blobs(
            THRESHOLDS,
            merge=False,
            pixels_threshold=MIN_PIXELS,
            area_threshold=MIN_PIXELS
        )

        max_blob = None
        max_p = 0

        if blobs:
            for b in blobs:
                p = b.pixels
                if p <= MAX_PIXELS and p > max_p:
                    max_p = p
                    max_blob = b

        if max_blob is not None and max_blob.pixels <= MAX_PIXELS:
            current_yaw, current_pitch = calculate_and_send(max_blob.cx, max_blob.cy, uart)
            target_found = True
        else:
            send_lost(uart)
            target_found = False

    except Exception as e:
        print("ERROR:", e)
        send_lost(uart)
        target_found = False
    # 【修改点 3】：每 500ms 在终端打印一次 FPS 和 偏转角度
    now = time.ticks_ms()
    if time.ticks_diff(now, last_print) > 1000:
        if target_found:
            # %+.2f 可以自动带 +/- 符号，直观看到偏左偏右/偏上偏下
            print("FPS: %.1f | Yaw: %+.2f° | Pitch: %+.2f°" % (clock.fps(), current_yaw, current_pitch))
        else:
            print("FPS: %.1f | 目标丢失 (Lost)" % clock.fps())
        last_print = now
