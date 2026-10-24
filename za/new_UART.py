import sensor
import time
import math
import struct
import gc
from pyb import UART

# ==================== 1. 串口初始化 ====================
uart = UART(3, 115200)

# ==================== 2. 参数预设 ====================
ROI_X, ROI_Y = 100, 60
ROI_W, ROI_H = 120, 120
CENTER_X = 60.0
CENTER_Y = 60.0
FOV_DEG = 70.0
F_PX = 228.5
ANGLE_SCALE = 100
ANGLE_LIMIT = 32767
THRESHOLDS = [(100, 255)]
MIN_PIXELS = 10
MAX_PIXELS = 30000

# ==================== 3. 置信度计算 ====================


def calculate_confidence(pixel_count):
    """仅根据像素点数计算置信度 (0 ~ 100)"""
    if pixel_count <= MIN_PIXELS:
        return 0
    if pixel_count >= MAX_PIXELS:
        return 100
    confidence = int((pixel_count - MIN_PIXELS) * 100.0 / (MAX_PIXELS - MIN_PIXELS))
    return max(0, min(100, confidence))

# ==================== 4. 解算 + 打包 + 发送 ====================


def calculate_and_send(cx, cy, confidence, uart):
    dx = cx - CENTER_X
    dy = CENTER_Y - cy
    yaw = math.degrees(math.atan2(dx, F_PX))
    pitch = math.degrees(math.atan2(dy, F_PX))
    yaw_int = int(yaw * ANGLE_SCALE)
    pitch_int = int(pitch * ANGLE_SCALE)
    yaw_int = max(-ANGLE_LIMIT, min(ANGLE_LIMIT, yaw_int))
    pitch_int = max(-ANGLE_LIMIT, min(ANGLE_LIMIT, pitch_int))
    confidence = max(0, min(100, int(confidence)))
    head1, head2, status, tail = 0x55, 0xAA, 0x01, 0x0D
    prefix = struct.pack("<BBhhBB", head1, head2, yaw_int, pitch_int, status, confidence)
    checksum = sum(prefix) & 0xFF
    packet = prefix + struct.pack("<BB", checksum, tail)
    uart.write(packet)
    return yaw, pitch, confidence

# ==================== 5. 丢靶数据包 ====================


def send_lost(uart):
    head1, head2, status, tail = 0x55, 0xAA, 0x00, 0x0D
    prefix = struct.pack("<BBhhBB", head1, head2, 0, 0, status, 0)
    checksum = sum(prefix) & 0xFF
    packet = prefix + struct.pack("<BB", checksum, tail)
    uart.write(packet)

# ==================== 6. 传感器初始化 ====================


sensor.reset()
sensor.set_pixformat(sensor.GRAYSCALE)
sensor.set_framesize(sensor.QVGA)
sensor.set_framebuffers(2)   # 若板子不支持请注释掉
sensor.ioctl(
    sensor.IOCTL_SET_READOUT_WINDOW,
    (ROI_X, ROI_Y, ROI_W, ROI_H)
)
sensor.skip_frames(time=500)
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)
# 使用位置参数以兼容多数固件
sensor.set_auto_exposure(False, exposure_us=500)

# ==================== 7. 主循环 ====================
clock = time.clock()
last_print = time.ticks_ms()
current_yaw = 0.0
current_pitch = 0.0
current_confidence = 0
target_found = False

while True:
    clock.tick()
    gc.collect()
    img = sensor.snapshot()
    try:
        blobs = img.find_blobs(
            THRESHOLDS,
            merge=False,
            pixels_threshold=MIN_PIXELS
        )
        max_blob = None
        max_p = 0

        # ============ 寻找最大有效亮斑 ============
        if blobs:
            for b in blobs[:5]:
                p = b.pixels()
                if p > MAX_PIXELS or p <= max_p:
                    continue
                # 贴边过滤
                touch_edge = (
                    b.x() <= 1 or
                    b.y() <= 1 or
                    (b.x() + b.w()) >= (ROI_W - 1) or
                    (b.y() + b.h()) >= (ROI_H - 1)
                )
                if touch_edge:
                    continue
                w, h = b.w(), b.h()
                if w == 0 or h == 0:
                    continue
                aspect = float(w) / float(h) if w < h else float(h) / float(w)
                if aspect < 0.4:
                    continue
                max_p = p
                max_blob = b

        # ============ 找到目标 ============
        if max_blob is not None:
            confidence = calculate_confidence(max_blob.pixels())
            current_yaw, current_pitch, current_confidence = calculate_and_send(
                max_blob.cx(),
                max_blob.cy(),
                confidence,
                uart
            )
            target_found = True
        # ============ 丢靶 ============
        else:
            send_lost(uart)
            current_confidence = 0
            target_found = False

    except Exception as e:
        print("ERROR:", e)
        send_lost(uart)
        current_confidence = 0
        target_found = False

    # ============ 8. 终端输出 ============
    now = time.ticks_ms()
    if time.ticks_diff(now, last_print) > 500:
        if target_found:
            print(
                "FPS: %.1f | Yaw: %+.2f deg | Pitch: %+.2f deg | Confidence: %d"
                % (
                    clock.fps(),
                    current_yaw,
                    current_pitch,
                    current_confidence
                )
            )
        else:
            print("FPS: %.1f | Target Lost | Confidence: 0" % clock.fps())
        last_print = now
