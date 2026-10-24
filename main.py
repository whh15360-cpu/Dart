import sensor
import time
import struct
import gc
import pyb
import math
from pyb import UART


RUN_MS = 15000  # 实测
DETECT_DELAY_MS = 50
INIT_STABILIZE_MS = 1000

FRAME_SIZE = sensor.QVGA
PIXFORMAT = sensor.GRAYSCALE
READOUT_ROI = (0, 0, 640, 480)
OUTPUT_ROI = (0, 0, 320, 240)
THRESHOLDS = [(180, 250)]
EXPOSURE_US = 500
GAIN_DB = 20
FRAME_BUFFERS = 3  # 2或3

# Detection
IMG_W = 320
IMG_H = 240
IMG_CX = 164.01  # 中心点,相机标定所得，后续可再次标定
IMG_CY = 116.42
FX = 238.15
FY = 238.32
FOUND_W = 100  # 实测
FOUND_H = 100
MIN_PIXELS = 15  # 实测
MAX_PIXELS = 30000
TARGET_FILL_RATIO = 0.785
ANGLE_SCALE = 100
ANGLE_LIMIT = 32767

uart = UART(3, 115200)
TX = bytearray(10)
TX[0] = 0x55
TX[1] = 0xAA
TX[9] = 0x0D


def init_camera():
    sensor.reset()
    sensor.set_pixformat(PIXFORMAT)
    sensor.set_framesize(FRAME_SIZE)
    sensor.set_framebuffers(FRAME_BUFFERS)
    sensor.ioctl(sensor.IOCTL_SET_READOUT_WINDOW, READOUT_ROI)
    sensor.set_windowing(OUTPUT_ROI)
    sensor.set_auto_gain(False, gain_db=GAIN_DB)
    sensor.set_auto_whitebal(False)
    sensor.set_auto_exposure(False, exposure_us=EXPOSURE_US)
    sensor.skip_frames(time=INIT_STABILIZE_MS)
    sensor.sleep(True)


"""
def confidence_of(blob):
    w = blob.w()
    h = blob.h()
    pixels = blob.pixels()
    if w <= 0 or h <= 0:
        return 0
    aspect = (h / w) if w > h else (w / h)
    if aspect < 0.30:
        return 0
    deviation = abs(pixels / (w * h) - TARGET_FILL_RATIO)
    if deviation > 0.38:
        return 0
    score = int((aspect * 0.5 + (1.0 - deviation * 2.5) * 0.5) * 100)
    if score < 0:
        return 0
    upper = 80 if pixels < 20 else 100
    return upper if score > upper else score
"""


def confidence_of(blob):
    w = blob.w
    h = blob.h
    pixels = blob.pixels
    if w <= 0 or h <= 0:
        return 0
    aspect = (h / w) if w > h else (w / h)

    if pixels < 150:
        # 方案A：远处 / 小目标（放宽限制，防止因为畸变或锯齿而直接丢目标）
        min_aspect = 0.20  # 高则严苛
        max_deviation = 0.45  # 小则严苛
    else:
        # 方案B：近处 / 大目标（收紧标准，严防把场馆反光或干扰误认为靶标）
        min_aspect = 0.50
        max_deviation = 0.25

    if aspect < min_aspect:
        return 0

    deviation = abs(pixels / (w * h) - TARGET_FILL_RATIO)
    if deviation > max_deviation:
        return 0

    score = int((aspect * 0.5 + (1.0 - deviation * 2.5) * 0.5) * 100)
    if score < 0:
        return 0
    upper = 80 if pixels < 20 else 100
    return upper if score > upper else score


def send_result(yaw, pitch, status, confidence):
    yi = int(yaw * ANGLE_SCALE)
    pi = int(pitch * ANGLE_SCALE)
    if yi > ANGLE_LIMIT:
        yi = ANGLE_LIMIT
    elif yi < -ANGLE_LIMIT:
        yi = -ANGLE_LIMIT
    if pi > ANGLE_LIMIT:
        pi = ANGLE_LIMIT
    elif pi < -ANGLE_LIMIT:
        pi = -ANGLE_LIMIT
    struct.pack_into('<hhBB', TX, 2, yi, pi, status, confidence)
    checksum = 0
    for i in range(8):
        checksum += TX[i]
    TX[8] = checksum & 0xFF
    uart.write(TX)


def wait_start():
    while True:
        if uart.any():
            c = uart.readchar()
            if c == 49 or c == 1:  # ASCII '1' or raw 0x01
                return time.ticks_ms()
        pyb.wfi()


def run_once(start_ms):
    sensor.sleep(False)
    roi = OUTPUT_ROI
    lost_frames = 0
    MAX_LOST_FRAMES = 5

    try:
        while time.ticks_diff(time.ticks_ms(), start_ms) < RUN_MS:
            img = sensor.snapshot()
            elapsed = time.ticks_diff(time.ticks_ms(), start_ms)
            if elapsed >= RUN_MS:
                break
            if elapsed < DETECT_DELAY_MS:
                continue

            blobs = img.find_blobs(THRESHOLDS, roi=roi, merge=False,
                                   pixels_threshold=MIN_PIXELS,
                                   area_threshold=MIN_PIXELS)
            best = None
            best_score = 0

# 【优化】按综合得分（score）来挑选最佳目标
            for b in blobs:
                n = b.pixels
                if n <= MAX_PIXELS:
                    score = confidence_of(b)
                    if score > best_score:
                        best = b
                        best_score = score

            if best is not None and best_score > 0:
                lost_frames = 0  # 找到目标，重置丢失计数
                cx = best.cx
                cy = best.cy

                # 动态更新局部 ROI 窗口
                x = max(0, min(IMG_W - FOUND_W, cx - FOUND_W // 2))
                y = max(0, min(IMG_H - FOUND_H, cy - FOUND_H // 2))
                roi = (x, y, FOUND_W, FOUND_H)

                yaw = math.degrees(math.atan((cx - IMG_CX) / FX))
                pitch = math.degrees(math.atan((cy - IMG_CY) / FY))

                send_result(yaw, pitch, 1, best_score)
                continue

            # 未检测到有效目标时的处理
            lost_frames += 1
            if lost_frames >= MAX_LOST_FRAMES:
                roi = OUTPUT_ROI  # 连续多帧丢失后，放大回全图寻找
                lost_frames = 0

            send_result(0, 0, 0, 0)
    finally:
        sensor.sleep(True)
        gc.collect()


def main():
    init_camera()
    while True:
        start_ms = wait_start()
        run_once(start_ms)


main()
