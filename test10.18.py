import sensor
import time
import struct
import math
from pyb import UART
# from pyb import LED
import mjpeg
import os


# =========================
# 全局配置
# =========================

# 串口参数
uart = UART(3, 115200)
# led_green = LED(2)  # 检验是否开启动态roi，绿灯为开启

# 模式
DEBUG = True
RECORD_ENABLE = False  # False\True

# 时间
SKIP_FRAMES_MS = 1500  # 跳帧时间ms，上电后跳过时间
T_START = 80  # 时间ms，起飞后多久开始检测，接受到电控信息后跳过的时间

RECORD_TIME = 600000 if DEBUG else 3500

# 图像与ROI设置
READOUT_ROI = (0, 0, 640, 480)  # 硬件ROI
BLOB_ROI = (0, 0, 320, 240)  # 寻找ROI
INIT_ROI = (0, 0, 320, 240)  # 初始ROI，丢失后重新寻找

THRESHOLDS = [(200, 255)]  # 灰度区间（上限255）
# 动态roi范围
FOUND_RANGE_X = 100  # 实测
FOUND_RANGE_Y = 100

# 传感器参数
FRAME_SIZE = sensor.QVGA
PIXFORMAT = sensor.GRAYSCALE
EXPOSURE_US = 1000  # 联调，可实验500
GAIN_DB = 10

# 计算数据
ANGLE_SCALE = 100
ANGLE_LIMIT = 32767

MIN_PIXELS = 5
MAX_PIXELS = 20000

IMG_CX = 164.01  # 中心点,相机标定所得，后续可再次标定
IMG_CY = 116.42
FX = 238.15
FY = 238.32

TARGET_FILL_RATIO = 0.785  # 置信度填充定值

# DEG_PER_PX_X = 0.2531  # 水平 81° / 320 px
#DEG_PER_PX_Y = 0.2750  # 垂直 66° / 240 px

# 预分配
CUR_ROI = [0, 0, 320, 240]
UART_BUF = bytearray(10)
UART_BUF[9] = 0x0D


# =========================
# 工具函数
# =========================

def get_next_file(base_name, ext):
    max_num = 0
    for file in os.listdir():
        if file.startswith(base_name) and file.endswith(ext):
            try:
                num_str = file[len(base_name):-len(ext)]
                max_num = max(max_num, int(num_str))
            except Exception as e:
                print("error:", e)
                continue
    return "%s%02d%s" % (base_name, max_num + 1, ext)


"""
def calculate_confidence(blob):
    w = blob.w
    h = blob.h
    pixels = blob.pixels
    if w <= 0 or h <= 0:
        return 0

    aspect = (h / w) if w > h else (w / h)
    fill_ratio = pixels / (w * h)

    if pixels < 130:
        # 方案A：远处小目标（实测呈小长方形，放开填充率上限到 1.0，只杀细长线条）
        if aspect < 0.35 or fill_ratio < 0.45:
            return 0
        deviation = abs(fill_ratio - TARGET_FILL_RATIO)
    else:
        # 方案B：近处大目标（近似圆带微光晕）
        # 1. 杀横竖长条矩形/灯管边缘
        if aspect < 0.65:
            return 0
        # 2. 非对称填充率：
        #    < 0.52 杀斜45°线段与残缺框；
        #    > 0.87 杀正放的实心矩形/正方形无影灯；
        #    0.52 ~ 0.87 包容带微光晕的圆形引导灯
        if fill_ratio < 0.52 or fill_ratio > 0.87:
            return 0
        deviation = abs(fill_ratio - TARGET_FILL_RATIO)

    score = int((aspect * 0.5 + (1.0 - deviation * 2.0) * 0.5) * 100)
    if score < 10:
        return 10
    upper = 80 if pixels < 20 else 100
    return upper if score > upper else score
"""


def calculate_confidence(blob):
    w = blob.w
    h = blob.h
    pixels = blob.pixels
    if w <= 0 or h <= 0:
        return 0

    aspect = (h / w) if w > h else (w / h)
    fill_ratio = pixels / (w * h)

    if pixels < 80:
        # 方案A：远处小目标（实测呈小长方形，放开填充率上限，只杀细长线）
        if aspect < 0.20 or fill_ratio < 0.45:  # aspect < 0.35
            return 0
        max_el = 0.72  # 允许远处小长方形光团通过，max_el = 0.68
    else:
        # 方案B：近处大目标（近似圆带微光晕）
        if aspect < 0.65:
            return 0
        # 下限 0.52 杀斜细线，上限 0.87 杀正放的实心矩形无影灯
        if fill_ratio < 0.52 or fill_ratio > 0.87:
            return 0
        max_el = 0.45  # 严防倾斜 15°~35° 伪装成圆填充率的矩形无影灯！

    el = blob.elongation
    if el > max_el:
        return 0

    roundness_fast = 1.0 - el
    deviation = abs(fill_ratio - TARGET_FILL_RATIO)

    score = int((aspect * 0.4 + roundness_fast * 0.4 + (1.0 - deviation * 2.0) * 0.2) * 100)
    if score < 10:
        return 10
    upper = 80 if pixels < 20 else 100
    return upper if score > upper else score


def init_sensor():
    sensor.reset()
    sensor.set_pixformat(PIXFORMAT)
    sensor.set_framesize(FRAME_SIZE)
    sensor.set_framebuffers(3)    # 缓冲流水线1、2、3
    sensor.ioctl(sensor.IOCTL_SET_READOUT_WINDOW, READOUT_ROI)
    sensor.set_windowing(BLOB_ROI)

    sensor.set_auto_gain(False, gain_db=GAIN_DB)
    sensor.set_auto_whitebal(False)
    sensor.set_auto_exposure(False, exposure_us=EXPOSURE_US)

    sensor.skip_frames(time=SKIP_FRAMES_MS)


def build_found_roi(blob, roi_buf):
    cx = int(blob.cx)
    cy = int(blob.cy)

    x = cx - (FOUND_RANGE_X // 2)
    y = cy - (FOUND_RANGE_Y // 2)

    roi_buf[0] = max(0, min(x, 320 - FOUND_RANGE_X))
    roi_buf[1] = max(0, min(y, 240 - FOUND_RANGE_Y))
    roi_buf[2] = FOUND_RANGE_X
    roi_buf[3] = FOUND_RANGE_Y


def reset_roi_inplace(roi_buf):
    roi_buf[0] = 0
    roi_buf[1] = 0
    roi_buf[2] = 320
    roi_buf[3] = 240


def send_blob_uart(yaw=0.0, pitch=0.0, status=0x00, confidence=0):
    yaw_int = max(-ANGLE_LIMIT, min(ANGLE_LIMIT, int(yaw * ANGLE_SCALE)))
    pitch_int = max(-ANGLE_LIMIT, min(ANGLE_LIMIT, int(pitch * ANGLE_SCALE)))

    struct.pack_into(
        "<BBhhBB",
        UART_BUF,
        0,
        0x55, 0xAA,
        yaw_int, pitch_int,
        status & 0xFF,
        confidence & 0xFF
    )

    checksum = (UART_BUF[0] + UART_BUF[1] + UART_BUF[2] + UART_BUF[3] + UART_BUF[4] +
                UART_BUF[5] + UART_BUF[6] + UART_BUF[7]) & 0xFF

    UART_BUF[8] = checksum
    uart.write(UART_BUF)


def draw_target_cross(img, blob):
    if img and blob:
        img.draw_cross((blob.cx, blob.cy), color=0, size=10, thickness=3)
        img.draw_cross((blob.cx, blob.cy), color=255, size=10, thickness=1)


# =========================
# 主程序
# =========================

init_sensor()
clock = time.clock()

# 录像初始化
m = mjpeg.Mjpeg(get_next_file("record", "avi")) if RECORD_ENABLE else None

sensor.sleep(False)
sensor.skip_frames(time=50)
start = time.ticks_ms()
frame_cnt = 0

while time.ticks_diff(time.ticks_ms(), start) < RECORD_TIME:
    if DEBUG:
        clock.tick()

    img = sensor.snapshot()
    frame_cnt += 1
    t_ms = time.ticks_diff(time.ticks_ms(), start)

    max_blob = None
    best_conf = 0

    if t_ms > T_START:
        blobs = img.find_blobs(THRESHOLDS, roi=CUR_ROI, merge=False,
                               pixels_threshold=MIN_PIXELS, area_threshold=MIN_PIXELS)

        if blobs:
            max_p = 0
            for b in blobs:
                p = b.pixels
                if p <= MAX_PIXELS and p > max_p:
                    conf = calculate_confidence(b)
                    if conf > 0:
                        max_p = p
                        max_blob = b
                        best_conf = conf

        if max_blob:
            # yaw = (max_blob.cx - IMG_CX) * DEG_PER_PX_X
            # pitch = (IMG_CY - max_blob.cy) * DEG_PER_PX_Y
            yaw = math.degrees(math.atan((max_blob.cx - IMG_CX) / FX))
            pitch = math.degrees(math.atan((max_blob.cy - IMG_CY) / FY))
            send_blob_uart(yaw=yaw, pitch=pitch, status=0x01, confidence=best_conf)
            build_found_roi(max_blob, CUR_ROI)
        else:
            send_blob_uart(yaw=0, pitch=0, status=0x00, confidence=0)
            reset_roi_inplace(CUR_ROI)

    if DEBUG:
        if max_blob:
            draw_target_cross(img, max_blob)
        if RECORD_ENABLE and (frame_cnt % 5 == 0) and m:
            m.write(img)

if RECORD_ENABLE and m:
    m.close()
sensor.sleep(True)  # 进入极低功耗休眠模式
