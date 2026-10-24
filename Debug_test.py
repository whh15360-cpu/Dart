import sensor
import time
import struct
from pyb import UART, LED
import mjpeg
import os
import math


# =========================
# 全局配置
# =========================

# 串口参数
uart = UART(3, 115200)
led_green = LED(2)  # 检验是否开启动态roi，绿灯为开启

# 模式
DEBUG = False
RECORD_ENABLE = True  # False\True

# 时间
SKIP_FRAMES_MS = 1500  # 跳帧时间ms，上电后跳过时间
T_START = 80  # 时间ms，起飞后多久开始检测，接受到电控信息后跳过的时间

RECORD_TIME = 600000 if DEBUG else 3500

# 图像与ROI设置
READOUT_ROI = (0, 0, 640, 480)  # 硬件ROI
# READOUT_ROI = (0, 60, 320, 240)

BLOB_ROI = (0, 0, 320, 240)  # 寻找ROI

INIT_ROI = (0, 0, 320, 240)  # 初始ROI，丢失后重新寻找

THRESHOLDS = [(160, 255)]  # 灰度区间
# 动态roi范围
FOUND_RANGE_X = 100  # 实测
FOUND_RANGE_Y = 100

# 传感器参数
FRAME_SIZE = sensor.QVGA
# PIXFORMAT = sensor.RGB565
PIXFORMAT = sensor.GRAYSCALE
EXPOSURE_US = 500  # 联调,500
GAIN_DB = 20

# 计算数据
# F_PX = 225.14  # 等效像素焦距，QVGA下，HFOV_DEG = 70.8

ANGLE_SCALE = 100
ANGLE_LIMIT = 32767

MIN_PIXELS = 15
MAX_PIXELS = 20000

IMG_CX = 164.01  # 中心点,相机标定所得，后续可再次标定
IMG_CY = 116.42
FX = 238.15
FY = 238.32

# RAD_TO_DEG = 57.29578  # 180 / pi 预计算常数
TARGET_FILL_RATIO = 0.785  # 置信度填充定值

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
def calculate_confidence(blob):  # 置信度
    if not blob:
        return 0
    w = blob.w
    h = blob.h
    pixels = blob.pixels

    # 长宽比筛选
    aspect_ratio = (h / w) if w > h else (w / h)
    if aspect_ratio < 0.30:   # 极限0.45，越高越严格
        return 0

    # 计算当前 blob 占其外接矩形的比例
    fill_ratio = pixels / (w * h)

    # 偏差计算：越接近 0.785 分数越高,dev 范围约为 0.0 ~ 0.4
    dev = fill_ratio - TARGET_FILL_RATIO
    if dev < 0:
        dev = -dev

    if dev > 0.38:  # 极限0.35，越低越严格
        return 0

    # 综合形状打分 (0 ~ 100)
    # 长宽比权重 50% + 填充率匹配度 50%
    shape_score = int((aspect_ratio * 0.5 + (1.0 - dev * 2.5) * 0.5) * 100)

    # 远距离微弱光斑保护
    # 如果极远 (如像素点在 8~20 之间)，只要形状满足圆，不扣分，保底给 70+ 置信度
    if pixels < 20:  # 考虑，如果后续太宽松就删除
        return max(0, min(80, shape_score))

    return max(0, min(100, shape_score))
"""


def calculate_confidence(blob):
    w = blob.w
    h = blob.h
    pixels = blob.pixels
    if w <= 0 or h <= 0:
        return 0
    aspect = (h / w) if w > h else (w / h)

    if pixels < 90:  # 越大越宽松
        # 远处 / 小目标
        # min_aspect = 0.20
        if aspect < 0.50 or blob.elongation > 0.45 or blob.roundness < 0.50:  # 越大越严苛，对圆形的
            return 0
        max_deviation = 0.30  # 越大越宽松，对矩形的
    else:
        # 方案B：近处 / 大目标
        # min_aspect = 0.50
        if aspect < 0.75:
            return 0
        if blob.elongation > 0.32 or blob.roundness < 0.68:
            return 0
        max_deviation = 0.090  # 越大越宽松

    deviation = abs(pixels / (w * h) - TARGET_FILL_RATIO)
    if deviation > max_deviation:
        return 0

    score = int((aspect * 0.4 + blob.roundness * 0.4 + (1.0 - deviation * 2.5) * 0.2) * 100)
    if score < 0:
        return 0
    upper = 80 if pixels < 20 else 100
    return upper if score > upper else score


def init_sensor():
    sensor.reset()
    sensor.set_pixformat(PIXFORMAT)
    sensor.set_framesize(FRAME_SIZE)
    sensor.set_framebuffers(2)    # 缓冲流水线
    sensor.ioctl(sensor.IOCTL_SET_READOUT_WINDOW, READOUT_ROI)
    sensor.set_windowing(BLOB_ROI)

    sensor.set_auto_gain(False, gain_db=GAIN_DB)
    sensor.set_auto_whitebal(False)
    sensor.set_auto_exposure(False, exposure_us=EXPOSURE_US)

    sensor.skip_frames(time=SKIP_FRAMES_MS)


def build_found_roi(blob, roi_buf):
    # 使用 max(0, ...) 分别对 x 和 y 进行限幅，防止出现负坐标
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
    """
    yaw, pitch: 角度值（单位：度），内部会 ×100 转成 int16
    status: 0x01 找到目标，0x00 脱靶
    confidence: 0~100
    """
    yaw_int = max(-ANGLE_LIMIT, min(ANGLE_LIMIT, int(yaw * ANGLE_SCALE)))
    pitch_int = max(-ANGLE_LIMIT, min(ANGLE_LIMIT, int(pitch * ANGLE_SCALE)))

# "<" 代表小端模式，BBhhBB 分别对应：头1(1B), 头2(1B), Yaw(2B), Pitch(2B), 状态(1B), 置信度(1B)

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
    print("Send Hex:", UART_BUF.hex())


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
is_roi_tracking = False  # 记录是否

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
            yaw = math.degrees(math.atan((max_blob.cx - IMG_CX) / FX))
            pitch = math.degrees(math.atan((max_blob.cy - IMG_CY) / FY))
            send_blob_uart(yaw=yaw, pitch=pitch, status=0x01, confidence=best_conf)
            build_found_roi(max_blob, CUR_ROI)
            if DEBUG:
                led_green.on()
        else:
            send_blob_uart(yaw=0, pitch=0, status=0x00, confidence=0)
            reset_roi_inplace(CUR_ROI)
            if DEBUG:
                led_green.off()

    if DEBUG:
        if max_blob:
            draw_target_cross(img, max_blob)
        if RECORD_ENABLE and (frame_cnt % 5 == 0) and m:
            m.write(img)


if RECORD_ENABLE and m:
    m.close()
sensor.sleep(True)  # 进入极低功耗休眠模式
