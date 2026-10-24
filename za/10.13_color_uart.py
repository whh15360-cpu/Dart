import sensor
import time
import struct
from pyb import UART
import mjpeg
import os

# =========================
# 全局配置
# =========================

# 串口参数
uart = UART(3, 115200)

# 模式
DEBUG = True
RECORD_ENABLE = False  # False\True
RECORD_ENABLE = DEBUG and RECORD_ENABLE

# 时间
SKIP_FRAMES_MS = 1500  # 跳帧时间ms，上电后跳过时间
T_START = 80  # 时间ms，起飞后多久开始检测，接受到电控信息后跳过的时间
RUN_TIME = 5000 if not DEBUG else 60000  # 时间设置为5s，超时自动静默

# 图像与ROI设置
READOUT_ROI = (0, 100, 640, 480)  # 硬件ROI，先设置100尝试
BLOB_ROI = (0, 0, 320, 240)  # 寻找ROI
INIT_ROI = (0, 0, 320, 240)  # 初始ROI，丢失后重新寻找

# LAB 颜色空间阈值（针对绿色引导灯，保持彩色模式）
THRESHOLDS = [(3, 53, -11, 80, 2, 66)]

# 动态roi范围
FOUND_RANGE_X = 100
FOUND_RANGE_Y = 100

# 传感器参数
FRAME_SIZE = sensor.QVGA
PIXFORMAT = sensor.RGB565  # 维持彩色模式不变

EXPOSURE_US = 1500
GAIN_DB = 18

# 计算数据
ANGLE_SCALE = 100
ANGLE_LIMIT = 32767

MIN_PIXELS = 5
MAX_PIXELS = 30000

IMG_CX = 160.0  # 320 / 2，中心点
IMG_CY = 120.0  # 240 / 2

DEG_PER_PX_X = 0.2531  # 水平 81° / 320 px
DEG_PER_PX_Y = 0.2750  # 垂直 66° / 240 px

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
            except Exception:
                continue
    return "%s%02d%s" % (base_name, max_num + 1, ext)


def calculate_confidence(blob):  # 置信度
    if not blob:
        return 0
    w = blob.w
    h = blob.h
    pixels = blob.pixels

    # 长宽比筛选
    aspect_ratio = (h / w) if w > h else (w / h)
    if aspect_ratio < 0.30:
        return 0

    # 计算当前 blob 占其外接矩形的比例
    fill_ratio = pixels / (w * h)

    # 偏差计算：越接近 0.785 分数越高
    dev = fill_ratio - TARGET_FILL_RATIO
    if dev < 0:
        dev = -dev

    if dev > 0.38:
        return 0

    shape_score = int((aspect_ratio * 0.5 + (1.0 - dev * 2.5) * 0.5) * 100)

    # 远距离微弱光斑保护
    if pixels < 20:
        return max(0, min(80, shape_score))

    return max(0, min(100, shape_score))


def init_sensor():
    sensor.reset()
    sensor.set_pixformat(PIXFORMAT)
    sensor.set_framesize(FRAME_SIZE)
    sensor.set_framebuffers(3)    # 缓冲流水线
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


"""
def draw_target_cross(img, blob):
    if img and blob:
        # 彩色模式下绘制黑绿色双层十字准星
        img.draw_cross((blob.cx, blob.cy), color=(0, 0, 0), size=10, thickness=3)
        img.draw_cross((blob.cx, blob.cy), color=(0, 255, 0), size=10, thickness=1)
"""
# =========================
# 主程序
# =========================


init_sensor()
clock = time.clock()

# 录像初始化
m = mjpeg.Mjpeg(get_next_file("record", "avi")) if RECORD_ENABLE else None

while True:
    sensor.sleep(True)

    while True:
        val = uart.readchar()
        if val == ord('1') or val == 1:
            break
        time.sleep_ms(5)

sensor.sleep(False)
sensor.skip_frames(time=50)
start = time.ticks_ms()
frame_cnt = 0

while time.ticks_diff(time.ticks_ms(), start) < RUN_TIME:
    if DEBUG:
        clock.tick()

    img = sensor.snapshot()
    frame_cnt += 1
    t_ms = time.ticks_diff(time.ticks_ms(), start)

    if t_ms > T_START:
        blobs = img.find_blobs(
            THRESHOLDS,
            roi=CUR_ROI,
            merge=True,
            pixels_threshold=MIN_PIXELS,
            area_threshold=MIN_PIXELS,
            x_stride=2,
            y_stride=2
        )

        max_blob = None

        if blobs:
            max_p = 0
            for b in blobs:
                p = b.pixels
                if p <= MAX_PIXELS and p > max_p:
                    max_p = p
                    max_blob = b

        if max_blob:
            confidence = calculate_confidence(max_blob)
            if confidence > 0:
                yaw = (max_blob.cx - IMG_CX) * DEG_PER_PX_X
                pitch = (IMG_CY - max_blob.cy) * DEG_PER_PX_Y

                send_blob_uart(yaw=yaw, pitch=pitch, status=0x01, confidence=confidence)
                build_found_roi(max_blob, CUR_ROI)
            else:
                send_blob_uart(yaw=0, pitch=0, status=0x00, confidence=0)
                reset_roi_inplace(CUR_ROI)

        else:  # 未找到
            send_blob_uart(yaw=0, pitch=0, status=0x00, confidence=0)
            reset_roi_inplace(CUR_ROI)

    if DEBUG:
        # if 'max_blob' in locals() and max_blob:
        # draw_target_cross(img, max_blob)
        if RECORD_ENABLE and (frame_cnt % 5 == 0) and m:
            m.write(img)

if RECORD_ENABLE and m:
    m.close()
