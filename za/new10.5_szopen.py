import sensor
import time
import math
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
RECORD_ENABLE = False  # False

m = None

# 时间
SKIP_FRAMES_MS = 1500  # 跳帧时间ms，上电后跳过时间
T_START = 80  # 时间ms，起飞后多久开始检测，接受到电控信息后跳过的时间

if DEBUG:
    RECORD_TIME = 600000
else:
    RECORD_TIME = 3500

# 图像与ROI设置
READOUT_ROI = (0, 700, 640, 480)  # 硬件ROI
# READOUT_ROI = (0, 60, 320, 240)

BLOB_ROI = (0, 0, 320, 240)  # 寻找ROI

INIT_ROI = (0, 0, 320, 240)  # 初始ROI，丢失后重新寻找

# 动态roi范围
FOUND_RANGE_X = 100  # 实测
FOUND_RANGE_Y = 100

# 灰度区间
THRESHOLDS = [(60, 250)]

# 定义状态常量
STATUS_LOST = 0x00     # 脱靶
STATUS_FOUND = 0x01    # 找到目标

# 传感器参数
FRAME_SIZE = sensor.QVGA
# PIXFORMAT = sensor.RGB565
PIXFORMAT = sensor.GRAYSCALE
EXPOSURE_US = 3000  # 联调，可实验500,
GAIN_DB = 20

# 计算数据
F_PX = 225.14  # 等效像素焦距，QVGA下，HFOV_DEG = 70.8

ANGLE_SCALE = 100
ANGLE_LIMIT = 32767

MIN_PIXELS = 15
MAX_PIXELS = 40000

IMG_CX = 160.0  # 320 / 2，中心点
IMG_CY = 120.0  # 240 / 2

RAD_TO_DEG = 57.29578  # 180 / pi 预计算常数
TARGET_FILL_RATIO = 0.785  # 置信度填充定值
# =========================
# 工具函数
# =========================


def calculate_confidence(blob):  # 置信度
    if not blob:
        return 0

    w = blob.w
    h = blob.h
    pixels = blob.pixels

    # 长宽比筛选
    if w > h:
        aspect_ratio = h / w
    else:
        aspect_ratio = w / h

    # 如果长宽比低于 0.45 (比如 10x20 的扁条)，判定为飞线/反光干扰，直接返回 0
    if aspect_ratio < 0.308:   # 实测0.45，越高越严格
        return 0

    # 圆形填充率比对
    # 计算当前 blob 占其外接矩形的比例
    fill_ratio = pixels / (w * h)

    # 偏差计算：越接近 0.785 分数越高
    # dev 范围约为 0.0 ~ 0.4
    dev = fill_ratio - TARGET_FILL_RATIO
    if dev < 0:
        dev = -dev

    # 如果形状太怪异 (空洞严重或变成纯正方形)
    if dev > 0.38:  # 实测，0.35，越低越严格
        return 0

    # 综合形状打分 (0 ~ 100)
    # 长宽比权重 50% + 填充率匹配度 50%
    shape_score = int((aspect_ratio * 0.5 + (1.0 - dev * 2.5) * 0.5) * 100)

    # 远距离微弱光斑保护
    # 如果极远 (如像素点在 8~20 之间)，只要形状满足圆，不扣分，保底给 70+ 置信度
    if pixels < 20:  # 考虑，如果后续太宽松就删除
        return max(0, min(80, shape_score))

    return max(0, min(100, shape_score))


def calculate_angles(blob):
    if not blob:
        return 0.0, 0.0

    cx = blob.cx
    cy = blob.cy

    # 计算中心偏移 (像素)
    dx = cx - IMG_CX
    dy = IMG_CY - cy  # 图像 Y 轴向下，取反使上方为正

    # 三角函数精准映射
    yaw = math.atan2(dx, F_PX) * RAD_TO_DEG
    pitch = math.atan2(dy, F_PX) * RAD_TO_DEG

    return yaw, pitch


def get_next_file(base_name, ext):
    max_num = 0
    max_file = None
    reuse_file = None

    file_list = os.listdir()

    for file in file_list:
        if file.startswith(base_name) and file.endswith(ext):
            try:
                num_str = file[len(base_name):-len(ext)]
                file_num = int(num_str)
                if file_num > max_num:
                    max_num = file_num
                    max_file = file
            except Exception as e:
                # 打印错误信息（可选，方便调试）
                print("error:", e)
                # 跳过当前循环，进入下一帧
                continue

    if max_file and os.stat(max_file)[6] == 0:
        print("Found zero-size file:", max_file)
        reuse_file = max_file

    if reuse_file:
        return reuse_file
    else:
        return "%s%02d%s" % (base_name, max_num + 1, ext)


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


def find_max_blob(img, roi):
    blobs = img.find_blobs(
        THRESHOLDS,
        roi=roi,
        merge=False,
        pixels_threshold=MIN_PIXELS,
        area_threshold=MIN_PIXELS
    )

    max_blob = None
    max_p = 0

    for b in blobs:
        p = b.pixels

        if p <= MAX_PIXELS and p > max_p:
            max_p = p
            max_blob = b

    if max_blob is None:
        return 0, None

    return 1, max_blob


def build_found_roi(blob):
    # 使用 max(0, ...) 分别对 x 和 y 进行限幅，防止出现负坐标
    cx = int(blob.cx)
    cy = int(blob.cy)

    x = cx - (FOUND_RANGE_X // 2)
    y = cy - (FOUND_RANGE_Y // 2)

    x = max(0, min(x, 320 - FOUND_RANGE_X))
    y = max(0, min(y, 240 - FOUND_RANGE_Y))
    return (x, y, FOUND_RANGE_X, FOUND_RANGE_Y)


def send_blob_uart(uart, blob, yaw=0, pitch=0, status=STATUS_FOUND, confidence=0):
    """
    yaw, pitch: 角度值（单位：度），内部会 ×100 转成 int16
    status: 0x01 找到目标，0x00 脱靶
    confidence: 0~100
    """
    yaw_int = max(-ANGLE_LIMIT, min(ANGLE_LIMIT, int(yaw * ANGLE_SCALE)))
    pitch_int = max(-ANGLE_LIMIT, min(ANGLE_LIMIT, int(pitch * ANGLE_SCALE)))
    confidence = max(0, min(100, int(confidence)))

# 2. 先把前 8 个字节打包，利用内置 sum() 快速计算校验和
# "<" 代表小端模式，BBhhBB 分别对应：头1(1B), 头2(1B), Yaw(2B), Pitch(2B), 状态(1B), 置信度(1B)

    payload = struct.pack(
        "<BBhhBB",
        0x55, 0xAA,
        yaw_int, pitch_int,
        status & 0xFF,
        confidence & 0xFF
    )

    checksum = sum(payload) & 0xFF

    # 3. 拼接完整的 10 字节数据包并一次性写入串口
    packet = struct.pack("<BBhhBBBB", 0x55, 0xAA, yaw_int, pitch_int, status & 0xFF, confidence & 0xFF, checksum, 0x0D)
    uart.write(packet)


def draw_target_cross(img, blob):
    if img and blob:
        img.draw_cross((blob.cx, blob.cy), color=0, size=10, thickness=3)
        img.draw_cross((blob.cx, blob.cy), color=255, size=10, thickness=1)


"""
def draw_overlay(img, blob, found_flag, cur_roi, fps, t_ms):
    if found_flag == 1 and blob:
        img.draw_rectangle(blob.rect(), color=(255, 0, 0))
        img.draw_string(
            2,
            18,
            "Found! X=%d Y=%d" % (blob.cx(), blob.cy()),
            color=(0, 255, 0),
            scale=1
        )
    # img.draw_string(2, 2, "FPS:%.1f" % fps, color=(255, 0, 0), scale=1)
    # img.draw_string(2, 10, "t=%d ms" % t_ms, color=(255, 255, 0), scale=1)
    # img.draw_rectangle(cur_roi, color=(0, 255, 255))
"""

# =========================
# 主程序
# =========================

init_sensor()
clock = time.clock()

# 录像初始化
if RECORD_ENABLE:
    filename = get_next_file("record", ".avi")
    m = mjpeg.Mjpeg(filename)


sensor.sleep(False)
sensor.skip_frames(time=50)

start = time.ticks_ms()
frame_cnt = 0
cur_roi = INIT_ROI


while time.ticks_diff(time.ticks_ms(), start) < RECORD_TIME:
    if DEBUG:
        clock.tick()

    img = sensor.snapshot()
    frame_cnt += 1
    t_ms = time.ticks_diff(time.ticks_ms(), start)

    max_blob = None
    found_flag = 0

    if t_ms > T_START:
        found_flag, max_blob = find_max_blob(img, cur_roi)

        if found_flag and max_blob:  # 找到
            yaw, pitch = calculate_angles(max_blob)
            confidence = calculate_confidence(max_blob)

            if confidence > 0:
                send_blob_uart(uart, max_blob, yaw=yaw, pitch=pitch, status=0x01, confidence=confidence)
                cur_roi = build_found_roi(max_blob)  # 动态更新小框
            else:
                send_blob_uart(uart, None, yaw=0, pitch=0, status=0x00, confidence=0)
                if cur_roi != INIT_ROI:
                    cur_roi = INIT_ROI

        else:  # 未找到
            send_blob_uart(uart, None, yaw=0, pitch=0, status=0x00, confidence=0)
            if cur_roi != INIT_ROI:
                cur_roi = INIT_ROI

    # 2. DEBUG 调试模式与 SD 卡抽帧保存
    if DEBUG:
        fps = clock.fps() if DEBUG else 0
        # draw_overlay(img, max_blob, found_flag, cur_roi, fps, t_ms)
        if max_blob:
            draw_target_cross(img, max_blob)

        if RECORD_ENABLE and (frame_cnt % 5 == 0) and m:
            m.write(img)

if RECORD_ENABLE and m:
    m.close()
# 进入极低功耗休眠模式
sensor.sleep(True)
