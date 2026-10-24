import sensor
import time
from pyb import UART
import mjpeg
import os


# =========================
# 全局配置
# =========================

# 串口参数
uart = UART(3, 115200)

DEBUG = False
RECORD_ENABLE = False

UART_WAIT_TIMEOUT_MS = 3000   # 新增：等串口字节的最长时间，超时不再死等
DATA_START_TIMEOUT_MS = 5000  # 新增：接收飞行数据时等首字节的超时

CAPTURE_TIME = 800

if DEBUG:
    RECORD_TIME = 75000
else:
    RECORD_TIME = 900

RECORD_TIME_PHASE2 = 2000

DATA_TARGET_SIZE = 40 * 1024
DATA_READ_CHUNK = 64
DATA_IDLE_TIMEOUT_MS = 1000

def wait_uart_byte(uart, targets, timeout_ms=UART_WAIT_TIMEOUT_MS):
    """新增：等待 targets 里的任意一个字节，超时返回 None（替代原来的 while True 死等）"""
    t0 = time.ticks_ms()
    while time.ticks_diff(time.ticks_ms(), t0) < timeout_ms:
        if uart.any():
            b = uart.read(1)
            if b:
                v = ord(b)
                if v in targets:
                    return v
        time.sleep_ms(1)
    return None

mode = "yaw"

if DEBUG is False:
    # ... existing code ...
    v = wait_uart_byte(uart, (0x11, 0x22))      # 原为 while True 阻塞
    if v == 0x22:
        mode = "pitch"
    elif v == 0x11:
        mode = "yaw"
    else:
        print("UART mode handshake timeout, use default:", mode)

# 图像与ROI设置
READOUT_ROI = (0, 700, 640, 480)  # IOCTL ROI

BLOB_ROI = (0, 0, 320, 240)  # 裁切ROI
BLOB_ROI_PHASE2 = (0, 0, 320, 240)

INIT_ROI = (0, 0, 320, 240)
INIT_ROI_PHASE2 = (0, 0, 320, 240)

FOUND_RANGE_X = 40
FOUND_RANGE_Y = 40

THRESHOLDS1 = [(9, 100, -102, -25, 0, 127)]  # 联调
# THRESHOLDS1 = [(31, 100, -102, -7, -16, 127)]

T_START = 80

# 传感器参数
FRAME_SIZE = sensor.QVGA
FRAME_SIZE_PHASE2 = sensor.QVGA
PIXFORMAT = sensor.RGB565
EXPOSURE_US = 800  # 联调

SKIP_FRAMES_MS = 1500

COMPETITION_MODE = False


# =========================
# 数据结构
# =========================
class BlobData:
    def __init__(self, x=0, y=0, w=0, h=0):
        self.x = x
        self.y = y
        self.w = w
        self.h = h

    def area(self):
        return self.w * self.h


def wait_for_uart_trigger(uart):
    """发射后开始录制：DEBUG=False 时等待串口有数据"""
    global COMPETITION_MODE
    if DEBUG is False:
        if wait_uart_byte(uart, (0xAB,)) is None:   # 原为 while True 阻塞
            print("trigger timeout, continue anyway")

        sensor.sleep(False)
        print("sensor awake")
        t0 = time.ticks_ms()

        while True:
            sensor.snapshot()
            if uart.any():
                data_buf = uart.read(1)
                data = ord(data_buf)
                if data == 0x55:
                    COMPETITION_MODE = False
                    break
                if data == 0xFF:
                    COMPETITION_MODE = True
                    break
            if time.ticks_diff(time.ticks_ms(), t0) > UART_WAIT_TIMEOUT_MS:
                print("mode select timeout, keep COMPETITION_MODE =", COMPETITION_MODE)
                break
    else:
        sensor.sleep(False)

# =========================
# 工具函数
# =========================


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
            except Exception:
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
    sensor.set_framesize(FRAME_SIZE)
    sensor.set_pixformat(PIXFORMAT)
    sensor.ioctl(sensor.IOCTL_SET_READOUT_WINDOW, READOUT_ROI)
    sensor.set_windowing(BLOB_ROI)

    sensor.set_auto_gain(False, gain_db=20)
    sensor.set_auto_whitebal(False)
    sensor.set_auto_exposure(False, exposure_us=EXPOSURE_US)

    sensor.skip_frames(time=SKIP_FRAMES_MS)


def find_max_blob(img, roi):
    max_blob = BlobData()
    cnt = 0
    found_flag = 0

    blobs = img.find_blobs([THRESHOLDS1[0]], roi=roi)

    for blob in blobs:
        rect = blob.rect()

        if cnt == 0:
            max_blob.x, max_blob.y, max_blob.w, max_blob.h = rect
            cnt += 1

        if cnt > 0:
            c_blob = BlobData(*rect)
            if c_blob.area() > max_blob.area():
                max_blob = c_blob

        found_flag = 1

    return found_flag, max_blob


def build_found_roi(blob):
    # 限幅，防止 blob 贴着 ROI 边缘时出现负坐标，find_blobs 会直接报错
    x = max(0, blob.x - FOUND_RANGE_X)
    y = max(0, blob.y - FOUND_RANGE_Y)
    return (
        x,
        y,
        2 * FOUND_RANGE_X + blob.w,
        2 * FOUND_RANGE_Y + blob.h
    )


def send_blob_uart(uart, blob):
    data = bytearray([
        0xff,
        (blob.x >> 8) & 0xff, blob.x & 0xff,
        (blob.y >> 8) & 0xff, blob.y & 0xff,
        blob.w, blob.h
    ])
    uart.write(data)


def rename_record_file(uart, filename):
    while not uart.any():
        pass

    timedate = uart.read(15).decode("ASCII")
    print(timedate)
    if timedate[0] == "2":
        os.rename(filename, "record_%s.avi" % timedate)


def receive_flight_data_csv(uart):
    print("Start receiving flight data...")

    # filename = get_next_file("data", ".csv")
    filename = "data.csv"
    print("Saving to:", filename)

    buf = ""
    started = False
    last_rx = time.ticks_ms()

    with open(filename, "w") as f:
        while True:
            if uart.any():
                started = True
                last_rx = time.ticks_ms()

                raw = uart.read(DATA_READ_CHUNK)
                if raw:
                    buf += raw.decode("ASCII", "ignore")

                    while "\n" in buf:
                        line, buf = buf.split("\n", 1)

                        if line.strip():   # 去空行
                            f.write(line + "\n")

            if started and time.ticks_diff(time.ticks_ms(), last_rx) > DATA_IDLE_TIMEOUT_MS:
                break

            time.sleep_ms(5)

        if buf.strip():
            f.write(buf + "\n")

    print("Flight data saved:", filename)


def draw_overlay(img, blob, found_flag, cur_roi, fps, t_ms):
    if found_flag == 1:
        img.draw_rectangle(blob.x, blob.y, blob.w, blob.h, color=(255, 0, 0))
        img.draw_string(
            2,
            18,
            "Found! X=%d Y=%d" % (blob.x + blob.w // 2, blob.y + blob.h // 2),
            scale=1
        )

    img.draw_string(2, 2, "FPS:%.1f" % fps, color=(255, 0, 0), scale=1)
    img.draw_string(2, 10, "t=%d ms" % t_ms, color=(255, 255, 0), scale=1)
    img.draw_rectangle(cur_roi[0], cur_roi[1], cur_roi[2], cur_roi[3], color=(0, 0, 255))


# =========================
# 主程序
# =========================

# os.chdir("record")

init_sensor()

uart.write(bytearray([0xff, ord("R"), ord("P"), ord("D"), ord("A"), ord("R"), ord("T")]))
print("init successful")
clock = time.clock()


# 录像初始化
if RECORD_ENABLE:
    filename = get_next_file("record", ".avi")
    m = mjpeg.Mjpeg(filename)

img = sensor.snapshot()
if RECORD_ENABLE:
    m.write(img)

sensor.sleep(True)

wait_for_uart_trigger(uart)

start = time.ticks_ms()
frame_cnt = 0

# 0: idle 1: search 2: found
find_status = 0
cur_roi = INIT_ROI
is_captured = False

while time.ticks_diff(time.ticks_ms(), start) < RECORD_TIME:
    clock.tick()
    t_ms = time.ticks_ms() - start
    img = sensor.snapshot()

    max_blob = BlobData()
    found_flag = 0

    if t_ms > T_START:
        found_flag, max_blob = find_max_blob(img, cur_roi)

        if found_flag == 1:
            cur_roi = build_found_roi(max_blob)
        elif cur_roi != INIT_ROI:
            cur_roi = INIT_ROI

    if found_flag == 1:
        send_blob_uart(uart, max_blob)
        draw_found_flag = 1
        found_flag = 0
    else:
        draw_found_flag = 0

    fps = clock.fps()
    draw_overlay(img, max_blob, draw_found_flag, cur_roi, fps, t_ms)

    if frame_cnt % 5 == 0:
        if RECORD_ENABLE:
            m.write(img)
            print("write img")

    frame_cnt += 1

    if RECORD_ENABLE:
        print(clock.fps())

READOUT_ROI_PHASE2 = (0, 160+max_blob.y*2+max_blob.h, 640, 480)
sensor.ioctl(sensor.IOCTL_SET_READOUT_WINDOW, READOUT_ROI_PHASE2)

while time.ticks_diff(time.ticks_ms(), start) < RECORD_TIME_PHASE2:
    clock.tick()
    t_ms = time.ticks_ms() - start
    img = sensor.snapshot()

    img_width = img.width()
    img_height = img.height()

    max_blob = BlobData()
    found_flag = 0

    if t_ms > T_START:
        found_flag, max_blob = find_max_blob(img, cur_roi)

        if found_flag == 1:
            cur_roi = build_found_roi(max_blob)
        elif cur_roi != INIT_ROI_PHASE2:
            cur_roi = INIT_ROI_PHASE2

    if found_flag == 1:
        send_blob_uart(uart, max_blob)
        draw_found_flag = 1
        found_flag = 0
    else:
        draw_found_flag = 0

    fps = clock.fps()
    draw_overlay(img, max_blob, draw_found_flag, cur_roi, fps, t_ms)

    if frame_cnt % 5 == 0:
        if RECORD_ENABLE:
            m.write(img)
            print("write img")

    frame_cnt += 1

    if RECORD_ENABLE:
        print(clock.fps())

if RECORD_ENABLE:
    m.close()

if RECORD_ENABLE and (not COMPETITION_MODE):
    rename_record_file(uart, filename)

if COMPETITION_MODE:
    receive_flight_data_csv(uart)
