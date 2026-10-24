import sensor
import image
import time

# 初始化相机
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.set_auto_exposure(False, exposure_us=2500)  # 设置和你主程序一样的曝光
sensor.set_auto_gain(False, gain_db=16)
sensor.skip_frames(time=1000)

print("请将绿色引导灯对准画面正中心，3秒后自动采样...")
time.sleep(5)

img = sensor.snapshot()

# 定义一个位于画面正中央的 20x20 采样区域（你可以根据灯在画面的位置调整这个矩形）
target_roi = (150, 110, 20, 20)

# 在画面上画出采样框，方便你在 IDE 预览里确认是否套住了绿灯
img.draw_rectangle(target_roi, color=(255, 0, 0))

# 获取该区域的直方图（在 RGB565 模式下，会自动处理为 LAB 三通道统计）
hist = img.get_histogram(roi=target_roi)

# 利用第 5 百分位和第 95 百分位自动掐头去尾，过滤离群杂光
lo = hist.get_percentile(0.05)
hi = hist.get_percentile(0.95)

# 打印出可以直接复制到主程序的 6 元组阈值
print("--- 标定成功！请复制下方阈值到主程序中 ---")
print(
    "THRESHOLDS = [({}, {}, {}, {}, {}, {})]".format(
        lo.l_value,
        hi.l_value,
        lo.a_value,
        hi.a_value,
        lo.b_value,
        hi.b_value,
    )
)
