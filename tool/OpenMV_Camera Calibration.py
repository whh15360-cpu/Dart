import sensor
import image
import time
import os

# 1. 传感器初始化
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.VGA)  # 可根据需要改回 sensor.HD
sensor.skip_frames(time=2000)


def get_next_file(base_name, ext):
    max_num = 0
    # 直接遍历根目录（插入SD卡后，根目录就是SD卡）
    for file in os.listdir():
        if file.startswith(base_name) and file.endswith(ext):
            try:
                num_str = file[len(base_name):-len(ext)]
                max_num = max(max_num, int(num_str))
            except Exception as e:
                continue
    # 直接返回文件名，不加 /sd/ 前缀
    return "%s%02d%s" % (base_name, max_num + 1, ext)


clock = time.clock()
count = 0
target_count = 30        # 总共需要拍摄的照片张数
interval = 8000          # 拍照间隔：3秒拍一张
last_capture_time = time.ticks_ms()

print("=== 自动标定拍照程序启动 ===")
print(f"程序将每隔 {interval / 1000} 秒自动拍一张照片，总共拍摄 {target_count} 张。")
print("请在每次倒计时结束前，将 2.8cm 棋盘格摆好并拿稳！\n")

while count < target_count:
    clock.tick()
    img = sensor.snapshot()

    current_time = time.ticks_ms()
    elapsed = current_time - last_capture_time
    remaining = max(0, int((interval - elapsed) / 1000) + 1)

    # 倒计时时间到，触发拍照
    if elapsed >= interval:
        # 动态获取下一个不重复的文件名（如 calib_01.jpg）
        filename = get_next_file("calib_", ".jpg")

        try:
            img.save(filename)
            count += 1
            print(f"-> 已成功保存: {filename} (进度: {count}/{target_count})")
        except OSError as e:
            print(f"保存失败，错误信息: {e}")
            break

        # 拍照瞬间在屏幕边缘画一个框闪烁提示
        # img.draw_rectangle(0, 0, img.width(), img.height(), color=255, thickness=10)
        # last_capture_time = current_time
        # time.sleep_ms(200)

    # 串口实时输出倒计时
    if count < target_count and (elapsed % 1000 < 50):
        print(f"倒计时: {remaining} 秒 | 已拍: {count}/{target_count}")

print("\n🎉 标定流程结束！")
