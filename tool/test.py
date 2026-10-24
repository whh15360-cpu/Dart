import sensor
import time
import pyb

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time=1000)

# OpenMV H7 Plus 板载 RGB LED (1:红, 2:绿, 3:蓝)
led_blue = pyb.LED(3)

clock = time.clock()

while True:
    clock.tick()
    img = sensor.snapshot()

    # 每次循环切换蓝灯状态，实现肉眼可见的闪烁
    led_blue.toggle()

    print("FPS:", clock.fps())
