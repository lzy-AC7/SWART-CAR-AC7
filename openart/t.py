import sensor
import time
from machine import UART
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time=2000)
uart = UART(12, baudrate=115200)
clock = time.clock()
while True:
    clock.tick()
    img = sensor.snapshot()
    if uart.any():
        raw_data = uart.read().decode().strip()
        print(raw_data)
        if raw_data:
            uart.write(f"{raw_data}\n")
            print(raw_data)
        time.sleep_ms(10)
