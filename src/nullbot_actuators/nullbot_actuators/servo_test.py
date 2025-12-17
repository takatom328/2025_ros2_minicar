import time
import board, busio
from adafruit_pca9685 import PCA9685

i2c = busio.I2C(board.SCL, board.SDA)
pca = PCA9685(i2c, address=0x40)
pca.frequency = 50

def set_us(ch, us):
    period_us = 1_000_000 / pca.frequency
    duty = int(us / period_us * 4096)
    duty = max(0, min(4095, duty))
    pca.channels[ch].duty_cycle = duty

# ch0でテスト（刺してるチャンネルに合わせて変えてOK）
ch = 0

print(f"Testing servo on channel {ch}")
print(f"PCA9685 frequency: {pca.frequency} Hz")
print("Moving servo in a loop...")

while True:
    print(f"  Setting {ch} to 1500us (center)")
    set_us(ch, 1500)  # だいたいセンター
    time.sleep(1)
    print(f"  Setting {ch} to 800us (min)")
    set_us(ch, 800)   # 片側
    time.sleep(1)
    print(f"  Setting {ch} to 2200us (max)")
    set_us(ch, 2200)  # 反対側
    time.sleep(1)
