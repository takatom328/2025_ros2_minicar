#!/usr/bin/env python3
"""
GPIO18（BCM18, 物理ピン12）でサーボを直接制御するテスト
GPIO18はハードウェアPWM0に対応
"""
import time
import sys
import lgpio

# 即座に出力するためにバッファリング無効化
sys.stdout.reconfigure(line_buffering=True)

# GPIO18 (BCM番号)
SERVO_PIN = 18

# lgpioチップをオープン
h = lgpio.gpiochip_open(0)

# ハードウェアPWMの設定
# 周波数: 50Hz (サーボ標準)
# パルス幅: 500〜2500us (0.5ms〜2.5ms)
frequency = 50  # Hz
period_us = 1_000_000 / frequency  # 20000us = 20ms

print(f"Testing servo on GPIO{SERVO_PIN} (BCM)")
print(f"Frequency: {frequency} Hz, Period: {period_us} us")

def set_servo_us(pulse_us):
    """パルス幅（マイクロ秒）でサーボを制御"""
    duty_cycle = (pulse_us / period_us) * 100.0  # パーセント
    duty_cycle = max(0, min(100, duty_cycle))

    # lgpioでハードウェアPWMを設定
    # tx_pwm(handle, gpio, pwm_frequency, pwm_duty_cycle, offset=0, cycles=0)
    lgpio.tx_pwm(h, SERVO_PIN, frequency, duty_cycle)

    print(f"  Pulse: {pulse_us}us -> Duty: {duty_cycle:.2f}%")

try:
    print("\nStarting servo test loop...")
    while True:
        print("Center position (1500us)")
        set_servo_us(1500)
        time.sleep(2)

        print("Min position (600us)")
        set_servo_us(600)
        time.sleep(2)

        print("Max position (2400us)")
        set_servo_us(2400)
        time.sleep(2)

except KeyboardInterrupt:
    print("\nStopping...")
finally:
    # PWM停止
    lgpio.tx_pwm(h, SERVO_PIN, 0, 0)
    lgpio.gpiochip_close(h)
    print("Cleanup done")
