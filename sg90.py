#!/usr/bin/env python
# -*- coding: utf-8 -*-
import time
import Adafruit_PCA9685

# PCA9685 初期化
pwm = Adafruit_PCA9685.PCA9685(address=0x40, busnum=1)

# サーボは 50Hz
pwm.set_pwm_freq(50)

SERVO_CH = 15  # PCA9685 PIN15

# SG90用パルス幅（要調整）
SERVO_MIN = 150  # 約0.5ms
SERVO_MAX = 600  # 約2.4ms

def set_servo_angle(channel, angle):
    """
    angle: 0〜180
    """
    pulse = SERVO_MIN + (SERVO_MAX - SERVO_MIN) * angle / 180.0
    pwm.set_pwm(channel, 0, int(pulse))

try:
    while True:
        set_servo_angle(SERVO_CH, 0)
        time.sleep(1)

        set_servo_angle(SERVO_CH, 90)
        time.sleep(1)

        set_servo_angle(SERVO_CH, 180)
        time.sleep(1)

except KeyboardInterrupt:
    pwm.set_pwm(SERVO_CH, 0, 0)
