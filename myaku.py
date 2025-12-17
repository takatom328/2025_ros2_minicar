#!/usr/bin/env python3
# -*- coding: utf-8 -*-
'''
# Adafruit Blinka（CircuitPython互換レイヤー）をインストール
pip3 install adafruit-blinka --break-system-packages

# VL53L0Xライブラリをインストール
pip3 install adafruit-circuitpython-vl53l0x --break-system-packages
'''

import time
import board
import busio
import adafruit_vl53l0x
import Adafruit_PCA9685

class myaku:
    
    def __init__(self):     # 初期化処理（インスタンス生成時に一度だけ実行）"""
        print("初期化処理を実行")
        #self.pwm   → クラス全体で使える（インスタンス変数.c言語でいうグローバル変数）
        #pwm        → その関数内だけで使える（ローカル変数）


        """パラメータのセットアップ"""
        # PIN設定
        self.MOTOR_PIN  = 18
        self.LED_PIN    = 23
        self.CHANNEL    = 15
        # 定数設定
        self.PWM_FREQUENCY  = 50        # Hz
        self.FUNCTYPE       = "ALL"     # "SG90", "VL53L0X", "ALL"
        self.SG90TYPE       = "sweep"   # "simple", "sweep"
        # 上下限設定
        self.DUTY_MIN       = 5.0     # %
        self.DUTY_MAX       = 10.0    # %
        self.SERVO_MIN      = 150
        self.SERVO_MAX      = 600

        """各デバイスのセットアップ"""
        print("デバイス接続確認中...")
        try:
            self.pwm = Adafruit_PCA9685.PCA9685(address=0x40)
            print("pwm 0x40で接続成功")
        except:
            try:
                self.pwm = Adafruit_PCA9685.PCA9685(address=0x43)
                print("pwm 0x43で接続成功")
            except:
                try:
                    self.pwm = Adafruit_PCA9685.PCA9685(address=0x44)
                    print("pwm 0x44で接続成功")
                except:
                    print("pwm すべて失敗")
                    exit()

        try:
            self.i2c = busio.I2C(board.SCL, board.SDA)
            self.vl53 = adafruit_vl53l0x.VL53L0X(self.i2c, address=0x29)
            print("VL53L0X接続成功")
        except Exception as e:
            print(f"VL53L0X接続エラー: {e}")
            exit()

        print("デバイス初期化中...")
        self.pwm.set_pwm_freq(self.PWM_FREQUENCY)           # PWMデバイスの初期化

        print("リソースセットアップ完了")
    
    def set_angle(self, channel, angle):
        """サーボ角度設定"""
        pulse = int(self.SERVO_MIN + (self.SERVO_MAX - self.SERVO_MIN) * angle / 180)
        self.pwm.set_pwm(channel, 0, pulse)

    def distance_to_angle(self, distance_mm):
        """距離(mm)を角度(0-180度)に変換"""
        # 測定範囲を設定（例：100mm〜1000mm）
        self.MIN_DISTANCE = 10      # mm
        self.MAX_DISTANCE = 1000    # mm
        
        # 範囲外の値をクリップ
        if distance_mm   < self.MIN_DISTANCE:
            distance_mm  = self.MIN_DISTANCE
        elif distance_mm > self.MAX_DISTANCE:
            distance_mm  = self.MAX_DISTANCE
        
        # 距離を角度に線形変換
        # 近い(100mm) → 0度、遠い(1000mm) → 180度
        angle = int((distance_mm - self.MIN_DISTANCE) * 180 / (self.MAX_DISTANCE - self.MIN_DISTANCE))
        
        return angle

    def run(self):
        """メインループ"""
        try:
            print("動作確認開始")
            
            if self.FUNCTYPE == "ALL":
                while True:
                    # 距離測定
                    self.distance_mm = self.vl53.range
                    self.distance_cm = self.distance_mm / 10.0
                    
                    # 距離を角度に変換
                    angle = self.distance_to_angle(self.distance_mm)
                    
                    # サーボを動かす
                    self.set_angle(self.CHANNEL, angle)
                    
                    # 状態表示
                    print(f"距離: {self.distance_cm:.1f}cm -> 角度: {angle}度")
                    
                    time.sleep(0.1)  # 100ms周期で更新

            elif self.FUNCTYPE == "SG90":
                if self.SG90TYPE == "simple":
                    for angle in [0, 90, 180, 90, 0]:
                        print(f"{angle}度")
                        self.set_angle(self.CHANNEL, angle)
                        time.sleep(1)
                    print("完了")
                elif self.FUNCTYPE == "sweep":
                    print("スイープテスト開始 (Ctrl+Cで終了)")
                    while True:
                        # 0 → 180度
                        for angle in range(0, 181, 5):
                            self.set_angle(self.CHANNEL, angle)
                            time.sleep(0.05)
                        
                        # 180 → 0度
                        for angle in range(180, -1, -5):
                            self.set_angle(self.CHANNEL, angle)
                            time.sleep(0.05)
            elif self.FUNCTYPE == "VL53L0X":
                while True:
                    self.distance_mm = self.vl53.range              # 距離測定（mm単位）
                    self.distance_cm = self.distance_mm / 10.0      # cm単位に変換
                    
                    print(f"距離: {self.distance_cm:6.1f} cm ({self.distance_mm:4d} mm)")
                    time.sleep(0.1)

        except KeyboardInterrupt:
            print("\n終了")

        finally:
            self.pwm.set_all_pwm(0, 0)

if __name__ == "__main__":
    app = myaku()  # __init__が自動実行される
    app.run()