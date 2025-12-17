#!/usr/bin/env python3
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range

import lgpio


class UltrasonicNode(Node):
    def __init__(self):
        super().__init__('ultrasonic_node')

        # GPIO 設定
        self.TRIG = 23  # BCM23
        self.ECHO = 24  # BCM24

        # lgpio チップをオープン (通常は0)
        self.h = lgpio.gpiochip_open(0)

        # ピンを出力/入力として設定
        lgpio.gpio_claim_output(self.h, self.TRIG)
        lgpio.gpio_claim_input(self.h, self.ECHO)

        # HC-SR04 の仕様に合わせて Range メッセージを準備
        self.publisher_ = self.create_publisher(Range, 'ultrasonic/range', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10Hz

        self.frame_id = 'ultrasonic_link'
        self.get_logger().info('UltrasonicNode started (TRIG=23, ECHO=24)')

    def measure_distance(self) -> float:
        """
        HC-SR04 で距離を測定して、メートルで返す。
        測定に失敗したら None を返す。
        """
        # センサ安定のための初期化パルス
        lgpio.gpio_write(self.h, self.TRIG, 0)
        time.sleep(0.0002)  # 200us

        # 10us のトリガパルスを送出
        lgpio.gpio_write(self.h, self.TRIG, 1)
        time.sleep(0.00001)
        lgpio.gpio_write(self.h, self.TRIG, 0)

        # エコーの立ち上がり待ち
        timeout = time.time() + 0.02  # 20ms timeout
        while lgpio.gpio_read(self.h, self.ECHO) == 0:
            pulse_start = time.time()
            if pulse_start > timeout:
                return None

        # エコーの立ち下がり待ち
        timeout = time.time() + 0.02
        while lgpio.gpio_read(self.h, self.ECHO) == 1:
            pulse_end = time.time()
            if pulse_end > timeout:
                return None

        pulse_duration = pulse_end - pulse_start
        # 音速 343m/s (20℃) → 片道なので /2
        distance_m = (pulse_duration * 343.0) / 2.0

        return distance_m

    def timer_callback(self):
        distance = self.measure_distance()
        if distance is None:
            self.get_logger().warn('Ultrasonic timeout')
            return

        msg = Range()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.radiation_type = Range.ULTRASOUND
        msg.field_of_view = 0.5  # 適当な値（ラジアン）
        msg.min_range = 0.02     # 2cm
        msg.max_range = 4.0      # 4m
        msg.range = distance

        self.publisher_.publish(msg)
        self.get_logger().info(f'Distance: {distance:.3f} m')

    def destroy_node(self):
        lgpio.gpiochip_close(self.h)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = UltrasonicNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
