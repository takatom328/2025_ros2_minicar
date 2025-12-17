
#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

# Adafruit PCA9685
import board
import busio
from adafruit_pca9685 import PCA9685


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


class ServoNode(Node):
    """
    nullbot_actuators/servo_node
    - /nullbot/servo0/angle_deg (std_msgs/Float32) : 0〜180
    - /nullbot/servo1/angle_deg (std_msgs/Float32) : 0〜180
    """

    def __init__(self):
        super().__init__("servo_node")

        # ---- parameters ----
        self.declare_parameter("i2c_bus", 1)
        self.declare_parameter("address", 0x40)
        self.declare_parameter("frequency_hz", 50.0)

        # SG90 は個体差が大きいので「だいたい」で開始して、あとで調整する前提
        self.declare_parameter("servo0_channel", 0)
        self.declare_parameter("servo1_channel", 1)

        # 角度→パルス幅の変換（us）
        # SG90: 500〜2500us くらいが多いが、無理させないため少し狭め推奨
        self.declare_parameter("min_us", 600.0)
        self.declare_parameter("max_us", 2400.0)

        # 安全のため角度範囲も制限
        self.declare_parameter("min_deg", 0.0)
        self.declare_parameter("max_deg", 180.0)

        # 初期角度
        self.declare_parameter("servo0_init_deg", 90.0)
        self.declare_parameter("servo1_init_deg", 90.0)

        freq = float(self.get_parameter("frequency_hz").value)
        addr = int(self.get_parameter("address").value)
        self.min_us = float(self.get_parameter("min_us").value)
        self.max_us = float(self.get_parameter("max_us").value)
        self.min_deg = float(self.get_parameter("min_deg").value)
        self.max_deg = float(self.get_parameter("max_deg").value)

        self.ch0 = int(self.get_parameter("servo0_channel").value)
        self.ch1 = int(self.get_parameter("servo1_channel").value)

        # ---- PCA9685 init ----
        # Pi の I2C は通常 board.SCL/board.SDA が /dev/i2c-1 に対応
        i2c = busio.I2C(board.SCL, board.SDA)
        self.pca = PCA9685(i2c, address=addr)
        self.pca.frequency = int(freq)

        self.get_logger().info(
            f"PCA9685 init ok: addr=0x{addr:02x}, freq={self.pca.frequency}Hz, ch0={self.ch0}, ch1={self.ch1}"
        )

        # ---- subscribers ----
        self.sub0 = self.create_subscription(Float32, "/nullbot/servo0/angle_deg", self.cb_servo0, 10)
        self.sub1 = self.create_subscription(Float32, "/nullbot/servo1/angle_deg", self.cb_servo1, 10)

        # init positions
        self.set_servo_angle(self.ch0, float(self.get_parameter("servo0_init_deg").value))
        self.set_servo_angle(self.ch1, float(self.get_parameter("servo1_init_deg").value))

    def angle_to_duty(self, angle_deg: float) -> int:
        # clamp angle
        a = clamp(angle_deg, self.min_deg, self.max_deg)

        # map deg -> pulse us
        pulse_us = self.min_us + (self.max_us - self.min_us) * (a - self.min_deg) / (self.max_deg - self.min_deg)

        # PCA9685: 12-bit (0..4095) over one period
        period_us = 1_000_000.0 / float(self.pca.frequency)
        duty = int(clamp(pulse_us / period_us * 4096.0, 0.0, 4095.0))
        return duty

    def set_servo_angle(self, channel: int, angle_deg: float) -> None:
        duty = self.angle_to_duty(angle_deg)
        self.pca.channels[channel].duty_cycle = duty
        self.get_logger().debug(f"set ch{channel}: angle={angle_deg:.1f} deg -> duty={duty}")

    def cb_servo0(self, msg: Float32) -> None:
        self.set_servo_angle(self.ch0, float(msg.data))

    def cb_servo1(self, msg: Float32) -> None:
        self.set_servo_angle(self.ch1, float(msg.data))

    def destroy_node(self):
        # サーボ停止（duty=0）にしたいならここで 0 を入れてもOK
        try:
            self.pca.deinit()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ServoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
