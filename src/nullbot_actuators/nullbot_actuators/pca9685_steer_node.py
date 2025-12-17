#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped

import board
import busio
from adafruit_pca9685 import PCA9685


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


class PCA9685DriveNode(Node):
    def __init__(self):
        super().__init__("pca9685_drive_node")

        # ===== PCA9685 params =====
        self.declare_parameter("frequency", 50)
        self.declare_parameter("channel_steer", 0)
        self.declare_parameter("channel_esc", 1)

        # ===== steer calibration (us) =====
        self.declare_parameter("steer_us_left", 1100)
        self.declare_parameter("steer_us_center", 1500)
        self.declare_parameter("steer_us_right", 1900)
        self.declare_parameter("steer_limit_rad", 0.35)

        # ===== ESC calibration (us) : forward/reverse ESC =====
        self.declare_parameter("esc_us_min", 1000)       # full reverse (or min)
        self.declare_parameter("esc_us_neutral", 1500)   # neutral
        self.declare_parameter("esc_us_max", 2000)       # full forward (or max)

        # speed interpretation
        self.declare_parameter("max_speed_cmd", 1.0)     # speed input is assumed in [-max_speed_cmd, +max_speed_cmd]
        self.declare_parameter("esc_deadband", 0.05)     # small inputs -> neutral
        self.declare_parameter("esc_limit", 0.25)        # limit to [-0.25, +0.25] at first for safety

        # safety
        self.declare_parameter("timeout_sec", 0.3)
        self.declare_parameter("arm_neutral_sec", 2.0)   # keep neutral for this duration after start

        # ===== read params =====
        self.freq = int(self.get_parameter("frequency").value)
        self.ch_steer = int(self.get_parameter("channel_steer").value)
        self.ch_esc = int(self.get_parameter("channel_esc").value)

        self.steer_us_left = float(self.get_parameter("steer_us_left").value)
        self.steer_us_center = float(self.get_parameter("steer_us_center").value)
        self.steer_us_right = float(self.get_parameter("steer_us_right").value)
        self.steer_limit = float(self.get_parameter("steer_limit_rad").value)

        self.esc_us_min = float(self.get_parameter("esc_us_min").value)
        self.esc_us_neutral = float(self.get_parameter("esc_us_neutral").value)
        self.esc_us_max = float(self.get_parameter("esc_us_max").value)

        self.max_speed_cmd = float(self.get_parameter("max_speed_cmd").value)
        self.esc_deadband = float(self.get_parameter("esc_deadband").value)
        self.esc_limit = float(self.get_parameter("esc_limit").value)

        self.timeout = float(self.get_parameter("timeout_sec").value)
        self.arm_neutral_sec = float(self.get_parameter("arm_neutral_sec").value)

        # ===== PCA9685 init =====
        i2c = busio.I2C(board.SCL, board.SDA)
        self.pca = PCA9685(i2c)
        self.pca.frequency = self.freq

        self.last_cmd_time = self.get_clock().now()
        self.start_time = self.get_clock().now()

        # set safe initial output
        self._write_steer_us(self.steer_us_center)
        self._write_esc_us(self.esc_us_neutral)

        self.sub = self.create_subscription(
            AckermannDriveStamped,
            "/cmd_manual",
            self.cb,
            10
        )

        self.timer = self.create_timer(0.05, self.on_timer)

        self.get_logger().info(
            f"pca9685_drive_node started: steer_ch={self.ch_steer}, esc_ch={self.ch_esc}, "
            f"arm_neutral_sec={self.arm_neutral_sec}, esc_limit={self.esc_limit}"
        )

    def us_to_duty(self, us):
        period_us = 1_000_000 / self.freq  # 20,000us at 50Hz
        return int(clamp(us / period_us * 65535, 0, 65535))

    def _write_steer_us(self, us):
        self.pca.channels[self.ch_steer].duty_cycle = self.us_to_duty(us)

    def _write_esc_us(self, us):
        self.pca.channels[self.ch_esc].duty_cycle = self.us_to_duty(us)

    def steering_to_us(self, steer_rad):
        steer_rad = clamp(steer_rad, -self.steer_limit, self.steer_limit)
        if steer_rad >= 0:
            return self.steer_us_center + (steer_rad / self.steer_limit) * (self.steer_us_right - self.steer_us_center)
        else:
            return self.steer_us_center + (steer_rad / self.steer_limit) * (self.steer_us_center - self.steer_us_left)

    def speed_to_esc_us(self, speed):
        # speed is expected in [-max_speed_cmd, +max_speed_cmd]
        if self.max_speed_cmd <= 0:
            return self.esc_us_neutral

        s = clamp(speed / self.max_speed_cmd, -1.0, 1.0)

        # safety limiter (start small!)
        s = clamp(s, -self.esc_limit, self.esc_limit)

        # deadband around zero
        if abs(s) < self.esc_deadband:
            return self.esc_us_neutral

        if s > 0:
            return self.esc_us_neutral + s * (self.esc_us_max - self.esc_us_neutral)
        else:
            return self.esc_us_neutral + s * (self.esc_us_neutral - self.esc_us_min)

    def cb(self, msg: AckermannDriveStamped):
        self.last_cmd_time = self.get_clock().now()

        # During arming window, force neutral on ESC
        t_from_start = (self.get_clock().now() - self.start_time).nanoseconds * 1e-9
        steer_us = self.steering_to_us(msg.drive.steering_angle)

        if t_from_start < self.arm_neutral_sec:
            esc_us = self.esc_us_neutral
        else:
            esc_us = self.speed_to_esc_us(msg.drive.speed)

        self._write_steer_us(steer_us)
        self._write_esc_us(esc_us)

    def on_timer(self):
        # failsafe: no command -> center + neutral
        dt = (self.get_clock().now() - self.last_cmd_time).nanoseconds * 1e-9
        if dt > self.timeout:
            self._write_steer_us(self.steer_us_center)
            self._write_esc_us(self.esc_us_neutral)

    def destroy_node(self):
        try:
            self._write_steer_us(self.steer_us_center)
            self._write_esc_us(self.esc_us_neutral)
            self.pca.deinit()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = PCA9685DriveNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
