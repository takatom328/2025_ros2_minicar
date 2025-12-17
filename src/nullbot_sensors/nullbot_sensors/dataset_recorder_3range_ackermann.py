#!/usr/bin/env python3
import os
import csv
from datetime import datetime

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Range
from ackermann_msgs.msg import AckermannDriveStamped


class DatasetRecorder3RangeAckermann(Node):
    """
    Record:
      - /range/front_filtered, /range/left_filtered, /range/right_filtered (sensor_msgs/Range)
      - /cmd_manual (ackermann_msgs/AckermannDriveStamped)
    to CSV at fixed sample_hz.

    Safe for "no hardware" situations:
      - If any input missing, it will skip writing (or write blank if you prefer).
    """

    def __init__(self):
        super().__init__("dataset_recorder_3range_ackermann")

        # ---- params ----
        self.declare_parameter("front_topic", "/range/front_filtered")
        self.declare_parameter("left_topic", "/range/left_filtered")
        self.declare_parameter("right_topic", "/range/right_filtered")
        self.declare_parameter("cmd_topic", "/cmd_manual")

        self.declare_parameter("out_dir", os.path.expanduser("~/datasets"))
        self.declare_parameter("file_prefix", "run")
        self.declare_parameter("sample_hz", 10.0)

        # if True: write even when some sensors are missing (fill with NaN)
        self.declare_parameter("allow_partial", False)

        # flush to disk every N rows
        self.declare_parameter("flush_every_n", 20)

        self.front_topic = str(self.get_parameter("front_topic").value)
        self.left_topic  = str(self.get_parameter("left_topic").value)
        self.right_topic = str(self.get_parameter("right_topic").value)
        self.cmd_topic   = str(self.get_parameter("cmd_topic").value)

        self.out_dir = str(self.get_parameter("out_dir").value)
        self.file_prefix = str(self.get_parameter("file_prefix").value)
        self.sample_hz = float(self.get_parameter("sample_hz").value)
        self.allow_partial = bool(self.get_parameter("allow_partial").value)
        self.flush_every_n = int(self.get_parameter("flush_every_n").value)

        os.makedirs(self.out_dir, exist_ok=True)
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.out_path = os.path.join(self.out_dir, f"{self.file_prefix}_{ts}.csv")

        # ---- caches (latest values) ----
        self.front = None
        self.left = None
        self.right = None
        self.cmd = None

        self.front_stamp = None
        self.left_stamp = None
        self.right_stamp = None
        self.cmd_stamp = None

        # ---- subs ----
        self.create_subscription(Range, self.front_topic, self.cb_front, 10)
        self.create_subscription(Range, self.left_topic,  self.cb_left, 10)
        self.create_subscription(Range, self.right_topic, self.cb_right, 10)
        self.create_subscription(AckermannDriveStamped, self.cmd_topic, self.cb_cmd, 10)

        # ---- file ----
        self.f = open(self.out_path, "w", newline="")
        self.w = csv.writer(self.f)
        self.w.writerow([
            "t_ros",
            "front_m", "left_m", "right_m",
            "front_stamp", "left_stamp", "right_stamp",
            "steer_rad", "speed_mps",
            "cmd_stamp",
        ])
        self.f.flush()

        self.rows = 0
        period = 1.0 / max(0.1, self.sample_hz)
        self.timer = self.create_timer(period, self.on_timer)

        self.get_logger().info(f"Recording to: {self.out_path}")
        self.get_logger().info(
            f"topics: front={self.front_topic}, left={self.left_topic}, right={self.right_topic}, cmd={self.cmd_topic}"
        )
        self.get_logger().info(f"sample_hz={self.sample_hz}, allow_partial={self.allow_partial}")

    @staticmethod
    def _stamp_to_sec(stamp) -> float:
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    def cb_front(self, msg: Range):
        self.front = float(msg.range)
        self.front_stamp = self._stamp_to_sec(msg.header.stamp)

    def cb_left(self, msg: Range):
        self.left = float(msg.range)
        self.left_stamp = self._stamp_to_sec(msg.header.stamp)

    def cb_right(self, msg: Range):
        self.right = float(msg.range)
        self.right_stamp = self._stamp_to_sec(msg.header.stamp)

    def cb_cmd(self, msg: AckermannDriveStamped):
        self.cmd = msg
        self.cmd_stamp = self._stamp_to_sec(msg.header.stamp)

    def _ready(self) -> bool:
        if self.allow_partial:
            # need at least cmd + one sensor
            sensors_ok = (self.front is not None) or (self.left is not None) or (self.right is not None)
            return (self.cmd is not None) and sensors_ok
        else:
            # need all
            return (self.front is not None) and (self.left is not None) and (self.right is not None) and (self.cmd is not None)

    def on_timer(self):
        if not self._ready():
            return

        now = self.get_clock().now().nanoseconds * 1e-9

        def f_or_nan(x):
            return "nan" if x is None else f"{float(x):.6f}"

        def s_or_nan(x):
            return "nan" if x is None else f"{float(x):.6f}"

        steer = float(self.cmd.drive.steering_angle) if self.cmd is not None else float("nan")
        speed = float(self.cmd.drive.speed) if self.cmd is not None else float("nan")

        self.w.writerow([
            f"{now:.6f}",
            f_or_nan(self.front), f_or_nan(self.left), f_or_nan(self.right),
            s_or_nan(self.front_stamp), s_or_nan(self.left_stamp), s_or_nan(self.right_stamp),
            f"{steer:.6f}", f"{speed:.6f}",
            s_or_nan(self.cmd_stamp),
        ])
        self.rows += 1

        if self.rows % self.flush_every_n == 0:
            self.f.flush()

    def destroy_node(self):
        try:
            if hasattr(self, "f") and self.f:
                self.f.flush()
                self.f.close()
        finally:
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DatasetRecorder3RangeAckermann()
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
