#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from ackermann_msgs.msg import AckermannDriveStamped


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


class JoyToCmd(Node):
    """
    /joy -> /cmd_manual (AckermannDriveStamped)
      - steering: left stick LR (axis_steer) -> steering_angle [rad]
      - throttle: right stick UD or trigger (axis_throttle) -> speed (normalized or m/s-like)
    Safety:
      - enable button (hold) optional
      - e-stop button (hold) optional
      - timeout can be handled in mux node later
    """

    def __init__(self):
        super().__init__("joy_to_cmd")

        # ---- Parameters (F710 mapping: adjust these using jstest/echo /joy) ----
        self.declare_parameter("axis_steer", 4)        # your current setting (F710)
        self.declare_parameter("axis_throttle", 1)     # set to your right stick Y etc.
        self.declare_parameter("invert_throttle", True)
        self.declare_parameter("deadzone", 0.08)

        # steering scale
        self.declare_parameter("max_steer_rad", 0.35)  # ~20deg

        # throttle scale (normalized for now)
        self.declare_parameter("max_speed", 1.0)       # 1.0 means full scale

        # optional safety buttons (set -1 to disable)
        self.declare_parameter("btn_enable_hold", -1)  # e.g. RB: publish only while pressed
        self.declare_parameter("btn_estop_hold", -1)   # e.g. LB: while pressed -> speed=0, steer=0

        self.axis_steer = int(self.get_parameter("axis_steer").value)
        self.axis_throttle = int(self.get_parameter("axis_throttle").value)
        self.invert_throttle = bool(self.get_parameter("invert_throttle").value)
        self.deadzone = float(self.get_parameter("deadzone").value)
        self.max_steer_rad = float(self.get_parameter("max_steer_rad").value)
        self.max_speed = float(self.get_parameter("max_speed").value)
        self.btn_enable_hold = int(self.get_parameter("btn_enable_hold").value)
        self.btn_estop_hold = int(self.get_parameter("btn_estop_hold").value)

        self.pub = self.create_publisher(AckermannDriveStamped, "/cmd_manual", 10)
        self.sub = self.create_subscription(Joy, "/joy", self.cb, 10)

        self.get_logger().info(
            "joy_to_cmd started: "
            f"axis_steer={self.axis_steer}, axis_throttle={self.axis_throttle}, "
            f"invert_throttle={self.invert_throttle}, deadzone={self.deadzone}, "
            f"max_steer_rad={self.max_steer_rad}, max_speed={self.max_speed}, "
            f"btn_enable_hold={self.btn_enable_hold}, btn_estop_hold={self.btn_estop_hold}"
        )

    def _get_axis(self, msg: Joy, idx: int) -> float:
        if idx < 0 or idx >= len(msg.axes):
            return 0.0
        return float(msg.axes[idx])

    def _get_button(self, msg: Joy, idx: int) -> int:
        if idx < 0 or idx >= len(msg.buttons):
            return 0
        return int(msg.buttons[idx])

    def _apply_deadzone(self, v: float) -> float:
        return 0.0 if abs(v) < self.deadzone else v

    def cb(self, msg: Joy):
        # index safety
        if len(msg.axes) <= max(self.axis_steer, self.axis_throttle):
            self.get_logger().warn(f"axes length {len(msg.axes)} too short")
            return

        # optional enable-hold
        if self.btn_enable_hold >= 0:
            if self._get_button(msg, self.btn_enable_hold) == 0:
                return

        # e-stop (hold)
        estop = False
        if self.btn_estop_hold >= 0:
            estop = (self._get_button(msg, self.btn_estop_hold) == 1)

        steer_in = self._apply_deadzone(self._get_axis(msg, self.axis_steer))
        thr_in = self._apply_deadzone(self._get_axis(msg, self.axis_throttle))
        if self.invert_throttle:
            thr_in = -thr_in

        steering_angle = clamp(steer_in, -1.0, 1.0) * self.max_steer_rad
        speed = clamp(thr_in, -1.0, 1.0) * self.max_speed

        if estop:
            steering_angle = 0.0
            speed = 0.0

        out = AckermannDriveStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = "base_link"
        out.drive.steering_angle = steering_angle
        out.drive.speed = speed

        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = JoyToCmd()
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
