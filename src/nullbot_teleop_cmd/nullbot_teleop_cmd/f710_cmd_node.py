#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32, Bool

def clamp(x, lo, hi):
    return max(lo, min(hi, x))

class F710CmdNode(Node):
    def __init__(self):
        super().__init__("f710_cmd_node")

        # ---- parameters ----
        self.declare_parameter("axis_steer", 4)      # 左スティック左右
        self.declare_parameter("axis_throttle", 3)   # 例：右スティック上下（必要なら）
        self.declare_parameter("invert_throttle", True)
        self.declare_parameter("deadzone", 0.08)

        # ボタンはコントローラ設定で変わるので、まずは param に
        self.declare_parameter("enable_button", 4)   # 仮：LBなど（あとで合わせる）
        self.declare_parameter("timeout_sec", 0.5)

        self.axis_steer = int(self.get_parameter("axis_steer").value)
        self.axis_throttle = int(self.get_parameter("axis_throttle").value)
        self.invert_throttle = bool(self.get_parameter("invert_throttle").value)
        self.deadzone = float(self.get_parameter("deadzone").value)
        self.enable_button = int(self.get_parameter("enable_button").value)
        self.timeout_sec = float(self.get_parameter("timeout_sec").value)

        self.pub_steer = self.create_publisher(Float32, "/nullbot/cmd/steer", 10)
        self.pub_throttle = self.create_publisher(Float32, "/nullbot/cmd/throttle", 10)
        self.pub_enable = self.create_publisher(Bool, "/nullbot/cmd/enable", 10)

        self.sub = self.create_subscription(Joy, "/joy", self.cb, 10)

        self.last_rx = time.time()
        self.timer = self.create_timer(0.05, self.safety_timer)  # 20Hz

        self.get_logger().info("f710_cmd_node started")

    def dz(self, v: float) -> float:
        v = float(v)
        return 0.0 if abs(v) < self.deadzone else v

    def cb(self, msg: Joy):
        self.last_rx = time.time()

        # safe index checks
        if len(msg.axes) <= max(self.axis_steer, self.axis_throttle):
            self.get_logger().warn(f"axes length too short: {len(msg.axes)}")
            return
        if len(msg.buttons) <= self.enable_button:
            self.get_logger().warn(f"buttons length too short: {len(msg.buttons)}")
            return

        enabled = (msg.buttons[self.enable_button] == 1)

        steer = self.dz(msg.axes[self.axis_steer])  # -1..+1
        thr_raw = self.dz(msg.axes[self.axis_throttle])  # -1..+1
        if self.invert_throttle:
            thr_raw = -thr_raw

        # throttle: -1..+1 -> 0..1 （前進だけ欲しい想定）
        throttle = clamp((thr_raw + 1.0) * 0.5, 0.0, 1.0)

        if not enabled:
            steer = 0.0
            throttle = 0.0

        self.pub_enable.publish(Bool(data=bool(enabled)))
        self.pub_steer.publish(Float32(data=float(steer)))
        self.pub_throttle.publish(Float32(data=float(throttle)))

    def safety_timer(self):
        if (time.time() - self.last_rx) > self.timeout_sec:
            self.pub_enable.publish(Bool(data=False))
            self.pub_steer.publish(Float32(data=0.0))
            self.pub_throttle.publish(Float32(data=0.0))

def main(args=None):
    rclpy.init(args=args)
    node = F710CmdNode()
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
