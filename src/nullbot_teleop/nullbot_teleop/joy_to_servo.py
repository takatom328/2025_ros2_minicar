#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32

def clamp(x, lo, hi):
    return max(lo, min(hi, x))

class JoyToServo(Node):
    """
    /joy -> /nullbot/servo0/angle_deg, /nullbot/servo1/angle_deg
    - servo0: left stick LR (axes[0])  => angle 30..150 (center 90)
    - servo1: left stick UD (axes[1])  => angle 30..150 (center 90)
    """
    def __init__(self):
        super().__init__("joy_to_servo")

        # パラメータ（あとで調整しやすい）
        self.declare_parameter("axis_steer", 4)     # F710: 左スティック左右
        self.declare_parameter("axis_tilt", 5)      # F710: 左スティック上下
        self.declare_parameter("deadzone", 0.08)
        self.declare_parameter("min_deg", 30.0)
        self.declare_parameter("max_deg", 150.0)
        self.declare_parameter("center_deg", 90.0)
        self.declare_parameter("invert_tilt", True) # 上下が逆なら True/False 切替

        self.axis_steer = int(self.get_parameter("axis_steer").value)
        self.axis_tilt  = int(self.get_parameter("axis_tilt").value)
        self.deadzone   = float(self.get_parameter("deadzone").value)
        self.min_deg    = float(self.get_parameter("min_deg").value)
        self.max_deg    = float(self.get_parameter("max_deg").value)
        self.center_deg = float(self.get_parameter("center_deg").value)
        self.invert_tilt= bool(self.get_parameter("invert_tilt").value)

        self.pub0 = self.create_publisher(Float32, "/nullbot/servo0/angle_deg", 10)
        self.pub1 = self.create_publisher(Float32, "/nullbot/servo1/angle_deg", 10)
        self.sub  = self.create_subscription(Joy, "/joy", self.cb, 10)

        self.get_logger().info(
            f"joy_to_servo started: steer axis={self.axis_steer}, tilt axis={self.axis_tilt}"
        )

    def axis_to_angle(self, v):
        # v: -1..+1 を 角度(min..max)に線形変換
        v = float(v)
        if abs(v) < self.deadzone:
            v = 0.0

        # -1 -> min, 0 -> center, +1 -> max
        if v >= 0:
            # 正の値: center -> max
            angle = self.center_deg + v * (self.max_deg - self.center_deg)
        else:
            # 負の値: min -> center
            angle = self.center_deg + v * (self.center_deg - self.min_deg)

        return clamp(angle, self.min_deg, self.max_deg)

    def cb(self, msg: Joy):
        # 安全にインデックスチェック
        if len(msg.axes) <= max(self.axis_steer, self.axis_tilt):
            self.get_logger().warn(f"axes length {len(msg.axes)} too short")
            return

        steer_v = msg.axes[self.axis_steer]
        tilt_v  = msg.axes[self.axis_tilt]
        if self.invert_tilt:
            tilt_v = -tilt_v

        a0 = self.axis_to_angle(steer_v)
        a1 = self.axis_to_angle(tilt_v)

        # デバッグログ: 大きな変化があった時だけ出力
        if abs(steer_v) > 0.1 or abs(tilt_v) > 0.1:
            self.get_logger().info(
                f"Joy: steer={steer_v:.2f} ({a0:.1f}°), tilt={tilt_v:.2f} ({a1:.1f}°)"
            )

        self.pub0.publish(Float32(data=float(a0)))
        self.pub1.publish(Float32(data=float(a1)))

def main(args=None):
    rclpy.init(args=args)
    node = JoyToServo()
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

