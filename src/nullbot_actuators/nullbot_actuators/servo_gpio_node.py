#!/usr/bin/env python3
"""
GPIO直接制御でサーボモーターを制御するROS2ノード
GPIO18でハードウェアPWMを使用
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import lgpio


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


class ServoGpioNode(Node):
    """
    nullbot_actuators/servo_gpio_node
    - /nullbot/servo0/angle_deg (std_msgs/Float32) : 0〜180度
    """

    def __init__(self):
        super().__init__("servo_gpio_node")

        # パラメータ
        self.declare_parameter("servo_pin", 18)  # GPIO18 (BCM)
        self.declare_parameter("frequency_hz", 50.0)  # サーボ標準周波数
        self.declare_parameter("min_us", 600.0)  # 最小パルス幅
        self.declare_parameter("max_us", 2400.0)  # 最大パルス幅
        self.declare_parameter("min_deg", 0.0)  # 最小角度
        self.declare_parameter("max_deg", 180.0)  # 最大角度
        self.declare_parameter("init_deg", 90.0)  # 初期角度

        self.servo_pin = int(self.get_parameter("servo_pin").value)
        self.frequency = float(self.get_parameter("frequency_hz").value)
        self.min_us = float(self.get_parameter("min_us").value)
        self.max_us = float(self.get_parameter("max_us").value)
        self.min_deg = float(self.get_parameter("min_deg").value)
        self.max_deg = float(self.get_parameter("max_deg").value)

        # 周期計算
        self.period_us = 1_000_000.0 / self.frequency

        # lgpio初期化
        self.h = lgpio.gpiochip_open(0)

        # GPIO18を出力として設定
        # 既に使用中の場合は一旦解放してから再クレーム
        try:
            lgpio.gpio_claim_output(self.h, self.servo_pin)
        except Exception as e:
            self.get_logger().warn(f"GPIO{self.servo_pin} claim failed (already in use?), attempting to free...")
            try:
                lgpio.gpio_free(self.h, self.servo_pin)
            except:
                pass
            lgpio.gpio_claim_output(self.h, self.servo_pin)

        self.get_logger().info(
            f"ServoGpioNode started: GPIO{self.servo_pin}, {self.frequency}Hz"
        )

        # サブスクライバ
        self.sub = self.create_subscription(
            Float32, "/nullbot/servo0/angle_deg", self.cb_angle, 10
        )

        # 初期位置
        init_angle = float(self.get_parameter("init_deg").value)
        self.set_servo_angle(init_angle)

    def angle_to_pulse_us(self, angle_deg: float) -> float:
        """角度をパルス幅（us）に変換"""
        a = clamp(angle_deg, self.min_deg, self.max_deg)
        pulse_us = self.min_us + (self.max_us - self.min_us) * (a - self.min_deg) / (
            self.max_deg - self.min_deg
        )
        return pulse_us

    def set_servo_angle(self, angle_deg: float) -> None:
        """サーボの角度を設定"""
        pulse_us = self.angle_to_pulse_us(angle_deg)
        duty_cycle = (pulse_us / self.period_us) * 100.0
        duty_cycle = clamp(duty_cycle, 0.0, 100.0)

        lgpio.tx_pwm(self.h, self.servo_pin, self.frequency, duty_cycle)

        self.get_logger().debug(
            f"Angle: {angle_deg:.1f}° -> {pulse_us:.0f}us -> {duty_cycle:.2f}%"
        )

    def cb_angle(self, msg: Float32) -> None:
        """角度指令のコールバック"""
        self.set_servo_angle(float(msg.data))
        self.get_logger().info(f"Servo angle set to: {msg.data:.1f}°")

    def destroy_node(self):
        """クリーンアップ"""
        try:
            # PWM停止
            lgpio.tx_pwm(self.h, self.servo_pin, 0, 0)
            # GPIOを解放
            lgpio.gpio_free(self.h, self.servo_pin)
            lgpio.gpiochip_close(self.h)
        except Exception as e:
            self.get_logger().warn(f"Cleanup warning: {e}")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ServoGpioNode()
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
