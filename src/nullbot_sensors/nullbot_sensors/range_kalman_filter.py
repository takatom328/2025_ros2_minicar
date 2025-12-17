#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range


def stamp_to_sec(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


class Kalman1D:
    # state: [x, v], measurement: z = x
    def __init__(self, x0=1.0, v0=0.0, p_x=1.0, p_v=1.0):
        self.x = [float(x0), float(v0)]
        # covariance P (2x2)
        self.P = [[float(p_x), 0.0],
                  [0.0, float(p_v)]]

    def predict(self, dt: float, q_x: float, q_v: float):
        dt = max(1e-4, float(dt))

        # F = [[1, dt],[0,1]]
        x0, v0 = self.x
        self.x = [x0 + v0 * dt, v0]

        # P = F P F^T + Q
        P00, P01 = self.P[0]
        P10, P11 = self.P[1]

        # F*P
        FP00 = P00 + dt * P10
        FP01 = P01 + dt * P11
        FP10 = P10
        FP11 = P11

        # (F*P)*F^T
        P00n = FP00 + dt * FP01
        P01n = FP01
        P10n = FP10 + dt * FP11
        P11n = FP11

        # Q（プロセスノイズ）: ここはチューニングポイント
        P00n += q_x
        P11n += q_v

        self.P = [[P00n, P01n],
                  [P10n, P11n]]

    def innovation(self, z: float, r: float):
        # H = [1, 0]
        x0 = self.x[0]
        y = float(z) - x0
        S = self.P[0][0] + float(r)
        return y, S

    def update(self, z: float, r: float):
        y, S = self.innovation(z, r)

        # K = P H^T S^-1 = [P00, P10]^T / S
        K0 = self.P[0][0] / S
        K1 = self.P[1][0] / S

        # x = x + K y
        self.x[0] += K0 * y
        self.x[1] += K1 * y

        # P = (I - K H) P
        # (I-KH) = [[1-K0, 0],[-K1, 1]]
        P00, P01 = self.P[0]
        P10, P11 = self.P[1]

        P00n = (1.0 - K0) * P00
        P01n = (1.0 - K0) * P01
        P10n = P10 - K1 * P00
        P11n = P11 - K1 * P01

        self.P = [[P00n, P01n],
                  [P10n, P11n]]


class RangeKalmanFilterNode(Node):
    def __init__(self):
        super().__init__("range_kalman_filter")

        # topics
        self.declare_parameter("input_topic", "/range/front")
        self.declare_parameter("output_topic", "/range/front_filtered")

        # filter params
        self.declare_parameter("r_meas", 0.03**2)     # 観測ノイズ分散（m^2）
        self.declare_parameter("q_x", 1e-4)           # プロセスノイズ（距離側）
        self.declare_parameter("q_v", 1e-2)           # プロセスノイズ（速度側）

        # robust params
        self.declare_parameter("nis_threshold", 9.0)  # 3σ相当（1自由度）
        self.declare_parameter("jump_threshold", 0.8) # これ以上の急変は弾く(m)
        self.declare_parameter("dt_max", 0.2)         # これ以上空いたらdtを頭打ち

        # init
        self.in_topic = str(self.get_parameter("input_topic").value)
        self.out_topic = str(self.get_parameter("output_topic").value)

        self.r_meas = float(self.get_parameter("r_meas").value)
        self.q_x = float(self.get_parameter("q_x").value)
        self.q_v = float(self.get_parameter("q_v").value)

        self.nis_th = float(self.get_parameter("nis_threshold").value)
        self.jump_th = float(self.get_parameter("jump_threshold").value)
        self.dt_max = float(self.get_parameter("dt_max").value)

        self.kf = None
        self.last_t = None
        self.last_raw = None

        self.pub = self.create_publisher(Range, self.out_topic, 10)
        self.sub = self.create_subscription(Range, self.in_topic, self.cb, 10)

        self.get_logger().info(f"RangeKalmanFilter: {self.in_topic} -> {self.out_topic}")

    def cb(self, msg: Range):
        # 入力がRangeとして妥当かを軽くチェック
        z = float(msg.range)
        if math.isnan(z) or math.isinf(z):
            return
        if z < msg.min_range or z > msg.max_range:
            return

        t = stamp_to_sec(msg.header.stamp)
        if t <= 0.0:
            t = self.get_clock().now().nanoseconds * 1e-9

        if self.kf is None:
            self.kf = Kalman1D(x0=z, v0=0.0, p_x=0.5, p_v=1.0)
            self.last_t = t
            self.last_raw = z
            self._publish_filtered(msg, self.kf.x[0])
            return

        dt = t - self.last_t
        if dt <= 0.0:
            dt = 1e-2
        dt = min(dt, self.dt_max)

        # predict
        self.kf.predict(dt, q_x=self.q_x, q_v=self.q_v)

        # ジャンプ抑制（まずは超単純に）
        if self.last_raw is not None and abs(z - self.last_raw) > self.jump_th:
            # 外れ値として更新しない（予測だけで出す）
            self._publish_filtered(msg, self.kf.x[0])
            self.last_t = t
            return

        # NISゲート
        y, S = self.kf.innovation(z, self.r_meas)
        nis = (y * y) / max(S, 1e-9)
        if nis <= self.nis_th:
            self.kf.update(z, r=self.r_meas)
        # else: 更新しない（予測のみ）

        self.last_t = t
        self.last_raw = z
        self._publish_filtered(msg, self.kf.x[0])

    def _publish_filtered(self, src_msg: Range, filtered_range: float):
        out = Range()
        out.header = src_msg.header
        out.radiation_type = src_msg.radiation_type
        out.field_of_view = src_msg.field_of_view
        out.min_range = src_msg.min_range
        out.max_range = src_msg.max_range

        # 範囲内にクリップ
        out.range = max(out.min_range, min(float(filtered_range), out.max_range))

        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = RangeKalmanFilterNode()
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
