#!/usr/bin/env python3
import time
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range

import board
import busio
import adafruit_vl53l0x

import lgpio


def now_sec(node: Node) -> float:
    return node.get_clock().now().nanoseconds * 1e-9


class VL53L0XTripletNode(Node):
    """
    XSHUT で 3台を順番に起動し、アドレスを 0x30/0x31/0x32 に振り直してから
    /range/front, /range/left, /range/right に Range を publish するノード。
    """

    def __init__(self):
        super().__init__("vl53l0x_triplet_node")

        # --- params（ピンは仮でOK：後で変えられる） ---
        self.declare_parameter("xshut_front", 17)  # BCM
        self.declare_parameter("xshut_left", 27)
        self.declare_parameter("xshut_right", 22)

        self.declare_parameter("addr_front", 0x30)
        self.declare_parameter("addr_left", 0x31)
        self.declare_parameter("addr_right", 0x32)

        self.declare_parameter("publish_hz", 20.0)

        self.declare_parameter("topic_front", "/range/front")
        self.declare_parameter("topic_left", "/range/left")
        self.declare_parameter("topic_right", "/range/right")

        self.declare_parameter("frame_front", "tof_front")
        self.declare_parameter("frame_left", "tof_left")
        self.declare_parameter("frame_right", "tof_right")

        # Range msg metadata
        self.declare_parameter("min_range_m", 0.02)  # 2cm
        self.declare_parameter("max_range_m", 2.00)  # とりあえず2m（必要に応じて変更）
        self.declare_parameter("fov_rad", 0.5)

        # --- load params ---
        self.xshut = {
            "front": int(self.get_parameter("xshut_front").value),
            "left":  int(self.get_parameter("xshut_left").value),
            "right": int(self.get_parameter("xshut_right").value),
        }
        self.addr = {
            "front": int(self.get_parameter("addr_front").value),
            "left":  int(self.get_parameter("addr_left").value),
            "right": int(self.get_parameter("addr_right").value),
        }
        self.topics = {
            "front": str(self.get_parameter("topic_front").value),
            "left":  str(self.get_parameter("topic_left").value),
            "right": str(self.get_parameter("topic_right").value),
        }
        self.frames = {
            "front": str(self.get_parameter("frame_front").value),
            "left":  str(self.get_parameter("frame_left").value),
            "right": str(self.get_parameter("frame_right").value),
        }

        self.min_range = float(self.get_parameter("min_range_m").value)
        self.max_range = float(self.get_parameter("max_range_m").value)
        self.fov = float(self.get_parameter("fov_rad").value)
        self.hz = float(self.get_parameter("publish_hz").value)

        # publishers
        self.pub = {
            k: self.create_publisher(Range, self.topics[k], 10)
            for k in ["front", "left", "right"]
        }

        # --- GPIO chip open (lgpio) ---
        self.gpioh = lgpio.gpiochip_open(0)
        for k, pin in self.xshut.items():
            lgpio.gpio_claim_output(self.gpioh, pin)
            lgpio.gpio_write(self.gpioh, pin, 0)  # all shutdown

        # --- I2C init (Blinka) ---
        self.i2c = busio.I2C(board.SCL, board.SDA)

        # --- bring up sensors + assign addresses ---
        self.sensors = {}
        self._init_triplet()

        # timer
        period = 1.0 / max(1.0, self.hz)
        self.timer = self.create_timer(period, self.on_timer)

        self.get_logger().info(
            f"VL53L0XTriplet ready. pub_hz={self.hz}, addrs="
            f"{self.addr['front']:02x}/{self.addr['left']:02x}/{self.addr['right']:02x}"
        )

    def _xshut_all(self, level: int):
        for pin in self.xshut.values():
            lgpio.gpio_write(self.gpioh, pin, 1 if level else 0)

    def _bring_up_and_set_addr(self, key: str, new_addr: int):
        """
        key: 'front'/'left'/'right'
        new_addr: 0x30.. etc
        """
        # その1台だけ起動
        lgpio.gpio_write(self.gpioh, self.xshut[key], 1)
        time.sleep(0.05)

        # 0x29 として見えるはずなので生成してアドレス変更
        s = adafruit_vl53l0x.VL53L0X(self.i2c, address=0x29)

        # Adafruitライブラリには set_address がある前提（あなたのコード構成的にこの系統）
        # もし環境で名前が違ったらここだけ変える
        s.set_address(new_addr)
        time.sleep(0.01)

        # 新アドレスで作り直して保持（確実にするため）
        s2 = adafruit_vl53l0x.VL53L0X(self.i2c, address=new_addr)
        self.sensors[key] = s2

        self.get_logger().info(f"{key}: assigned addr=0x{new_addr:02x}")

    def _init_triplet(self):
        # 全台OFF
        self._xshut_all(0)
        time.sleep(0.05)

        # 1台ずつ起動→アドレス割当
        # 注意：割り当て中は他の台はOFFのまま
        self._bring_up_and_set_addr("front", self.addr["front"])
        lgpio.gpio_write(self.gpioh, self.xshut["front"], 1)  # keep ON

        self._bring_up_and_set_addr("left", self.addr["left"])
        lgpio.gpio_write(self.gpioh, self.xshut["left"], 1)

        self._bring_up_and_set_addr("right", self.addr["right"])
        lgpio.gpio_write(self.gpioh, self.xshut["right"], 1)

        # 念のため少し待つ
        time.sleep(0.05)

    def _publish_range(self, key: str, distance_m: float):
        msg = Range()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frames[key]
        msg.radiation_type = Range.INFRARED  # ToFはここをINFRARED扱いにするのが一般的
        msg.field_of_view = self.fov
        msg.min_range = float(self.min_range)
        msg.max_range = float(self.max_range)

        # クリップ
        d = max(msg.min_range, min(float(distance_m), msg.max_range))
        msg.range = d

        self.pub[key].publish(msg)

    def on_timer(self):
        # mm -> m
        for key in ["front", "left", "right"]:
            try:
                mm = int(self.sensors[key].range)
                m = mm / 1000.0
                if not math.isfinite(m):
                    continue
                self._publish_range(key, m)
            except Exception as e:
                self.get_logger().warn(f"{key}: read failed: {e}")

    def destroy_node(self):
        try:
            # 全台OFF
            for pin in self.xshut.values():
                try:
                    lgpio.gpio_write(self.gpioh, pin, 0)
                except Exception:
                    pass
            try:
                lgpio.gpiochip_close(self.gpioh)
            except Exception:
                pass
        finally:
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VL53L0XTripletNode()
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
