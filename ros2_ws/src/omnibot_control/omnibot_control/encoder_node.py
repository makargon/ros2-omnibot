import lgpio
from typing import Any
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray

class RotaryEncoder:
    def __init__(self, chip: Any, pin_a: int, pin_b: int):
        self.handle = chip
        self.pin_a = pin_a
        self.pin_b = pin_b
        self.counter: int = 0
        self.lock = threading.Lock()

        lgpio.gpio_claim_input(self.handle, self.pin_a)
        lgpio.gpio_claim_input(self.handle, self.pin_b)

        self.last_a = lgpio.gpio_read(self.handle, self.pin_a)
        self.last_b = lgpio.gpio_read(self.handle, self.pin_b)

        self.cb_a = lgpio.callback(self.handle, self.pin_a, lgpio.BOTH_EDGES, self._on_change)
        self.cb_b = lgpio.callback(self.handle, self.pin_b, lgpio.BOTH_EDGES, self._on_change)

    def _on_change(self, chip, gpio, level, timestamp):
        a = lgpio.gpio_read(self.handle, self.pin_a)
        b = lgpio.gpio_read(self.handle, self.pin_b)

        if a == self.last_a and b == self.last_b:
            return
        
        delta = 0
        if self.last_a == 0 and self.last_b == 0:
            if a == 0 and b == 1:
                delta = -1
            elif a == 1 and b == 0:
                delta = +1
        elif self.last_a == 0 and self.last_b == 1:
            if a == 1 and b == 1:
                delta = -1
            elif a == 0 and b == 0:
                delta = +1
        elif self.last_a == 1 and self.last_b == 0:
            if a == 0 and b == 0:
                delta = -1
            elif a == 1 and b == 1:
                delta = +1
        elif self.last_a == 1 and self.last_b == 1:
            if a == 1 and b == 0:
                delta = -1
            elif a == 0 and b == 1:
                delta = +1
                
        if delta != 0:
            with self.lock:
                self.counter += delta

        self.last_a = a
        self.last_b = b

    def count(self) -> int:
        with self.lock:
            return self.counter

    def reset(self):
        with self.lock:
            self.counter = 0

    def stop(self):
        self.cb_a.cancel()
        self.cb_b.cancel()

class EncoderNode(Node):
    def __init__(self):
        super().__init__('encoder_node')

        self._log_counter = 0

        self.chip = lgpio.gpiochip_open(4)

        self.encoders = [
            RotaryEncoder(self.chip, 10, 9),
            RotaryEncoder(self.chip, 13, 19),
            RotaryEncoder(self.chip, 20, 21)
        ]

        self.pub = self.create_publisher(
            Int32MultiArray,
            '/encoders_counter',
            10
        )

        self.timer = self.create_timer(0.05, self.publish_count)

    def publish_count(self):
        msg = Int32MultiArray()
        counts = []
        for enc in self.encoders:
            counts.append(enc.count())
            enc.reset()
        # counts = [enc.count() for enc in self.encoders]
        msg.data = counts
        self.pub.publish(msg)

        self._log_counter += 1
        if self._log_counter % 40 == 0:
            self.get_logger().info(f"Состояние энкодеров: {counts}")

    def destroy_node(self) -> bool:
        for enc in self.encoders:
            enc.stop()
        lgpio.gpiochip_close(self.chip)
        return super().destroy_node()

def main(args=None) -> None:
    rclpy.init(args=args)
    node = EncoderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()