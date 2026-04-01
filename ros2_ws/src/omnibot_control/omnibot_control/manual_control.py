import select
import sys
import termios
import tty
from typing import List

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


class ManualControlNode(Node):
    def __init__(self) -> None:
        super().__init__('manual_control')

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.servo_pub = self.create_publisher(Float32MultiArray, '/servo_angles_deg', 10)

        self.linear_step = 0.10
        self.angular_step = 0.25
        self.max_linear = 1.0
        self.max_angular = 2.0

        self.servo_step = 5.0
        self.servo_min = 0.0
        self.servo_max = 180.0
        self.servo_angles: List[float] = [90.0, 90.0, 90.0]

        self.vx = 0.0
        self.vy = 0.0
        self.wz = 0.0

        self.create_timer(0.1, self._publish_cmd_vel)  # 10 Hz hold
        self._publish_servo_angles()

    @staticmethod
    def _clamp(value: float, low: float, high: float) -> float:
        return max(low, min(high, value))

    def _publish_cmd_vel(self) -> None:
        msg = Twist()
        msg.linear.x = self.vx
        msg.linear.y = self.vy
        msg.angular.z = self.wz
        self.cmd_pub.publish(msg)

    def _publish_servo_angles(self) -> None:
        msg = Float32MultiArray()
        msg.data = list(self.servo_angles)
        self.servo_pub.publish(msg)

    def _update_servo(self, idx: int, delta: float) -> None:
        self.servo_angles[idx] = self._clamp(
            self.servo_angles[idx] + delta,
            self.servo_min,
            self.servo_max,
        )
        self._publish_servo_angles()

    def stop_motion(self) -> None:
        self.vx = 0.0
        self.vy = 0.0
        self.wz = 0.0
        self._publish_cmd_vel()

    def handle_key(self, key: str) -> bool:
        k = key.lower()

        # Motion: WASD (linear), QE (yaw)
        if k == 'w':
            self.vx = self._clamp(self.vx + self.linear_step, -self.max_linear, self.max_linear)
        elif k == 's':
            self.vx = self._clamp(self.vx - self.linear_step, -self.max_linear, self.max_linear)
        elif k == 'a':
            self.vy = self._clamp(self.vy + self.linear_step, -self.max_linear, self.max_linear)
        elif k == 'd':
            self.vy = self._clamp(self.vy - self.linear_step, -self.max_linear, self.max_linear)
        elif k == 'q':
            self.wz = self._clamp(self.wz + self.angular_step, -self.max_angular, self.max_angular)
        elif k == 'e':
            self.wz = self._clamp(self.wz - self.angular_step, -self.max_angular, self.max_angular)

        # Servos: IK, JL, UO
        elif k == 'i':
            self._update_servo(0, +self.servo_step)
        elif k == 'k':
            self._update_servo(0, -self.servo_step)
        elif k == 'j':
            self._update_servo(1, +self.servo_step)
        elif k == 'l':
            self._update_servo(1, -self.servo_step)
        elif k == 'u':
            self._update_servo(2, +self.servo_step)
        elif k == 'o':
            self._update_servo(2, -self.servo_step)

        elif k in ('x', ' '):
            self.stop_motion()
        elif k == 'r':
            self.servo_angles = [90.0, 90.0, 90.0]
            self._publish_servo_angles()
        elif k == 'h':
            self.print_help()
        elif k == '\x03':  # Ctrl+C
            return False

        self.get_logger().info(
            f'vx={self.vx:.2f} vy={self.vy:.2f} wz={self.wz:.2f} | '
            f'servo=[{self.servo_angles[0]:.1f}, {self.servo_angles[1]:.1f}, {self.servo_angles[2]:.1f}]'
        )
        return True

    @staticmethod
    def print_help() -> None:
        print('Manual control:')
        print('  Motion: W/S=forward/back, A/D=left/right, Q/E=yaw left/right')
        print('  Servos: I/K=servo PWM4 +/-, J/L=servo PWM5 +/-, U/O=servo PWM6 +/-')
        print('  Space or X = stop motion, R = reset servos to 90, H = help, Ctrl+C = exit')


def _get_key(timeout_s: float = 0.1) -> str:
    dr, _, _ = select.select([sys.stdin], [], [], timeout_s)
    if dr:
        return sys.stdin.read(1)
    return ''


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ManualControlNode()
    node.print_help()

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    tty.setcbreak(fd)

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.0)
            key = _get_key(0.1)
            if key:
                if not node.handle_key(key):
                    break
    finally:
        node.stop_motion()
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
