#!/usr/bin/env python3
import select
import sys
import termios
import tty
import math
from typing import List

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


class ManualControlNode(Node):
    def __init__(self) -> None:
        super().__init__('manual_control')

        # Публикации для actuator_node
        self.motor_pub = self.create_publisher(Float32MultiArray, '/motor_speeds', 10)
        self.servo_pub = self.create_publisher(Float32MultiArray, '/servo_angles', 10)

        # Шаги изменения
        self.linear_step = 0.10
        self.angular_step = 0.25
        self.max_linear = 1.0
        self.max_angular = 2.0

        # Скорости в платформе (робота)
        self.vx = 0.0
        self.vy = 0.0
        self.wz = 0.0

        # Сервы (hand, rotate, grab)
        self.servo_step = 5.0
        self.servo_min = 0.0
        self.servo_max = 180.0
        self.servo_angles: List[float] = [90.0, 90.0, 90.0]

        # Таймер для публикации скоростей моторов (10 Гц)
        self.create_timer(0.1, self._publish_motor_speeds)
        # Начальная публикация углов серв
        self._publish_servo_angles()

        self.get_logger().info("Manual control node started (omni kinematics)")

    # ---------- Кинематика трёхколёсного омнибота ----------
    # Расположение моторов под углами 0°, 120°, 240° (ось X вперёд)
    # Скорости моторов:
    #   m1 =  vx                     + L * wz
    #   m2 = -vx*sin30 - vy*cos30    + L * wz
    #   m3 = -vx*sin30 + vy*cos30    + L * wz
    # где L – коэффициент влияния поворота (обычно 0.5..1.0)
    # Для простоты возьмём L = 0.5 (робот поворачивается плавно)
    # Но лучше масштабировать так, чтобы максимальная скорость мотора не превышала 1.
    # Здесь мы просто вычисляем и затем клиппируем.
    def _compute_motor_speeds(self, vx: float, vy: float, wz: float) -> List[float]:
        L = 0.5  # коэффициент поворота, подберите под своего робота
        sin30 = 0.5
        cos30 = math.sqrt(3.0) / 2.0

        m1 = vx + L * wz
        m2 = -vx * sin30 - vy * cos30 + L * wz
        m3 = -vx * sin30 + vy * cos30 + L * wz

        # Нормализация (если какое-то значение вышло за [-1, 1], масштабируем все)
        max_abs = max(abs(m1), abs(m2), abs(m3))
        if max_abs > 1.0:
            m1 /= max_abs
            m2 /= max_abs
            m3 /= max_abs

        return [float(m1), float(m2), float(m3)]

    def _publish_motor_speeds(self) -> None:
        speeds = self._compute_motor_speeds(self.vx, self.vy, self.wz)
        msg = Float32MultiArray()
        msg.data = speeds
        self.motor_pub.publish(msg)

    def _publish_servo_angles(self) -> None:
        msg = Float32MultiArray()
        msg.data = self.servo_angles[:]  # копия
        self.servo_pub.publish(msg)

    def _update_servo(self, idx: int, delta: float) -> None:
        new_angle = self.servo_angles[idx] + delta
        self.servo_angles[idx] = max(self.servo_min, min(self.servo_max, new_angle))
        self._publish_servo_angles()

    def stop_motion(self) -> None:
        self.vx = 0.0
        self.vy = 0.0
        self.wz = 0.0
        # Скорости опубликуются по таймеру

    def handle_key(self, key: str) -> bool:
        k = key.lower()

        # Движение
        if k == 'w':
            self.vx = max(-self.max_linear, min(self.max_linear, self.vx + self.linear_step))
        elif k == 's':
            self.vx = max(-self.max_linear, min(self.max_linear, self.vx - self.linear_step))
        elif k == 'a':
            self.vy = max(-self.max_linear, min(self.max_linear, self.vy + self.linear_step))
        elif k == 'd':
            self.vy = max(-self.max_linear, min(self.max_linear, self.vy - self.linear_step))
        elif k == 'q':
            self.wz = max(-self.max_angular, min(self.max_angular, self.wz + self.angular_step))
        elif k == 'e':
            self.wz = max(-self.max_angular, min(self.max_angular, self.wz - self.angular_step))

        # Сервы: hand (индекс 0), rotate (1), grab (2)
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
            f'vel: vx={self.vx:.2f} vy={self.vy:.2f} wz={self.wz:.2f} | '
            f'servos: [{self.servo_angles[0]:.0f}, {self.servo_angles[1]:.0f}, {self.servo_angles[2]:.0f}]'
        )
        return True

    @staticmethod
    def print_help() -> None:
        print('Manual control for omnibot (3 motors, 3 servos)')
        print('  Motion: W/S = forward/back, A/D = left/right, Q/E = rotate left/right')
        print('  Servos: I/K = hand +/- , J/L = rotate +/- , U/O = grab +/-')
        print('  Space or X = stop motion, R = reset servos to 90°, H = help, Ctrl+C = exit')


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