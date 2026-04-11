#!/usr/bin/env python3
import math
import sys
import select
import termios
import tty
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

def quaternion_to_yaw(q):
    # yaw = atan2(2*(qy*qz + qw*qx), qw^2 - qx^2 - qy^2 + qz^2)
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

class GoalController(Node):
    def __init__(self):
        super().__init__('goal_controller')

        # Параметры регулятора (можно менять через ros2 param set)
        self.declare_parameter('control_rate', 20.0)      # Гц
        self.declare_parameter('max_linear', 0.5)         # м/с
        self.declare_parameter('max_angular', 1.5)        # рад/с
        self.declare_parameter('kp_linear', 0.8)
        self.declare_parameter('kp_angular', 1.2)
        self.declare_parameter('goal_tolerance', 0.05)    # м
        self.declare_parameter('angle_tolerance', 0.05)   # рад

        self.control_rate = self.get_parameter('control_rate').value
        self.max_linear = self.get_parameter('max_linear').value
        self.max_angular = self.get_parameter('max_angular').value
        self.kp_linear = self.get_parameter('kp_linear').value
        self.kp_angular = self.get_parameter('kp_angular').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        self.angle_tolerance = self.get_parameter('angle_tolerance').value

        # Цель (по умолчанию – нулевая)
        self.goal_x = 0.0
        self.goal_y = 0.0
        self.goal_yaw = 0.0

        # Текущая позиция
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.odom_received = False

        # Подписка на одометрию
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Публикация команд скорости
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Таймер управления
        self.timer = self.create_timer(1.0 / self.control_rate, self.control_loop)

        # Настройка терминала для неблокирующего ввода
        self.fd = sys.stdin.fileno()
        self.old_settings = termios.tcgetattr(self.fd)
        tty.setraw(self.fd)

        self.get_logger().info("Goal Controller started. Enter 'x y [yaw]' to set a new goal, or 'stop' to halt.")
        self.get_logger().info("Example: 1.0 2.0 1.57   (move to x=1.0, y=2.0, yaw=90°)")

    def odom_callback(self, msg: Odometry):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.current_yaw = quaternion_to_yaw(msg.pose.pose.orientation)
        self.odom_received = True

    def normalize_angle(self, angle: float) -> float:
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def set_goal(self, x: float, y: float, yaw: float = None):
        """Установить новую цель (координаты и опционально угол)."""
        self.goal_x = x
        self.goal_y = y
        if yaw is not None:
            self.goal_yaw = self.normalize_angle(yaw)
        self.get_logger().info(f"New goal: ({x:.2f}, {y:.2f}, {self.goal_yaw:.2f} rad)")

    def stop_robot(self):
        """Остановить робота и сбросить цель в текущее положение."""
        self.goal_x = self.current_x
        self.goal_y = self.current_y
        self.goal_yaw = self.current_yaw
        self.get_logger().info("Robot stopped, goal reset to current position.")

    def control_loop(self):
        if not self.odom_received:
            return

        # Вычисление ошибок
        dx = self.goal_x - self.current_x
        dy = self.goal_y - self.current_y
        dyaw = self.normalize_angle(self.goal_yaw - self.current_yaw)

        # Если цель достигнута – останавливаемся
        distance = math.hypot(dx, dy)
        if distance < self.goal_tolerance and abs(dyaw) < self.angle_tolerance:
            twist = Twist()
            self.cmd_pub.publish(twist)
            return

        # П-регулятор: скорость пропорциональна расстоянию (ограничена)
        # Движение к точке: линейная скорость направлена к цели, угловая – для разворота на цель
        # Здесь упрощённый вариант: едем прямо к цели, поворачивая при необходимости.
        # Для омниколесного робота можно задавать vx и vy отдельно:
        angle_to_goal = math.atan2(dy, dx)
        angle_error = self.normalize_angle(angle_to_goal - self.current_yaw)

        # Если угол ошибки велик, сначала поворачиваемся, потом едем
        if abs(angle_error) > 0.2:
            vx = 0.0
            vy = 0.0
            wz = self.kp_angular * angle_error
        else:
            # Движение вперёд (или в любом направлении для омни-робота)
            # Здесь для простоты используем vx, vy в локальной системе координат робота
            # Для омни-робота лучше пересчитать ошибку в локальную систему:
            vx = self.kp_linear * (dx * math.cos(self.current_yaw) + dy * math.sin(self.current_yaw))
            vy = self.kp_linear * (-dx * math.sin(self.current_yaw) + dy * math.cos(self.current_yaw))
            wz = self.kp_angular * dyaw   # поддержание заданного угла

        # Ограничения скоростей
        vx = max(-self.max_linear, min(self.max_linear, vx))
        vy = max(-self.max_linear, min(self.max_linear, vy))
        wz = max(-self.max_angular, min(self.max_angular, wz))

        twist = Twist()
        twist.linear.x = vx
        twist.linear.y = vy
        twist.angular.z = wz
        self.cmd_pub.publish(twist)

    def read_goal_from_stdin(self):
        """Неблокирующее чтение строки из stdin."""
        if select.select([self.fd], [], [], 0.0)[0]:
            line = sys.stdin.readline().strip()
            if line:
                parts = line.split()
                if parts[0].lower() == 'stop':
                    self.stop_robot()
                elif len(parts) >= 2:
                    try:
                        x = float(parts[0])
                        y = float(parts[1])
                        yaw = float(parts[2]) if len(parts) >= 3 else None
                        self.set_goal(x, y, yaw)
                    except ValueError:
                        self.get_logger().warn(f"Invalid input: {line}")
                else:
                    self.get_logger().warn("Usage: x y [yaw] or 'stop'")
        return

    def cleanup(self):
        """Восстановление терминала и остановка робота."""
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old_settings)
        stop_msg = Twist()
        self.cmd_pub.publish(stop_msg)
        self.get_logger().info("Goal Controller shut down.")

def main(args=None):
    rclpy.init(args=args)
    node = GoalController()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            node.read_goal_from_stdin()
    except KeyboardInterrupt:
        pass
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()