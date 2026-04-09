#!/usr/bin/env python3
import select
import sys
import termios
import tty
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion

class KeyboardGoalController(Node):
    def __init__(self):
        super().__init__('keyboard_goal_controller')

        # Параметры управления
        self.declare_parameter('linear_step', 0.2)       # шаг изменения целевой позиции (м)
        self.declare_parameter('angular_step', 0.3)      # шаг изменения целевого угла (рад)
        self.declare_parameter('kp_linear', 1.0)         # П-коэффициент для линейной скорости
        self.declare_parameter('kp_angular', 1.5)        # П-коэффициент для угловой скорости
        self.declare_parameter('max_linear', 0.5)        # макс. линейная скорость (м/с)
        self.declare_parameter('max_angular', 1.5)       # макс. угловая скорость (рад/с)
        self.declare_parameter('control_rate', 20.0)     # частота управления (Гц)

        self.linear_step = self.get_parameter('linear_step').value
        self.angular_step = self.get_parameter('angular_step').value
        self.kp_linear = self.get_parameter('kp_linear').value
        self.kp_angular = self.get_parameter('kp_angular').value
        self.max_linear = self.get_parameter('max_linear').value
        self.max_angular = self.get_parameter('max_angular').value
        self.control_rate = self.get_parameter('control_rate').value

        # Целевые координаты (в системе odom)
        self.goal_x = 0.0
        self.goal_y = 0.0
        self.goal_yaw = 0.0

        # Текущее положение робота
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.odom_received = False

        # Подписка на одометрию
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Публикация cmd_vel
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Таймер управления
        self.timer = self.create_timer(1.0 / self.control_rate, self.control_loop)

        # Клавиатурный ввод
        self.setup_keyboard()

        self.get_logger().info("Keyboard Goal Controller started. Use keys:")
        self.print_help()

    def setup_keyboard(self):
        self.fd = sys.stdin.fileno()
        self.old_settings = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)

    def odom_callback(self, msg: Odometry):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, self.current_yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.odom_received = True

    def normalize_angle(self, angle: float) -> float:
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def control_loop(self):
        if not self.odom_received:
            return  # ждём первую одометрию

        # Ошибка между целью и текущим положением
        dx = self.goal_x - self.current_x
        dy = self.goal_y - self.current_y
        dyaw = self.normalize_angle(self.goal_yaw - self.current_yaw)

        # П-регулятор
        vx = self.kp_linear * dx
        vy = self.kp_linear * dy
        wz = self.kp_angular * dyaw

        # Ограничение скоростей
        vx = max(-self.max_linear, min(self.max_linear, vx))
        vy = max(-self.max_linear, min(self.max_linear, vy))
        wz = max(-self.max_angular, min(self.max_angular, wz))

        # Если цель достигнута с допуском, останавливаемся
        if abs(dx) < 0.02 and abs(dy) < 0.02 and abs(dyaw) < 0.02:
            vx = vy = wz = 0.0

        twist = Twist()
        twist.linear.x = vx
        twist.linear.y = vy
        twist.angular.z = wz
        self.cmd_pub.publish(twist)

    def handle_key(self, key: str) -> bool:
        k = key.lower()
        if k == 'w':
            self.goal_x += self.linear_step
        elif k == 's':
            self.goal_x -= self.linear_step
        elif k == 'a':
            self.goal_y += self.linear_step
        elif k == 'd':
            self.goal_y -= self.linear_step
        elif k == 'q':
            self.goal_yaw += self.angular_step
        elif k == 'e':
            self.goal_yaw -= self.angular_step
        elif k == 'x' or k == ' ':
            # Сброс цели в текущее положение (остановка)
            self.goal_x = self.current_x
            self.goal_y = self.current_y
            self.goal_yaw = self.current_yaw
        elif k == 'r':
            # Сброс цели в ноль (если нужен возврат в начало координат)
            self.goal_x = 0.0
            self.goal_y = 0.0
            self.goal_yaw = 0.0
        elif k == 'h':
            self.print_help()
        elif k == '\x03':  # Ctrl+C
            return False

        # Нормализация угла цели
        self.goal_yaw = self.normalize_angle(self.goal_yaw)

        self.get_logger().info(
            f"Goal: ({self.goal_x:.2f}, {self.goal_y:.2f}, {self.goal_yaw:.2f})  "
            f"Current: ({self.current_x:.2f}, {self.current_y:.2f}, {self.current_yaw:.2f})"
        )
        return True

    def print_help(self):
        print("\n--- Keyboard Goal Controller ---")
        print("W/S : increase/decrease goal X (forward/back)")
        print("A/D : increase/decrease goal Y (left/right)")
        print("Q/E : increase/decrease goal yaw (rotate)")
        print("Space / X : reset goal to current position (stop)")
        print("R : reset goal to (0,0,0)")
        print("H : this help")
        print("Ctrl+C : exit\n")

    def cleanup(self):
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old_settings)
        # Остановка робота
        stop_msg = Twist()
        self.cmd_pub.publish(stop_msg)

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardGoalController()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.0)
            dr, _, _ = select.select([sys.stdin], [], [], 0.1)
            if dr:
                key = sys.stdin.read(1)
                if not node.handle_key(key):
                    break
    except KeyboardInterrupt:
        pass
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()