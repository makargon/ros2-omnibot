#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

def quaternion_to_yaw(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

class GoalController(Node):
    def __init__(self):
        super().__init__('goal_controller')

        # Параметры регулятора
        self.declare_parameter('control_rate', 20.0)
        self.declare_parameter('max_linear', 0.5)
        self.declare_parameter('max_angular', 1.5)
        self.declare_parameter('kp_linear', 0.8)
        self.declare_parameter('kp_angular', 1.2)
        self.declare_parameter('goal_tolerance', 0.05)
        self.declare_parameter('angle_tolerance', 0.05)

        self.control_rate = self.get_parameter('control_rate').value
        self.max_linear = self.get_parameter('max_linear').value
        self.max_angular = self.get_parameter('max_angular').value
        self.kp_linear = self.get_parameter('kp_linear').value
        self.kp_angular = self.get_parameter('kp_angular').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        self.angle_tolerance = self.get_parameter('angle_tolerance').value

        # Маршрут: список точек (x, y, yaw) – yaw = None означает сохранять текущий угол
        self.waypoints = [
            (1.0, 0.0, None),
            (0.0, 0.0, None),
            (0.0, 1.0, None),
            (1.0, 1.0, None)
        ]
        self.current_waypoint_idx = 0

        # Текущая цель
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

        # Устанавливаем первую цель
        self.set_waypoint_as_goal()

        self.get_logger().info("Goal Controller started (automatic waypoint mode).")
        self.get_logger().info(f"Waypoints: {self.waypoints}")

    def set_waypoint_as_goal(self):
        if self.current_waypoint_idx < len(self.waypoints):
            x, y, yaw = self.waypoints[self.current_waypoint_idx]
            self.goal_x = x
            self.goal_y = y
            if yaw is not None:
                self.goal_yaw = self.normalize_angle(yaw)
            self.get_logger().info(
                f"Moving to waypoint {self.current_waypoint_idx+1}/{len(self.waypoints)}: ({x:.2f}, {y:.2f})"
            )
        else:
            self.get_logger().info("All waypoints reached. Robot stopped.")

    def advance_to_next_waypoint(self):
        self.current_waypoint_idx += 1
        if self.current_waypoint_idx < len(self.waypoints):
            self.set_waypoint_as_goal()
        else:
            self.get_logger().info("Final waypoint reached. Mission complete.")
            self.stop_robot()

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

    def stop_robot(self):
        """Остановка робота и отключение таймера."""
        self.goal_x = self.current_x
        self.goal_y = self.current_y
        self.goal_yaw = self.current_yaw
        twist = Twist()
        self.cmd_pub.publish(twist)
        self.timer.cancel()
        self.get_logger().info("Robot stopped and controller disabled.")

    def control_loop(self):
        if not self.odom_received:
            return

        dx = self.goal_x - self.current_x
        dy = self.goal_y - self.current_y
        dyaw = self.normalize_angle(self.goal_yaw - self.current_yaw)

        distance = math.hypot(dx, dy)

        # Проверка достижения текущей цели
        if distance < self.goal_tolerance and abs(dyaw) < self.angle_tolerance:
            self.advance_to_next_waypoint()
            return

        # Управление движением
        angle_to_goal = math.atan2(dy, dx)
        angle_error = self.normalize_angle(angle_to_goal - self.current_yaw)

        if abs(angle_error) > 0.2:
            vx = 0.0
            vy = 0.0
            wz = self.kp_angular * angle_error
        else:
            vx = self.kp_linear * (dx * math.cos(self.current_yaw) + dy * math.sin(self.current_yaw))
            vy = self.kp_linear * (-dx * math.sin(self.current_yaw) + dy * math.cos(self.current_yaw))
            wz = self.kp_angular * dyaw

        vx = max(-self.max_linear, min(self.max_linear, vx))
        vy = max(-self.max_linear, min(self.max_linear, vy))
        wz = max(-self.max_angular, min(self.max_angular, wz))

        twist = Twist()
        twist.linear.x = vx
        twist.linear.y = vy
        twist.angular.z = wz
        self.cmd_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = GoalController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()