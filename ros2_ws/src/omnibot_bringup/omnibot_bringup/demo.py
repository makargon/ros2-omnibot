import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class DemoNode(Node):
    def __init__(self):
        super().__init__('demo_node')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # Шаги движения по квадрату: вперед, влево, назад, вправо
        self._steps = [
            (0.1, 0.0),
            (0.0, 0.1),
            (-0.1, 0.0),
            (0.0, -0.1),
        ]
        self._step_idx = 0

        # Публикуем новую команду раз в секунду без блокирующего sleep внутри callback.
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.get_logger().info('Demo node started')

    def timer_callback(self):
        """Бесконечно двигается по квадрату, меняя направление раз в секунду."""
        vx, vy = self._steps[self._step_idx]

        msg = Twist()
        msg.linear.x = vx
        msg.linear.y = vy
        msg.angular.z = 0.0
        self.publisher.publish(msg)
        self.get_logger().info(
            'Published cmd_vel: linear.x=%.2f linear.y=%.2f angular.z=%.2f'
            % (msg.linear.x, msg.linear.y, msg.angular.z)
        )

        self._step_idx = (self._step_idx + 1) % len(self._steps)


def main(args=None):
    rclpy.init(args=args)
    node = DemoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()