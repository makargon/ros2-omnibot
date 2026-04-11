import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class DemoNode(Node):
    def __init__(self):
        super().__init__('demo_node')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        loop_rate = 10  # Hz
        self._loop_rate = self.create_rate(loop_rate, self.get_clock())
        self.timer = self.create_timer(4.0, self.timer_callback)
        self.get_logger().info('Demo node started')

    def timer_callback(self):
        """Бесконечно двигается по квадрату раз в секунду"""
        self.get_logger().info('Start')
        msg = Twist()
        msg.angular.z = 0.0
        msg.linear.x = 0.2
        msg.linear.y = 0.0
        self.publisher.publish(msg)
        self.get_logger().info('Published cmd_vel: linear.x=%.2f angular.z=%.2f' % (msg.linear.x, msg.angular.z))
        self._loop_rate.sleep(1)
        msg.linear.x = 0.0
        msg.linear.y = 0.2
        self.publisher.publish(msg)
        self.get_logger().info('Published cmd_vel: linear.x=%.2f linear.y=%.2f' % (msg.linear.x, msg.linear.y))
        self._loop_rate.sleep(1)
        msg.linear.x = -0.2
        msg.linear.y = 0.0
        self.publisher.publish(msg)
        self.get_logger().info('Published cmd_vel: linear.x=%.2f angular.z=%.2f' % (msg.linear.x, msg.angular.z))
        self._loop_rate.sleep(1)
        msg.linear.x = 0.0
        msg.linear.y = -0.2
        self.publisher.publish(msg)
        self.get_logger().info('Published cmd_vel: linear.y=%.2f' % (msg.linear.y))
        self._loop_rate.sleep(1)


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