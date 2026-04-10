import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray

class KinematicNode(Node):
    def __init__(self):
        super().__init__('kinematic_node')

        self.declare_parameter('wheel_radius', 0.05)
        self.declare_parameter('wheel_base', 0.15)
        self.declare_parameter('cmd_frame_rotation_deg', 0.0)

        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheel_base = self.get_parameter('wheel_base').value
        self.cmd_frame_rotation_deg = self.get_parameter('cmd_frame_rotation_deg').value

        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        self.publisher = self.create_publisher(
            Float32MultiArray,
            '/cmd_wheel',
            10
        )

        self.get_logger().info('Kinematic node started')

    def cmd_vel_callback(self, msg: Twist):
        vx = msg.linear.x
        vy = msg.linear.y
        wz = msg.angular.z

        r = self.wheel_radius
        l = self.wheel_base
        sqrt3 = math.sqrt(3.0)
        coef = 2/math.pi

        w3 = coef * (-vy - l * wz) / r
        w1 = coef * ((sqrt3 / 2) * vx + 0.5 * vy - l * wz ) / r 
        # w1 = coef * (- 0.5 * vx + (sqrt3 / 2) * vy - l * wz ) / r
        w2 = coef * (- (sqrt3 / 2) * vx + 0.5 * vy - l * wz) / r

        wheel_msg = Float32MultiArray()
        wheel_msg.data = [w1, w2, w3]

        self.publisher.publish(wheel_msg)
        self.get_logger().debug(f'Published wheel speeds: {wheel_msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = KinematicNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()