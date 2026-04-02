import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray

class EncoderTicksNode(Node):
    def __init__(self):
        super().__init__('encoder_ticks_node')

        self.declare_parameter('encoder_1.A', 17)
        self.declare_parameter('encoder_1.B', 17)
        self.declare_parameter('encoder_2.A', 27)
        self.declare_parameter('encoder_2.B', 27)
        self.declare_parameter('encoder_3.A', 22)
        self.declare_parameter('encoder_3.B', 22)

        self.ticks = [0, 0, 0]

        self.publisher_ = self.create_publisher(Int32MultiArray, '/encoder_ticks', 10)

        timer_period = 0.02  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info("Encoder Ticks Node запущена.")

    def timer_callback(self):

        tick_1 = self.ticks[0]
        tick_2 = self.ticks[1]
        tick_3 = self.ticks[2]

        self.ticks = [tick_1, tick_2, tick_3]

        msg = Int32MultiArray()
        msg.data = self.ticks

        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = EncoderTicksNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()