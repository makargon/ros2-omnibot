import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

import lgpio

class StartNode(Node):
    def __init__(self):
        super().__init__('start_node')
    
        self.chip = lgpio.gpiochip_open(4)
        self.publish_rate = 10
        self.pin = 18
        self.trigger_on_close = True
        
        lgpio.gpio_claim_input(self.chip, self.pin, lgpio.SET_PULL_UP)
        # lgpio.gpio_set_pull_up_down(self.chip, self.pin, lgpio.PULL_UP)

        self.pub_ = self.create_publisher(Bool, '/start', 10)
        self.timer = self.create_timer(1.0 / self.publish_rate, self.check_reed_switch)
        self.started = False

    def check_reed_switch(self):
        value = lgpio.gpio_read(self.chip, self.pin)
        if self.trigger_on_close:
            activated = (value == 0)
        else:
            activated = (value == 1)

        if activated:
            msg = Bool()
            msg.data = True
            self.start_pub.publish(msg)
            self.timer.cancel()

    def destroy_node(self) -> bool:
        lgpio.gpiochip_close(self.chip)
        return super().destroy_node()

def main(args=None) -> None:
    rclpy.init(args=args)
    node = StartNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()