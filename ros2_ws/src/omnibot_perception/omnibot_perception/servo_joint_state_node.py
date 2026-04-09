import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray, Int32MultiArray


class ServoJointStateNode(Node):
    """Convert servo angle commands into /joint_states for gripper URDF."""

    def __init__(self) -> None:
        super().__init__('servo_joint_state_node')

        # Topics
        self.declare_parameter('servo_topic_int', '/servo_angles')
        self.declare_parameter('servo_topic_deg', '/servo_angles_deg')
        self.declare_parameter('joint_state_topic', '/joint_states')

        # Input servo range (degrees)
        self.declare_parameter('servo_min_deg', 0.0)
        self.declare_parameter('servo_max_deg', 160.0)

        # Output joint ranges from URDF
        self.declare_parameter('arm_pitch_min_rad', -1.57)
        self.declare_parameter('arm_pitch_max_rad', 1.57)
        self.declare_parameter('left_pad_min_rad', -math.pi)
        self.declare_parameter('left_pad_max_rad', math.pi)
        self.declare_parameter('right_pad_min_m', 0.0)
        self.declare_parameter('right_pad_max_m', 0.03)

        # Publish rate
        self.declare_parameter('publish_hz', 20.0)

        self.servo_topic_int = str(self.get_parameter('servo_topic_int').value)
        self.servo_topic_deg = str(self.get_parameter('servo_topic_deg').value)
        self.joint_state_topic = str(self.get_parameter('joint_state_topic').value)

        self.servo_min_deg = float(self.get_parameter('servo_min_deg').value)
        self.servo_max_deg = float(self.get_parameter('servo_max_deg').value)

        self.arm_pitch_min_rad = float(self.get_parameter('arm_pitch_min_rad').value)
        self.arm_pitch_max_rad = float(self.get_parameter('arm_pitch_max_rad').value)
        self.left_pad_min_rad = float(self.get_parameter('left_pad_min_rad').value)
        self.left_pad_max_rad = float(self.get_parameter('left_pad_max_rad').value)
        self.right_pad_min_m = float(self.get_parameter('right_pad_min_m').value)
        self.right_pad_max_m = float(self.get_parameter('right_pad_max_m').value)

        publish_hz = float(self.get_parameter('publish_hz').value)
        publish_hz = max(1.0, publish_hz)

        # Last known servo command: [servo4, servo5, servo6]
        self.servo_deg = [80.0, 80.0, 80.0]

        self.create_subscription(Int32MultiArray, self.servo_topic_int, self._on_servo_int, 10)
        self.create_subscription(Float32MultiArray, self.servo_topic_deg, self._on_servo_deg, 10)

        self.joint_pub = self.create_publisher(JointState, self.joint_state_topic, 20)
        self.timer = self.create_timer(1.0 / publish_hz, self._publish_joint_state)

        self.joint_names = [
            'arm_pitch_joint',
            'left_pad_rotation_joint',
            'right_finger_prismatic_joint',
        ]

        self.get_logger().info(
            f'Publishing {self.joint_state_topic} from {self.servo_topic_int}/{self.servo_topic_deg}'
        )

    @staticmethod
    def _clamp(value: float, low: float, high: float) -> float:
        return max(low, min(high, value))

    def _map_linear(
        self,
        value: float,
        in_min: float,
        in_max: float,
        out_min: float,
        out_max: float,
    ) -> float:
        if in_max <= in_min:
            return out_min
        t = (value - in_min) / (in_max - in_min)
        t = self._clamp(t, 0.0, 1.0)
        return out_min + t * (out_max - out_min)

    def _update_servo(self, data) -> None:
        if len(data) < 3:
            self.get_logger().warn('Expected 3 servo values: [servo4, servo5, servo6]')
            return
        self.servo_deg[0] = float(data[0])
        self.servo_deg[1] = float(data[1])
        self.servo_deg[2] = float(data[2])

    def _on_servo_int(self, msg: Int32MultiArray) -> None:
        self._update_servo(msg.data)

    def _on_servo_deg(self, msg: Float32MultiArray) -> None:
        self._update_servo(msg.data)

    def _publish_joint_state(self) -> None:
        s4 = self._clamp(self.servo_deg[0], self.servo_min_deg, self.servo_max_deg)
        s5 = self._clamp(self.servo_deg[1], self.servo_min_deg, self.servo_max_deg)
        s6 = self._clamp(self.servo_deg[2], self.servo_min_deg, self.servo_max_deg)

        arm_pitch = self._map_linear(
            s4,
            self.servo_min_deg,
            self.servo_max_deg,
            self.arm_pitch_min_rad,
            self.arm_pitch_max_rad,
        )
        left_pad_rot = self._map_linear(
            s5,
            self.servo_min_deg,
            self.servo_max_deg,
            self.left_pad_min_rad,
            self.left_pad_max_rad,
        )
        right_pad_slide = self._map_linear(
            s6,
            self.servo_min_deg,
            self.servo_max_deg,
            self.right_pad_min_m,
            self.right_pad_max_m,
        )

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = [arm_pitch, left_pad_rot, right_pad_slide]
        self.joint_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ServoJointStateNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
