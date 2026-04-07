import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from math import pi


class TFBroadcasterNode(Node):
    """
    Broadcasts static transforms for the robot structure.
    
    Transforms:
        - odom -> base_link
        - base_link -> lidar_link
    """
    
    def __init__(self):
        super().__init__('tf_broadcaster')
        
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Parameters
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('lidar_frame', 'lidar_link')
        self.declare_parameter('lidar_x', 0.0)
        self.declare_parameter('lidar_y', 0.0)
        self.declare_parameter('lidar_z', 0.1)
        
        self.base_frame = self.get_parameter('base_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.lidar_frame = self.get_parameter('lidar_frame').value
        self.lidar_x = self.get_parameter('lidar_x').value
        self.lidar_y = self.get_parameter('lidar_y').value
        self.lidar_z = self.get_parameter('lidar_z').value
        
        # Create timer to publish transforms
        self.timer = self.create_timer(0.1, self._publish_transforms)
        
        self.get_logger().info('TF Broadcaster node initialized')
    
    def _publish_transforms(self):
        """Publish static transforms."""
        # odom -> base_link transform
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t)
        
        # base_link -> lidar_link transform
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.base_frame
        t.child_frame_id = self.lidar_frame
        t.transform.translation.x = self.lidar_x
        t.transform.translation.y = self.lidar_y
        t.transform.translation.z = self.lidar_z
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = TFBroadcasterNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
