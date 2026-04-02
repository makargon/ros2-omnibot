import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import serial
import struct
import time
from threading import Thread
import math


class RPLiDARNode(Node):
    """
    Node for reading data from RPLiDAR A1M8 via USB and publishing as LaserScan.
    
    Published topics:
        /scan (sensor_msgs/LaserScan) - laser scan data
    """
    
    def __init__(self):
        super().__init__('rplidar_node')
        
        # Parameters
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('frame_id', 'lidar_link')
        self.declare_parameter('angle_min', -math.pi)
        self.declare_parameter('angle_max', math.pi)
        self.declare_parameter('range_min', 0.15)
        self.declare_parameter('range_max', 12.0)
        self.declare_parameter('scan_time', 0.1)
        
        self.port = self.get_parameter('port').value
        self.baudrate = self.get_parameter('baudrate').value
        self.frame_id = self.get_parameter('frame_id').value
        self.angle_min = self.get_parameter('angle_min').value
        self.angle_max = self.get_parameter('angle_max').value
        self.range_min = self.get_parameter('range_min').value
        self.range_max = self.get_parameter('range_max').value
        self.scan_time = self.get_parameter('scan_time').value
        
        # Publisher
        self.scan_publisher = self.create_publisher(LaserScan, 'scan', 10)
        
        # Serial connection
        self.serial_port = None
        self.running = False
        self.connect_timeout = 5.0
        
        # Start lidar communication thread
        self.lidar_thread = Thread(target=self._lidar_loop, daemon=True)
        self.lidar_thread.start()
        
        self.get_logger().info(
            f'RPLiDAR node initialized. Port: {self.port}, '
            f'Baudrate: {self.baudrate}'
        )
    
    def _lidar_loop(self):
        """Background thread for LIDAR communication."""
        while rclpy.ok():
            try:
                if self.serial_port is None or not self.serial_port.is_open:
                    self._connect_lidar()
                
                if self.serial_port and self.serial_port.is_open:
                    self._read_and_publish_scan()
                    
            except Exception as e:
                self.get_logger().error(f'LIDAR error: {e}')
                self._disconnect_lidar()
                time.sleep(1)
    
    def _connect_lidar(self):
        """Connect to LIDAR via serial port."""
        try:
            self.serial_port = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=1.0,
                write_timeout=1.0
            )
            self.running = True
            self.get_logger().info(f'Connected to LIDAR on {self.port}')
            
            # Start LIDAR scan
            self._start_scan()
            
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to connect to LIDAR: {e}')
            self.serial_port = None
            raise
    
    def _disconnect_lidar(self):
        """Disconnect from LIDAR."""
        if self.serial_port and self.serial_port.is_open:
            try:
                self.serial_port.close()
            except:
                pass
        self.serial_port = None
        self.running = False
    
    def _start_scan(self):
        """Send start scan command to LIDAR."""
        # SCAN command: 0xA5 0x20
        try:
            self.serial_port.write(bytes([0xA5, 0x20]))
            self.serial_port.flush()
            # Wait for response
            response = self.serial_port.read(5)
            if response and len(response) >= 5:
                self.get_logger().info('LIDAR scan started')
            else:
                self.get_logger().warn('No response from LIDAR scan command')
        except Exception as e:
            self.get_logger().error(f'Failed to start scan: {e}')
    
    def _read_and_publish_scan(self):
        """Read scan data from LIDAR and publish as LaserScan message."""
        try:
            scan_msg = LaserScan()
            scan_msg.header.frame_id = self.frame_id
            scan_msg.header.stamp = self.get_clock().now().to_msg()
            scan_msg.angle_min = self.angle_min
            scan_msg.angle_max = self.angle_max
            scan_msg.angle_increment = (self.angle_max - self.angle_min) / 360
            scan_msg.time_increment = self.scan_time / 360
            scan_msg.scan_time = self.scan_time
            scan_msg.range_min = self.range_min
            scan_msg.range_max = self.range_max
            
            ranges = []
            
            # Read capsule data from LIDAR
            # Note: This is a simplified implementation
            # For full RPLiDAR support, consider using rplidar_ros package
            while len(ranges) < 360:
                if self.serial_port.in_waiting >= 5:
                    # Read data point
                    data = self.serial_port.read(5)
                    if len(data) == 5:
                        # Parse RPLiDAR response
                        # Format depends on specific command/mode
                        # This is a placeholder for actual parsing
                        quality = data[0] >> 2
                        angle_raw = ((data[1] & 0x3F) << 8) | data[2]
                        distance_raw = (data[3] << 8) | data[4]
                        
                        if distance_raw > 0:
                            distance = distance_raw / 4.0  # Convert to meters
                            ranges.append(distance)
                        else:
                            ranges.append(float('inf'))
                    
                    if len(ranges) >= 360:
                        break
                else:
                    time.sleep(0.001)
            
            scan_msg.ranges = ranges
            self.scan_publisher.publish(scan_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error reading scan: {e}')
    
    def destroy_node(self):
        """Cleanup on node destruction."""
        self.running = False
        self._disconnect_lidar()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RPLiDARNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
