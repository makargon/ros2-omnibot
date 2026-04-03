import os
import threading
from typing import Optional, Tuple

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image


class CameraNode(Node):
    def __init__(self) -> None:
        super().__init__('camera_node')

        self.declare_parameter('device', 0)
        self.declare_parameter('frame_id', 'camera_link')
        self.declare_parameter('width', 1280)
        self.declare_parameter('height', 720)
        self.declare_parameter('fps', 30.0)
        self.declare_parameter('publish_rectified', True)
        self.declare_parameter('calibration_file', '')
        self.declare_parameter('camera_info_topic', '/camera/camera_info')
        self.declare_parameter('image_raw_topic', '/camera/image_raw')
        self.declare_parameter('image_rect_topic', '/camera/image_rect')

        self.device = self.get_parameter('device').value
        self.frame_id = self.get_parameter('frame_id').value
        self.width = int(self.get_parameter('width').value)
        self.height = int(self.get_parameter('height').value)
        self.fps = float(self.get_parameter('fps').value)
        self.publish_rectified = bool(self.get_parameter('publish_rectified').value)
        self.calibration_file = self.get_parameter('calibration_file').value
        self.camera_info_topic = self.get_parameter('camera_info_topic').value
        self.image_raw_topic = self.get_parameter('image_raw_topic').value
        self.image_rect_topic = self.get_parameter('image_rect_topic').value

        self.bridge = CvBridge()
        self.raw_pub = self.create_publisher(Image, self.image_raw_topic, 10)
        self.rect_pub = self.create_publisher(Image, self.image_rect_topic, 10)
        self.info_pub = self.create_publisher(CameraInfo, self.camera_info_topic, 10)

        self.camera = self._open_camera(self.device)
        self.camera_lock = threading.Lock()
        self.map1 = None
        self.map2 = None
        self.camera_info_msg = self._load_camera_info(self.calibration_file)
        self.last_frame_size: Optional[Tuple[int, int]] = None

        if self.camera_info_msg is not None and self.publish_rectified:
            self._prepare_rectification_maps()

        timer_period = 1.0 / max(self.fps, 1.0)
        self.timer = self.create_timer(timer_period, self._capture_and_publish)
        self.get_logger().info('USB camera node started.')

    def _open_camera(self, device):
        if isinstance(device, str) and device.isdigit():
            device = int(device)
        capture = cv2.VideoCapture(device)
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, float(self.width))
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, float(self.height))
        capture.set(cv2.CAP_PROP_FPS, float(self.fps))
        capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        if not capture.isOpened():
            self.get_logger().warn(f'Could not open camera device: {device}')
        return capture

    def _load_camera_info(self, calibration_file: str) -> Optional[CameraInfo]:
        if not calibration_file:
            return None
        if not os.path.exists(calibration_file):
            self.get_logger().warn(f'Calibration file not found: {calibration_file}')
            return None

        try:
            import yaml
        except ImportError:
            self.get_logger().warn('PyYAML is not installed; camera calibration will be disabled.')
            return None

        with open(calibration_file, 'r', encoding='utf-8') as file_handle:
            data = yaml.safe_load(file_handle)

        camera_matrix = data.get('camera_matrix', {}).get('data', [])
        distortion = data.get('distortion_coefficients', {}).get('data', [])
        image_width = int(data.get('image_width', self.width))
        image_height = int(data.get('image_height', self.height))
        distortion_model = data.get('distortion_model', 'plumb_bob')

        camera_info = CameraInfo()
        camera_info.width = image_width
        camera_info.height = image_height
        camera_info.distortion_model = distortion_model
        camera_info.d = [float(value) for value in distortion]
        camera_info.k = [float(value) for value in camera_matrix[:9]]
        camera_info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        camera_info.p = [
            camera_info.k[0], camera_info.k[1], camera_info.k[2], 0.0,
            camera_info.k[3], camera_info.k[4], camera_info.k[5], 0.0,
            camera_info.k[6], camera_info.k[7], camera_info.k[8], 0.0,
        ]
        return camera_info

    def _prepare_rectification_maps(self) -> None:
        if self.camera_info_msg is None:
            return
        k = np.array(self.camera_info_msg.k, dtype=np.float64).reshape(3, 3)
        d = np.array(self.camera_info_msg.d, dtype=np.float64).reshape(-1, 1)
        if d.size == 4:
            d = d.reshape(4, 1)
        size = (self.camera_info_msg.width, self.camera_info_msg.height)
        self.map1, self.map2 = cv2.fisheye.initUndistortRectifyMap(
            k,
            d,
            np.eye(3),
            k,
            size,
            cv2.CV_16SC2,
        )

    def _capture_and_publish(self) -> None:
        with self.camera_lock:
            if self.camera is None or not self.camera.isOpened():
                return
            ok, frame = self.camera.read()
        if not ok or frame is None:
            self.get_logger().warn('Failed to read frame from camera.')
            return

        height, width = frame.shape[:2]
        self.last_frame_size = (width, height)
        timestamp = self.get_clock().now().to_msg()

        raw_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        raw_msg.header.stamp = timestamp
        raw_msg.header.frame_id = self.frame_id
        self.raw_pub.publish(raw_msg)

        if self.camera_info_msg is not None:
            camera_info = self.camera_info_msg
            camera_info.header.stamp = timestamp
            camera_info.header.frame_id = self.frame_id
            self.info_pub.publish(camera_info)

        if self.publish_rectified and self.map1 is not None and self.map2 is not None:
            rectified = cv2.remap(frame, self.map1, self.map2, interpolation=cv2.INTER_LINEAR)
        else:
            rectified = frame

        rect_msg = self.bridge.cv2_to_imgmsg(rectified, encoding='bgr8')
        rect_msg.header.stamp = timestamp
        rect_msg.header.frame_id = self.frame_id
        self.rect_pub.publish(rect_msg)

    def destroy_node(self) -> bool:
        if self.camera is not None:
            with self.camera_lock:
                self.camera.release()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
