from __future__ import annotations

import math
from pathlib import Path
from typing import Dict, Optional, Sequence, Tuple

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose, PoseArray, PoseStamped, TransformStamped
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Int32MultiArray
from tf2_ros import TransformBroadcaster


class ArucoPoseNode(Node):
    def __init__(self) -> None:
        super().__init__('aruco_pose_node')

        self.declare_parameter('image_topic', '/camera/image_rect')
        self.declare_parameter('camera_info_topic', '/camera/camera_info')
        self.declare_parameter('annotated_image_topic', '/aruco/image_annotated')
        self.declare_parameter('detections_topic', '/aruco/detections')
        self.declare_parameter('detection_ids_topic', '/aruco/detection_ids')
        self.declare_parameter('field_markers_topic', '/aruco/field_markers')
        self.declare_parameter('field_marker_ids_topic', '/aruco/field_marker_ids')
        self.declare_parameter('self_robot_pose_topic', '/aruco/self_robot_pose')
        self.declare_parameter('opponent_robot_pose_topic', '/aruco/opponent_robot_pose')
        self.declare_parameter('marker_map_file', '')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('reference_marker_id', 0)
        self.declare_parameter('reference_marker_world_pose', [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.declare_parameter('self_robot_marker_id', 10)
        self.declare_parameter('opponent_robot_marker_id', 20)
        self.declare_parameter('field_marker_ids', [0])
        self.declare_parameter('marker_length', 0.08)
        self.declare_parameter('dictionary', 'DICT_4X4_50')
        self.declare_parameter('publish_debug_image', True)
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('camera_tf_frame', '')
        self.declare_parameter('self_robot_tf_frame', 'robot_self')
        self.declare_parameter('opponent_robot_tf_frame', 'robot_opponent')
        self.declare_parameter('marker_tf_prefix', 'aruco_')
        self.declare_parameter('publish_marker_tfs', True)

        self.image_topic = self.get_parameter('image_topic').value
        self.camera_info_topic = self.get_parameter('camera_info_topic').value
        self.annotated_image_topic = self.get_parameter('annotated_image_topic').value
        self.detections_topic = self.get_parameter('detections_topic').value
        self.detection_ids_topic = self.get_parameter('detection_ids_topic').value
        self.field_markers_topic = self.get_parameter('field_markers_topic').value
        self.field_marker_ids_topic = self.get_parameter('field_marker_ids_topic').value
        self.self_robot_pose_topic = self.get_parameter('self_robot_pose_topic').value
        self.opponent_robot_pose_topic = self.get_parameter('opponent_robot_pose_topic').value
        self.marker_map_file = self.get_parameter('marker_map_file').value
        self.map_frame = self.get_parameter('map_frame').value
        self.reference_marker_id = int(self.get_parameter('reference_marker_id').value)
        self.reference_marker_world_pose = [float(value) for value in self.get_parameter('reference_marker_world_pose').value]
        self.self_robot_marker_id = int(self.get_parameter('self_robot_marker_id').value)
        self.opponent_robot_marker_id = int(self.get_parameter('opponent_robot_marker_id').value)
        self.field_marker_ids = [int(value) for value in self.get_parameter('field_marker_ids').value]
        self.marker_length = float(self.get_parameter('marker_length').value)
        self.dictionary_name = self.get_parameter('dictionary').value
        self.publish_debug_image = bool(self.get_parameter('publish_debug_image').value)
        self.publish_tf = bool(self.get_parameter('publish_tf').value)
        self.camera_tf_frame = self.get_parameter('camera_tf_frame').value
        self.self_robot_tf_frame = self.get_parameter('self_robot_tf_frame').value
        self.opponent_robot_tf_frame = self.get_parameter('opponent_robot_tf_frame').value
        self.marker_tf_prefix = self.get_parameter('marker_tf_prefix').value
        self.publish_marker_tfs = bool(self.get_parameter('publish_marker_tfs').value)

        self.bridge = CvBridge()
        self.camera_matrix: Optional[np.ndarray] = None
        self.dist_coeffs: Optional[np.ndarray] = None
        self.camera_info_received = False
        self.latest_camera_frame_id = 'camera_link'
        self.marker_world_map = self._load_marker_world_map(self.marker_map_file)
        if self.reference_marker_id not in self.marker_world_map:
            self.marker_world_map[self.reference_marker_id] = self._pose_to_transform(self.reference_marker_world_pose)

        if not self.field_marker_ids:
            self.field_marker_ids = sorted(list(self.marker_world_map.keys()))

        self.detection_pub = self.create_publisher(PoseArray, self.detections_topic, 10)
        self.detection_ids_pub = self.create_publisher(Int32MultiArray, self.detection_ids_topic, 10)
        self.field_markers_pub = self.create_publisher(PoseArray, self.field_markers_topic, 10)
        self.field_marker_ids_pub = self.create_publisher(Int32MultiArray, self.field_marker_ids_topic, 10)
        self.self_robot_pose_pub = self.create_publisher(PoseStamped, self.self_robot_pose_topic, 10)
        self.opponent_robot_pose_pub = self.create_publisher(PoseStamped, self.opponent_robot_pose_topic, 10)
        self.debug_image_pub = self.create_publisher(Image, self.annotated_image_topic, 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.image_sub = self.create_subscription(Image, self.image_topic, self._image_callback, 10)
        self.camera_info_sub = self.create_subscription(CameraInfo, self.camera_info_topic, self._camera_info_callback, 10)

        self.dictionary = self._load_dictionary(self.dictionary_name)
        self.detector = self._create_detector(self.dictionary)

        self.get_logger().info('ArUco pose node started.')

    def _load_marker_world_map(self, marker_map_file: str) -> Dict[int, np.ndarray]:
        result: Dict[int, np.ndarray] = {}
        if not marker_map_file:
            return result

        try:
            import yaml
        except ImportError:
            self.get_logger().warn('PyYAML is not installed; marker_map_file will be ignored.')
            return result

        map_path = Path(marker_map_file)
        if not map_path.exists():
            self.get_logger().warn(f'Marker map file not found: {marker_map_file}')
            return result

        with map_path.open('r', encoding='utf-8') as file_handle:
            data = yaml.safe_load(file_handle) or {}

        field_map = data.get('field_map', {})
        file_frame = field_map.get('frame_id')
        if isinstance(file_frame, str) and file_frame:
            self.map_frame = file_frame

        for marker in field_map.get('markers', []):
            try:
                marker_id = int(marker['id'])
                pose_values = marker['pose']
                if not isinstance(pose_values, list) or len(pose_values) != 6:
                    continue
                result[marker_id] = self._pose_to_transform([float(value) for value in pose_values])
            except (KeyError, TypeError, ValueError):
                continue

        self.get_logger().info(f'Loaded {len(result)} map marker(s) from {marker_map_file}.')
        return result

    def _load_dictionary(self, dictionary_name: str):
        dictionary_id = getattr(cv2.aruco, dictionary_name, None)
        if dictionary_id is None:
            self.get_logger().warn(f'Unknown ArUco dictionary {dictionary_name}, falling back to DICT_4X4_50.')
            dictionary_id = cv2.aruco.DICT_4X4_50
        return cv2.aruco.getPredefinedDictionary(dictionary_id)

    def _create_detector(self, dictionary):
        if hasattr(cv2.aruco, 'ArucoDetector'):
            parameters = cv2.aruco.DetectorParameters()
            return cv2.aruco.ArucoDetector(dictionary, parameters)
        return None

    def _camera_info_callback(self, msg: CameraInfo) -> None:
        if msg.k and len(msg.k) >= 9:
            self.camera_matrix = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        if msg.d:
            self.dist_coeffs = np.array(msg.d, dtype=np.float64).reshape(-1, 1)
        self.latest_camera_frame_id = msg.header.frame_id or self.latest_camera_frame_id
        self.camera_info_received = True

    def _detect_markers(self, image: np.ndarray):
        if self.detector is not None:
            corners, ids, rejected = self.detector.detectMarkers(image)
        else:
            corners, ids, rejected = cv2.aruco.detectMarkers(image, self.dictionary)
        return corners, ids, rejected

    def _marker_pose_to_matrix(self, rvec: np.ndarray, tvec: np.ndarray) -> np.ndarray:
        rotation_matrix, _ = cv2.Rodrigues(rvec)
        transform = np.eye(4, dtype=np.float64)
        transform[:3, :3] = rotation_matrix
        transform[:3, 3] = tvec.reshape(3)
        return transform

    def _matrix_to_pose(self, transform: np.ndarray) -> Pose:
        pose = Pose()
        pose.position.x = float(transform[0, 3])
        pose.position.y = float(transform[1, 3])
        pose.position.z = float(transform[2, 3])

        quaternion = self._rotation_matrix_to_quaternion(transform[:3, :3])
        pose.orientation.x = quaternion[0]
        pose.orientation.y = quaternion[1]
        pose.orientation.z = quaternion[2]
        pose.orientation.w = quaternion[3]
        return pose

    def _rotation_matrix_to_quaternion(self, rotation_matrix: np.ndarray) -> Tuple[float, float, float, float]:
        trace = float(np.trace(rotation_matrix))
        if trace > 0.0:
            scale = math.sqrt(trace + 1.0) * 2.0
            w = 0.25 * scale
            x = (rotation_matrix[2, 1] - rotation_matrix[1, 2]) / scale
            y = (rotation_matrix[0, 2] - rotation_matrix[2, 0]) / scale
            z = (rotation_matrix[1, 0] - rotation_matrix[0, 1]) / scale
        elif rotation_matrix[0, 0] > rotation_matrix[1, 1] and rotation_matrix[0, 0] > rotation_matrix[2, 2]:
            scale = math.sqrt(1.0 + rotation_matrix[0, 0] - rotation_matrix[1, 1] - rotation_matrix[2, 2]) * 2.0
            w = (rotation_matrix[2, 1] - rotation_matrix[1, 2]) / scale
            x = 0.25 * scale
            y = (rotation_matrix[0, 1] + rotation_matrix[1, 0]) / scale
            z = (rotation_matrix[0, 2] + rotation_matrix[2, 0]) / scale
        elif rotation_matrix[1, 1] > rotation_matrix[2, 2]:
            scale = math.sqrt(1.0 + rotation_matrix[1, 1] - rotation_matrix[0, 0] - rotation_matrix[2, 2]) * 2.0
            w = (rotation_matrix[0, 2] - rotation_matrix[2, 0]) / scale
            x = (rotation_matrix[0, 1] + rotation_matrix[1, 0]) / scale
            y = 0.25 * scale
            z = (rotation_matrix[1, 2] + rotation_matrix[2, 1]) / scale
        else:
            scale = math.sqrt(1.0 + rotation_matrix[2, 2] - rotation_matrix[0, 0] - rotation_matrix[1, 1]) * 2.0
            w = (rotation_matrix[1, 0] - rotation_matrix[0, 1]) / scale
            x = (rotation_matrix[0, 2] + rotation_matrix[2, 0]) / scale
            y = (rotation_matrix[1, 2] + rotation_matrix[2, 1]) / scale
            z = 0.25 * scale
        return float(x), float(y), float(z), float(w)

    def _pose_to_transform(self, pose_values: Sequence[float]) -> np.ndarray:
        x, y, z, roll, pitch, yaw = pose_values
        cx, sx = math.cos(roll), math.sin(roll)
        cy, sy = math.cos(pitch), math.sin(pitch)
        cz, sz = math.cos(yaw), math.sin(yaw)

        rotation = np.array([
            [cz * cy, cz * sy * sx - sz * cx, cz * sy * cx + sz * sx],
            [sz * cy, sz * sy * sx + cz * cx, sz * sy * cx - cz * sx],
            [-sy, cy * sx, cy * cx],
        ], dtype=np.float64)

        transform = np.eye(4, dtype=np.float64)
        transform[:3, :3] = rotation
        transform[:3, 3] = [x, y, z]
        return transform

    def _transform_inverse(self, transform: np.ndarray) -> np.ndarray:
        inverse = np.eye(4, dtype=np.float64)
        rotation = transform[:3, :3]
        translation = transform[:3, 3]
        inverse[:3, :3] = rotation.T
        inverse[:3, 3] = -rotation.T @ translation
        return inverse

    def _publish_pose_stamped(self, publisher, transform: np.ndarray, frame_id: str, stamp) -> None:
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = frame_id
        pose_msg.header.stamp = stamp
        pose_msg.pose = self._matrix_to_pose(transform)
        publisher.publish(pose_msg)

    def _build_tf(self, parent_frame: str, child_frame: str, transform: np.ndarray, stamp) -> TransformStamped:
        tf_msg = TransformStamped()
        tf_msg.header.stamp = stamp
        tf_msg.header.frame_id = parent_frame
        tf_msg.child_frame_id = child_frame
        tf_msg.transform.translation.x = float(transform[0, 3])
        tf_msg.transform.translation.y = float(transform[1, 3])
        tf_msg.transform.translation.z = float(transform[2, 3])

        quaternion = self._rotation_matrix_to_quaternion(transform[:3, :3])
        tf_msg.transform.rotation.x = quaternion[0]
        tf_msg.transform.rotation.y = quaternion[1]
        tf_msg.transform.rotation.z = quaternion[2]
        tf_msg.transform.rotation.w = quaternion[3]
        return tf_msg

    def _choose_anchor_marker(self, detected_camera_poses: Dict[int, np.ndarray]) -> Optional[int]:
        candidates = [marker_id for marker_id in detected_camera_poses.keys() if marker_id in self.marker_world_map]
        if not candidates:
            return None
        return min(candidates, key=lambda marker_id: float(np.linalg.norm(detected_camera_poses[marker_id][:3, 3])))

    def _image_callback(self, msg: Image) -> None:
        if not self.camera_info_received or self.camera_matrix is None or self.dist_coeffs is None:
            return

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self._detect_markers(gray)

        if ids is None or len(ids) == 0:
            return

        flattened_ids = [int(marker_id) for marker_id in ids.flatten()]
        pose_array = PoseArray()
        pose_array.header.stamp = msg.header.stamp
        pose_array.header.frame_id = self.latest_camera_frame_id

        annotated = frame.copy()
        detected_camera_poses: Dict[int, np.ndarray] = {}

        for index, marker_id in enumerate(flattened_ids):
            marker_corners = corners[index]
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                marker_corners,
                self.marker_length,
                self.camera_matrix,
                self.dist_coeffs,
            )
            rvec = np.array(rvecs[0]).reshape(3, 1)
            tvec = np.array(tvecs[0]).reshape(3, 1)
            transform = self._marker_pose_to_matrix(rvec, tvec)
            detected_camera_poses[marker_id] = transform
            pose_array.poses.append(self._matrix_to_pose(transform))

            if self.publish_debug_image:
                cv2.aruco.drawDetectedMarkers(annotated, [marker_corners], np.array([[marker_id]], dtype=np.int32))
                cv2.drawFrameAxes(annotated, self.camera_matrix, self.dist_coeffs, rvec, tvec, self.marker_length * 0.5)

        ids_msg = Int32MultiArray()
        ids_msg.data = flattened_ids
        self.detection_pub.publish(pose_array)
        self.detection_ids_pub.publish(ids_msg)

        anchor_marker_id = self._choose_anchor_marker(detected_camera_poses)
        if anchor_marker_id is None:
            if self.publish_debug_image:
                self._publish_debug_image(annotated, msg.header.stamp)
            return

        world_t_anchor = self.marker_world_map[anchor_marker_id]
        world_t_camera = world_t_anchor @ self._transform_inverse(detected_camera_poses[anchor_marker_id])

        field_markers = PoseArray()
        field_markers.header.frame_id = self.map_frame
        field_markers.header.stamp = msg.header.stamp
        field_marker_ids = Int32MultiArray()
        tf_messages = []

        if self.publish_tf:
            camera_frame = self.camera_tf_frame if self.camera_tf_frame else self.latest_camera_frame_id
            tf_messages.append(self._build_tf(self.map_frame, camera_frame, world_t_camera, msg.header.stamp))

        for marker_id, camera_t_marker in detected_camera_poses.items():
            world_t_marker = world_t_camera @ camera_t_marker
            if marker_id in self.field_marker_ids:
                field_markers.poses.append(self._matrix_to_pose(world_t_marker))
                field_marker_ids.data.append(marker_id)

            if self.publish_tf and self.publish_marker_tfs:
                tf_messages.append(
                    self._build_tf(self.map_frame, f'{self.marker_tf_prefix}{marker_id}', world_t_marker, msg.header.stamp)
                )

            if marker_id == self.self_robot_marker_id:
                self._publish_pose_stamped(self.self_robot_pose_pub, world_t_marker, self.map_frame, msg.header.stamp)
                if self.publish_tf:
                    tf_messages.append(
                        self._build_tf(self.map_frame, self.self_robot_tf_frame, world_t_marker, msg.header.stamp)
                    )
            elif marker_id == self.opponent_robot_marker_id:
                self._publish_pose_stamped(self.opponent_robot_pose_pub, world_t_marker, self.map_frame, msg.header.stamp)
                if self.publish_tf:
                    tf_messages.append(
                        self._build_tf(self.map_frame, self.opponent_robot_tf_frame, world_t_marker, msg.header.stamp)
                    )

        self.field_markers_pub.publish(field_markers)
        self.field_marker_ids_pub.publish(field_marker_ids)
        if self.publish_tf and tf_messages:
            self.tf_broadcaster.sendTransform(tf_messages)

        if self.publish_debug_image:
            self._publish_debug_image(annotated, msg.header.stamp)

    def _publish_debug_image(self, image: np.ndarray, stamp) -> None:
        image_msg = self.bridge.cv2_to_imgmsg(image, encoding='bgr8')
        image_msg.header.stamp = stamp
        image_msg.header.frame_id = self.latest_camera_frame_id
        self.debug_image_pub.publish(image_msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ArucoPoseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
