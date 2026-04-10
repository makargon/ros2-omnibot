import math
import time
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray, Int32MultiArray
from visualization_msgs.msg import MarkerArray

try:
    from rclpy.action import ActionClient
    from nav2_msgs.action import NavigateToPose
except Exception:  # pragma: no cover
    ActionClient = None
    NavigateToPose = None


@dataclass
class MarkerInfo:
    marker_id: int
    x: float
    y: float
    z: float
    stamp_sec: float


@dataclass
class Pose2D:
    x: float
    y: float
    yaw: float


@dataclass
class ScanSafety:
    nearest_m: float
    direction_m: float
    stop: bool
    scale: float


def _quat_to_yaw(z: float, w: float) -> float:
    # Для planar-движения достаточно z/w компоненты кватерниона.
    return math.atan2(2.0 * w * z, 1.0 - 2.0 * z * z)


def _yaw_to_quat(yaw: float) -> Tuple[float, float]:
    half = 0.5 * yaw
    return math.sin(half), math.cos(half)


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def _norm_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class RobotMissionAPI(Node):
    """Удобный API для прикладных миссий Omnibot."""

    def __init__(self) -> None:
        super().__init__('robot_mission_api')

        # Параметры позы и коррекции
        self._aruco_correction_alpha = float(self.declare_parameter('aruco_correction_alpha', 0.35).value)
        self._prefer_fused_pose = bool(self.declare_parameter('prefer_fused_pose', True).value)

        # Параметры лидар-безопасности
        self._scan_topic = str(self.declare_parameter('scan_topic', '/scan').value)
        self._safety_stop_distance_m = float(self.declare_parameter('safety_stop_distance_m', 0.35).value)
        self._safety_slow_distance_m = float(self.declare_parameter('safety_slow_distance_m', 0.80).value)
        self._safety_direction_half_angle_deg = float(self.declare_parameter('safety_direction_half_angle_deg', 25.0).value)
        self._safety_global_half_angle_deg = float(self.declare_parameter('safety_global_half_angle_deg', 35.0).value)
        self._safety_max_angular_when_blocked = float(self.declare_parameter('safety_max_angular_when_blocked', 0.6).value)

        # Управление движением
        self._cmd_pub = self.create_publisher(Twist, '/cmd_vel', 20)

        # Управление сервами (поддержка двух вариантов топика)
        self._servo_pub_deg = self.create_publisher(Float32MultiArray, '/servo_angles_deg', 10)
        self._servo_pub_int = self.create_publisher(Int32MultiArray, '/servo_angles', 10)

        # Координаты робота
        self._aruco_robot_pose: Optional[PoseStamped] = None
        self._odom_pose: Optional[Odometry] = None
        self._aruco_pose2d: Optional[Pose2D] = None
        self._odom_pose2d: Optional[Pose2D] = None

        # Комбинированная поза: одометрия + коррекции ArUco
        self._fused_pose: Optional[Pose2D] = None
        self._last_odom_for_delta: Optional[Pose2D] = None

        # Данные ArUco
        self._markers: Dict[int, MarkerInfo] = {}

        # Последний лидар
        self._last_scan: Optional[LaserScan] = None

        self.create_subscription(PoseStamped, '/aruco/robot_pose', self._on_aruco_robot_pose, 20)
        self.create_subscription(Odometry, '/odom', self._on_odom, 20)
        self.create_subscription(MarkerArray, '/aruco/detected_markers_field', self._on_markers, 20)
        self.create_subscription(LaserScan, self._scan_topic, self._on_scan, 20)

        self.get_logger().info(
            f'RobotMissionAPI initialized: odom=/odom, aruco=/aruco/robot_pose, scan={self._scan_topic}'
        )

    # ---------------------------- callbacks ----------------------------

    def _on_aruco_robot_pose(self, msg: PoseStamped) -> None:
        self._aruco_robot_pose = msg
        p = msg.pose.position
        q = msg.pose.orientation
        aruco_pose = Pose2D(x=float(p.x), y=float(p.y), yaw=_quat_to_yaw(float(q.z), float(q.w)))
        self._aruco_pose2d = aruco_pose

        # Коррекция накопленной одометрии по абсолютным координатам ArUco.
        if self._fused_pose is None:
            self._fused_pose = Pose2D(aruco_pose.x, aruco_pose.y, aruco_pose.yaw)
        else:
            a = _clamp(self._aruco_correction_alpha, 0.0, 1.0)
            yaw_err = _norm_angle(aruco_pose.yaw - self._fused_pose.yaw)
            self._fused_pose = Pose2D(
                x=self._fused_pose.x + a * (aruco_pose.x - self._fused_pose.x),
                y=self._fused_pose.y + a * (aruco_pose.y - self._fused_pose.y),
                yaw=_norm_angle(self._fused_pose.yaw + a * yaw_err),
            )

    def _on_odom(self, msg: Odometry) -> None:
        self._odom_pose = msg
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        odom_pose = Pose2D(x=float(p.x), y=float(p.y), yaw=_quat_to_yaw(float(q.z), float(q.w)))
        self._odom_pose2d = odom_pose

        # Интеграция инкремента одометрии в fused-позу.
        if self._last_odom_for_delta is not None and self._fused_pose is not None:
            dx = odom_pose.x - self._last_odom_for_delta.x
            dy = odom_pose.y - self._last_odom_for_delta.y
            dyaw = _norm_angle(odom_pose.yaw - self._last_odom_for_delta.yaw)
            self._fused_pose = Pose2D(
                x=self._fused_pose.x + dx,
                y=self._fused_pose.y + dy,
                yaw=_norm_angle(self._fused_pose.yaw + dyaw),
            )
        elif self._fused_pose is None:
            self._fused_pose = Pose2D(odom_pose.x, odom_pose.y, odom_pose.yaw)

        self._last_odom_for_delta = odom_pose

    def _on_markers(self, msg: MarkerArray) -> None:
        now_sec = self.get_clock().now().nanoseconds / 1e9
        for marker in msg.markers:
            # В топике есть и сферы, и тексты; для координат берём только sphere.
            if marker.ns != 'detected_markers_field_points':
                continue
            self._markers[int(marker.id)] = MarkerInfo(
                marker_id=int(marker.id),
                x=float(marker.pose.position.x),
                y=float(marker.pose.position.y),
                z=float(marker.pose.position.z),
                stamp_sec=now_sec,
            )

    def _on_scan(self, msg: LaserScan) -> None:
        self._last_scan = msg

    # ---------------------------- utilities ----------------------------

    def spin_once(self, timeout_sec: float = 0.05) -> None:
        rclpy.spin_once(self, timeout_sec=timeout_sec)

    def sleep(self, duration_sec: float, step_sec: float = 0.05) -> None:
        deadline = time.time() + max(0.0, duration_sec)
        while rclpy.ok() and time.time() < deadline:
            self.spin_once(timeout_sec=step_sec)

    # ---------------------------- robot pose ----------------------------

    def get_robot_pose(self, prefer_aruco: bool = True) -> Optional[Pose2D]:
        if self._prefer_fused_pose and self._fused_pose is not None:
            return Pose2D(self._fused_pose.x, self._fused_pose.y, self._fused_pose.yaw)

        if prefer_aruco and self._aruco_robot_pose is not None:
            p = self._aruco_robot_pose.pose.position
            q = self._aruco_robot_pose.pose.orientation
            return Pose2D(x=float(p.x), y=float(p.y), yaw=_quat_to_yaw(float(q.z), float(q.w)))

        if self._odom_pose is not None:
            p = self._odom_pose.pose.pose.position
            q = self._odom_pose.pose.pose.orientation
            return Pose2D(x=float(p.x), y=float(p.y), yaw=_quat_to_yaw(float(q.z), float(q.w)))

        if self._aruco_robot_pose is not None:
            p = self._aruco_robot_pose.pose.position
            q = self._aruco_robot_pose.pose.orientation
            return Pose2D(x=float(p.x), y=float(p.y), yaw=_quat_to_yaw(float(q.z), float(q.w)))

        return None

    def wait_for_robot_pose(self, timeout_sec: float = 5.0, prefer_aruco: bool = True) -> bool:
        deadline = time.time() + timeout_sec
        while rclpy.ok() and time.time() < deadline:
            self.spin_once(0.05)
            if self.get_robot_pose(prefer_aruco=prefer_aruco) is not None:
                return True
        return False

    # ---------------------------- free movement ----------------------------

    def cmd_vel(self, vx: float, vy: float, wz: float) -> None:
        vx, vy, wz = self._apply_collision_safety(vx, vy, wz)
        msg = Twist()
        msg.linear.x = float(vx)
        msg.linear.y = float(vy)
        msg.angular.z = float(wz)
        self._cmd_pub.publish(msg)

    def _scan_min_in_sector(self, center_rad: float, half_width_rad: float) -> Optional[float]:
        scan = self._last_scan
        if scan is None or not scan.ranges:
            return None

        a_min = float(scan.angle_min)
        a_inc = float(scan.angle_increment)
        if abs(a_inc) < 1e-9:
            return None

        best = math.inf
        for i, r in enumerate(scan.ranges):
            rr = float(r)
            if not math.isfinite(rr):
                continue
            if rr < float(scan.range_min) or rr > float(scan.range_max):
                continue
            angle = a_min + i * a_inc
            if abs(_norm_angle(angle - center_rad)) <= half_width_rad:
                if rr < best:
                    best = rr

        return None if not math.isfinite(best) else best

    def _compute_scan_safety(self, vx: float, vy: float) -> ScanSafety:
        motion_norm = math.hypot(vx, vy)
        # Если только вращаемся — не ограничиваем линейно.
        if motion_norm < 1e-6:
            return ScanSafety(nearest_m=math.inf, direction_m=math.inf, stop=False, scale=1.0)

        move_dir = math.atan2(vy, vx)
        direction_half = math.radians(max(1.0, self._safety_direction_half_angle_deg))
        global_half = math.radians(max(1.0, self._safety_global_half_angle_deg))

        direction_min = self._scan_min_in_sector(move_dir, direction_half)
        global_front_min = self._scan_min_in_sector(0.0, global_half)

        candidates = [d for d in (direction_min, global_front_min) if d is not None]
        if not candidates:
            return ScanSafety(nearest_m=math.inf, direction_m=math.inf, stop=False, scale=1.0)

        nearest = min(candidates)
        stop_d = max(0.05, self._safety_stop_distance_m)
        slow_d = max(stop_d + 0.01, self._safety_slow_distance_m)

        if nearest <= stop_d:
            return ScanSafety(nearest_m=nearest, direction_m=direction_min if direction_min is not None else nearest, stop=True, scale=0.0)

        if nearest >= slow_d:
            scale = 1.0
        else:
            scale = (nearest - stop_d) / (slow_d - stop_d)
            scale = _clamp(scale, 0.0, 1.0)

        return ScanSafety(nearest_m=nearest, direction_m=direction_min if direction_min is not None else nearest, stop=False, scale=scale)

    def _apply_collision_safety(self, vx: float, vy: float, wz: float) -> Tuple[float, float, float]:
        safety = self._compute_scan_safety(vx, vy)
        if safety.stop:
            # Полный стоп поступательного движения, но разрешаем медленное разворачивание.
            return 0.0, 0.0, _clamp(wz, -self._safety_max_angular_when_blocked, self._safety_max_angular_when_blocked)

        if safety.scale < 0.999:
            return vx * safety.scale, vy * safety.scale, wz

        return vx, vy, wz

    def stop(self) -> None:
        self.cmd_vel(0.0, 0.0, 0.0)

    def move_free(self, vx: float, vy: float, wz: float, duration_sec: float, rate_hz: float = 20.0) -> None:
        period = 1.0 / max(1.0, rate_hz)
        deadline = time.time() + max(0.0, duration_sec)
        while rclpy.ok() and time.time() < deadline:
            self.cmd_vel(vx, vy, wz)
            self.spin_once(period)
        self.stop()

    # ---------------------------- servo control ----------------------------

    def set_servos_deg(self, angles_deg: List[float]) -> None:
        if len(angles_deg) != 3:
            raise ValueError('Expected exactly 3 servo angles: [servo4, servo5, servo6]')

        clipped = [_clamp(float(a), 0.0, 180.0) for a in angles_deg]

        msg_deg = Float32MultiArray()
        msg_deg.data = clipped
        self._servo_pub_deg.publish(msg_deg)

        msg_int = Int32MultiArray()
        msg_int.data = [int(round(v)) for v in clipped]
        self._servo_pub_int.publish(msg_int)

    # ---------------------------- ArUco data ----------------------------

    def get_aruco_marker(self, marker_id: int, max_age_sec: float = 1.0) -> Optional[MarkerInfo]:
        marker = self._markers.get(int(marker_id))
        if marker is None:
            return None
        now_sec = self.get_clock().now().nanoseconds / 1e9
        if (now_sec - marker.stamp_sec) > max(0.0, max_age_sec):
            return None
        return marker

    def wait_for_aruco_marker(self, marker_id: int, timeout_sec: float = 5.0, max_age_sec: float = 1.0) -> Optional[MarkerInfo]:
        deadline = time.time() + timeout_sec
        while rclpy.ok() and time.time() < deadline:
            self.spin_once(0.05)
            marker = self.get_aruco_marker(marker_id, max_age_sec=max_age_sec)
            if marker is not None:
                return marker
        return None

    # ---------------------------- move to point (local) ----------------------------

    def move_to_point(
        self,
        x: float,
        y: float,
        *,
        yaw: Optional[float] = None,
        timeout_sec: float = 45.0,
        position_tolerance_m: float = 0.07,
        yaw_tolerance_rad: float = 0.15,
        kp_linear: float = 0.9,
        kp_angular: float = 1.6,
        max_linear: float = 0.30,
        max_angular: float = 1.2,
        prefer_aruco: bool = True,
    ) -> bool:
        """Движение к цели по текущей оценке позы (Aruco/odom), без Nav2."""
        deadline = time.time() + timeout_sec

        while rclpy.ok() and time.time() < deadline:
            self.spin_once(0.05)
            pose = self.get_robot_pose(prefer_aruco=prefer_aruco)
            if pose is None:
                continue

            dx = x - pose.x
            dy = y - pose.y
            dist = math.hypot(dx, dy)

            target_heading = math.atan2(dy, dx)
            heading_err = _norm_angle(target_heading - pose.yaw)

            yaw_err = 0.0
            if yaw is not None and dist <= position_tolerance_m:
                yaw_err = _norm_angle(yaw - pose.yaw)

            reached_pos = dist <= position_tolerance_m
            reached_yaw = (yaw is None) or (abs(yaw_err) <= yaw_tolerance_rad)
            if reached_pos and reached_yaw:
                self.stop()
                return True

            vx = 0.0
            vy = 0.0
            wz = 0.0

            if not reached_pos:
                # Перевод в локальную систему робота
                cmd_speed = _clamp(kp_linear * dist, -max_linear, max_linear)
                vx = cmd_speed * math.cos(heading_err)
                vy = cmd_speed * math.sin(heading_err)
                wz = _clamp(kp_angular * heading_err, -max_angular, max_angular)
            else:
                wz = _clamp(kp_angular * yaw_err, -max_angular, max_angular)

            self.cmd_vel(vx, vy, wz)

        self.stop()
        return False

    # ---------------------------- move via SLAM/Nav2 ----------------------------

    def move_to_point_slam(
        self,
        x: float,
        y: float,
        yaw: float = 0.0,
        frame_id: str = 'map',
        timeout_sec: float = 120.0,
    ) -> bool:
        """Постановка задачи на перемещение в Nav2 (если доступно)."""
        if ActionClient is None or NavigateToPose is None:
            self.get_logger().warn('Nav2 action API is unavailable (nav2_msgs not installed).')
            return False

        client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        if not client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn('NavigateToPose action server is not available.')
            return False

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = frame_id
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.position.z = 0.0
        qz, qw = _yaw_to_quat(float(yaw))
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw

        send_future = client.send_goal_async(goal)
        while rclpy.ok() and not send_future.done():
            self.spin_once(0.05)

        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().warn('SLAM goal was rejected.')
            return False

        result_future = goal_handle.get_result_async()
        deadline = time.time() + timeout_sec

        while rclpy.ok() and not result_future.done() and time.time() < deadline:
            self.spin_once(0.05)

        if not result_future.done():
            self.get_logger().warn('SLAM goal timed out, canceling...')
            cancel_future = goal_handle.cancel_goal_async()
            while rclpy.ok() and not cancel_future.done():
                self.spin_once(0.05)
            return False

        result = result_future.result()
        # 4 == STATUS_SUCCEEDED
        return result is not None and int(result.status) == 4
