import traceback
import time
from typing import List, Optional

import rclpy

from omnibot_mission.robot_api import MarkerInfo, RobotMissionAPI


class PickupDeliveryMission(RobotMissionAPI):
    """
    Пример миссии:
      1) едем в точку 1
      2) опускаем сервоприводы
      3) едем в точку 2
      4) поднимаем сервоприводы
      5) едем в координаты ArUco-метки
      6) поднимаем груз
      7) едем домой
    """

    # Координаты можно вынести в параметры/конфиг позже.
    POINT_1 = (1.20, 0.40, 0.0)
    POINT_2 = (2.10, 0.35, 0.0)
    HOME = (0.00, 0.00, 0.0)
    TARGET_MARKER_ID = 7

    # Пример углов сервоприводов.
    SERVOS_DOWN = [120.0, 120.0, 120.0]
    SERVOS_UP = [35.0, 35.0, 35.0]
    SERVOS_LIFT_CARGO = [20.0, 20.0, 20.0]

    def __init__(self) -> None:
        super().__init__()

        self.target_marker_id = int(self.declare_parameter('target_marker_id', self.TARGET_MARKER_ID).value)
        marker_ids_param = list(self.declare_parameter('target_marker_ids', []).value)
        self.target_marker_ids: List[int] = [int(v) for v in marker_ids_param if str(v).strip() != '']
        if self.target_marker_id not in self.target_marker_ids:
            self.target_marker_ids.insert(0, self.target_marker_id)

        self.prefer_nearest_marker = bool(self.declare_parameter('prefer_nearest_marker', True).value)
        self.marker_search_timeout_sec = float(self.declare_parameter('marker_search_timeout_sec', 12.0).value)

        self.get_logger().info(
            f'Marker selection configured: ids={self.target_marker_ids}, '
            f'prefer_nearest={self.prefer_nearest_marker}'
        )

    def _select_target_marker(self, timeout_sec: float) -> Optional[MarkerInfo]:
        deadline = time.time() + max(0.1, timeout_sec)
        best = None

        while rclpy.ok() and time.time() < deadline:
            self.spin_once(0.05)
            pose = self.get_robot_pose(prefer_aruco=True)

            candidates = []
            for marker_id in self.target_marker_ids:
                marker = self.get_aruco_marker(marker_id, max_age_sec=1.5)
                if marker is not None:
                    candidates.append(marker)

            if not candidates:
                continue

            if (not self.prefer_nearest_marker) or pose is None:
                return candidates[0]

            best = min(candidates, key=lambda m: (m.x - pose.x) ** 2 + (m.y - pose.y) ** 2)
            return best

        return best

    def _go(self, x: float, y: float, yaw: float, step_name: str) -> bool:
        self.get_logger().info(f'{step_name}: moving to x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}')

        # Сначала пытаемся через SLAM/Nav2, если нет — локальный fallback.
        ok = self.move_to_point_slam(x=x, y=y, yaw=yaw, frame_id='map', timeout_sec=90.0)
        if ok:
            return True

        self.get_logger().warn(f'{step_name}: SLAM goal unavailable, fallback to local controller')
        return self.move_to_point(x=x, y=y, yaw=yaw, timeout_sec=60.0, prefer_aruco=True)

    def run_mission(self) -> bool:
        self.get_logger().info('PickupDeliveryMission started')

        if not self.wait_for_robot_pose(timeout_sec=10.0, prefer_aruco=True):
            self.get_logger().error('No robot pose available to start mission')
            return False

        # 1) В первую точку
        if not self._go(*self.POINT_1, step_name='Step 1'):
            return False

        # 2) Опустить сервоприводы
        self.get_logger().info('Step 2: lower servos')
        self.set_servos_deg(self.SERVOS_DOWN)
        self.sleep(1.0)

        # 3) Во вторую точку
        if not self._go(*self.POINT_2, step_name='Step 3'):
            return False

        # 4) Поднять сервоприводы
        self.get_logger().info('Step 4: raise servos')
        self.set_servos_deg(self.SERVOS_UP)
        self.sleep(1.0)

        # 5) В координаты ArUco метки
        self.get_logger().info(f'Step 5: searching marker from ids={self.target_marker_ids}')
        marker = self._select_target_marker(timeout_sec=self.marker_search_timeout_sec)
        if marker is None:
            self.get_logger().error('Target ArUco marker not found')
            return False

        self.get_logger().info(
            f'Step 5: selected marker id={marker.marker_id} at x={marker.x:.2f}, y={marker.y:.2f}'
        )

        if not self._go(marker.x, marker.y, 0.0, step_name='Step 5'):
            return False

        # 6) Поднять груз
        self.get_logger().info('Step 6: lift cargo')
        self.set_servos_deg(self.SERVOS_LIFT_CARGO)
        self.sleep(1.0)

        # 7) Домой
        if not self._go(*self.HOME, step_name='Step 7'):
            return False

        self.stop()
        self.get_logger().info('PickupDeliveryMission finished successfully')
        return True


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PickupDeliveryMission()

    try:
        success = node.run_mission()
        if not success:
            node.get_logger().error('Mission failed')
    except Exception as exc:  # pragma: no cover
        node.get_logger().error(f'Unhandled mission exception: {exc}')
        node.get_logger().error(traceback.format_exc())
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()
