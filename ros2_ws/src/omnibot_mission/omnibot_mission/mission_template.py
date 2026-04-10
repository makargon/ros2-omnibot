import traceback

import rclpy

from .robot_api import RobotMissionAPI


class MissionTemplateNode(RobotMissionAPI):
    """Шаблон миссии: скопируй и заполни шаги в run_mission()."""

    def run_mission(self) -> bool:
        self.get_logger().info('Mission template started')

        # 1) Дождаться доступной оценки позы
        if not self.wait_for_robot_pose(timeout_sec=10.0, prefer_aruco=True):
            self.get_logger().error('No robot pose available (Aruco/odom).')
            return False

        # 2) Пример свободного движения
        # self.move_free(vx=0.1, vy=0.0, wz=0.0, duration_sec=1.0)

        # 3) Пример локального перемещения по координатам
        # ok = self.move_to_point(x=1.0, y=0.5, yaw=0.0)
        # if not ok:
        #     return False

        # 4) Пример постановки SLAM-задачи через Nav2
        # ok = self.move_to_point_slam(x=1.5, y=0.2, yaw=0.0, frame_id='map')
        # if not ok:
        #     self.get_logger().warn('SLAM goal failed or Nav2 unavailable')

        # 5) Пример работы с ArUco
        # marker = self.wait_for_aruco_marker(marker_id=7, timeout_sec=5.0)
        # if marker:
        #     self.get_logger().info(f'Marker 7 at x={marker.x:.2f}, y={marker.y:.2f}')

        # 6) Пример управления сервами
        # self.set_servos_deg([30.0, 30.0, 30.0])

        self.stop()
        self.get_logger().info('Mission template finished')
        return True


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MissionTemplateNode()

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
