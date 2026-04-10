import traceback

import rclpy

from .robot_api import RobotMissionAPI


class MissionTermoNode(RobotMissionAPI):

    def run_mission(self) -> bool:
        self.get_logger().info('Mission "not cold, not hot" started')

        # 1) Дождаться доступной оценки позы
        if not self.wait_for_robot_pose(timeout_sec=10.0, prefer_aruco=True):
            self.get_logger().error('No robot pose available (Aruco/odom).')
            return False

        self.set_servos_deg([30.0, 30.0, 30.0])
        
        # Перемещения по координатам
        ok = self.move_to_point(x=0.0, y=1.8, yaw=0.0)
        if not ok:
            return False
        
        self.set_servos_deg([30.0, 60.0, 30.0])

        ok = self.move_to_point(x=0.5, y=1.8, yaw=0.0)
        if not ok:
            return False

        self.set_servos_deg([30.0, 30.0, 30.0])


        self.stop()
        self.get_logger().info('Mission "not cold, not hot" finished')
        return True


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MissionTermoNode()

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
