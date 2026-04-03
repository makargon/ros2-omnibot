from typing import List
from enum import IntEnum

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Int32MultiArray

import board
from adafruit_pca9685 import PCA9685
from adafruit_motor.servo import Servo

# Пины
class ServoIndex(IntEnum):
	HAND = 4
	ROTATE = 5
	GRAB = 6


class ServoControllerNode(Node):
	def __init__(self) -> None:
		super().__init__('servo_controller')

		# self.declare_parameter('pca9685.i2c_bus', 1)
		self.declare_parameter('pca9685.address', 0x40)
		self.declare_parameter('pca9685.pwm_hz', 50)
		self.declare_parameter('servo.default', [90, 90, 90])

		i2c = board.I2C()
		# i2c_bus = int(self.get_parameter('pca9685.i2c_bus').value)
		addr = int(self.get_parameter('pca9685.address').value)
		pwm_hz = int(self.get_parameter('pca9685.pwm_hz').value)
		self.default_angles = self.get_parameter('servo.default').value

		self.pca = PCA9685(i2c, address=addr, reference_clock_speed=pwm_hz)
		self.serv_hand = Servo(self.pca.channels[ServoIndex.HAND], actuation_range=270) # вся рука
		self.serv_rotate = Servo(self.pca.channels[ServoIndex.ROTATE], actuation_range=160) # поворот
		self.serv_grab = Servo(self.pca.channels[ServoIndex.GRAB], actuation_range=270) # захват
		
		self.servos = {
            ServoIndex.HAND: self.serv_hand,
            ServoIndex.ROTATE: self.serv_rotate,
            ServoIndex.GRAB: self.serv_grab,
        }

		self.sub = self.create_subscription(
			Int32MultiArray,
			'/servo_angles_deg',
			self._on_servo_angles,
			10,
		)
		self.sub_s4 = self.create_subscription(
			Int32,
			'/servo_hand',
			lambda msg: self._set_servo_angle(ServoIndex.HAND, msg.data),
			10
		)
		self.sub_s5 = self.create_subscription(
			Int32,
			'/servo_rotate',
			lambda msg: self._set_servo_angle(ServoIndex.ROTATE, msg.data),
			10
		)
		self.sub_s6 = self.create_subscription(
			Int32,
			'/servo_grab',
			lambda msg: self._set_servo_angle(ServoIndex.GRAB, msg.data),
			10
		)

		self._set_defaults()

	def _set_servo_angle(self, servo_index: int, angle_deg: int) -> None:
		servo = self.servos.get(servo_index)
		servo.angle = angle_deg

	def _set_defaults(self) -> None:
		for servo_enum, default_angle in zip(ServoIndex, self.default_angles):
			self._set_servo_angle(servo_enum, default_angle)

	def _on_servo_angles(self, msg: Int32MultiArray) -> None:
		for servo_id, angle in zip(ServoIndex, msg.data):
			self._set_servo_angle(servo_id, angle)

	def destroy_node(self) -> bool:
		self.pca.deinit()
		return super().destroy_node()
	

def main(args=None) -> None:
	rclpy.init(args=args)
	node = ServoControllerNode()
	try:
		rclpy.spin(node)
	except KeyboardInterrupt:
		pass
	finally:
		node.destroy_node()
		rclpy.shutdown()


if __name__ == '__main__':
	main()