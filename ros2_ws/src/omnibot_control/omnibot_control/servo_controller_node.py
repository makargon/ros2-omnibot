from typing import List

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray

try:
	from smbus2 import SMBus  # type: ignore
except Exception:  # pragma: no cover
	SMBus = None


class PCA9685:
	MODE1 = 0x00
	MODE2 = 0x01
	PRESCALE = 0xFE
	LED0_ON_L = 0x06

	def __init__(self, bus_num: int, address: int = 0x40, pwm_hz: int = 50):
		self._enabled = False
		self._bus = None
		self._address = address
		self._last_error = ''

		if SMBus is None:
			self._last_error = 'smbus2 not installed'
			return

		try:
			self._bus = SMBus(bus_num)
			self._write_reg(self.MODE1, 0x00)
			self._write_reg(self.MODE2, 0x04)
			self.set_pwm_freq(pwm_hz)
			self._enabled = True
		except Exception as e:
			self._last_error = str(e)
			self._enabled = False

	@property
	def enabled(self) -> bool:
		return self._enabled

	@property
	def last_error(self) -> str:
		return self._last_error

	def _write_reg(self, reg: int, value: int) -> None:
		if self._bus is None:
			return
		self._bus.write_byte_data(self._address, reg, value & 0xFF)

	def set_pwm_freq(self, pwm_hz: int) -> None:
		if self._bus is None:
			return
		hz = max(24, min(1526, int(pwm_hz)))
		prescale = int(round(25_000_000.0 / (4096.0 * float(hz)) - 1.0))

		old_mode = self._bus.read_byte_data(self._address, self.MODE1)
		sleep_mode = (old_mode & 0x7F) | 0x10
		self._write_reg(self.MODE1, sleep_mode)
		self._write_reg(self.PRESCALE, prescale)
		self._write_reg(self.MODE1, old_mode)
		self._write_reg(self.MODE1, old_mode | 0xA1)

	def set_channel_duty(self, channel: int, duty: float) -> None:
		if not self._enabled or self._bus is None or channel < 0 or channel > 15:
			return

		base = self.LED0_ON_L + 4 * channel
		d = max(0.0, min(1.0, duty))
		if d <= 0.0:
			self._write_reg(base + 0, 0x00)
			self._write_reg(base + 1, 0x00)
			self._write_reg(base + 2, 0x00)
			self._write_reg(base + 3, 0x10)
			return

		if d >= 1.0:
			self._write_reg(base + 0, 0x00)
			self._write_reg(base + 1, 0x10)
			self._write_reg(base + 2, 0x00)
			self._write_reg(base + 3, 0x00)
			return

		off = int(d * 4095.0)
		self._write_reg(base + 0, 0x00)
		self._write_reg(base + 1, 0x00)
		self._write_reg(base + 2, off & 0xFF)
		self._write_reg(base + 3, (off >> 8) & 0x0F)

	def close(self) -> None:
		if self._bus is not None:
			self._bus.close()


class ServoControllerNode(Node):
	def __init__(self) -> None:
		super().__init__('servo_controller')

		self.declare_parameter('pca9685.i2c_bus', 1)
		self.declare_parameter('pca9685.address', 0x40)
		self.declare_parameter('pca9685.pwm_hz', 50)

		self.declare_parameter('servo.channels', [4, 5, 6])
		self.declare_parameter('servo.min_deg', 0.0)
		self.declare_parameter('servo.max_deg', 180.0)
		self.declare_parameter('servo.min_pulse_us', 500.0)
		self.declare_parameter('servo.max_pulse_us', 2500.0)
		self.declare_parameter('servo.default_deg', [90.0, 90.0, 90.0])

		i2c_bus = int(self.get_parameter('pca9685.i2c_bus').value)
		addr = int(self.get_parameter('pca9685.address').value)
		self.pwm_hz = int(self.get_parameter('pca9685.pwm_hz').value)

		self.channels = [int(v) for v in self.get_parameter('servo.channels').value]
		self.min_deg = float(self.get_parameter('servo.min_deg').value)
		self.max_deg = float(self.get_parameter('servo.max_deg').value)
		self.min_pulse_us = float(self.get_parameter('servo.min_pulse_us').value)
		self.max_pulse_us = float(self.get_parameter('servo.max_pulse_us').value)
		self.default_deg = [float(v) for v in self.get_parameter('servo.default_deg').value]

		self.pca = PCA9685(i2c_bus, addr, self.pwm_hz)
		if self.pca.enabled:
			self.get_logger().info('Servo PWM initialized via PCA9685.')
		else:
			self.get_logger().warn(
				f'Servo controller in dry-run mode: {self.pca.last_error or "PCA9685 unavailable"}'
			)

		self.sub = self.create_subscription(
			Float32MultiArray,
			'/servo_angles_deg',
			self._on_servo_angles,
			10,
		)
		self.sub_s4 = self.create_subscription(Float32, '/servo4_deg', self._on_servo4, 10)
		self.sub_s5 = self.create_subscription(Float32, '/servo5_deg', self._on_servo5, 10)
		self.sub_s6 = self.create_subscription(Float32, '/servo6_deg', self._on_servo6, 10)

		self._apply_defaults()

	def _clamp(self, value: float, low: float, high: float) -> float:
		return max(low, min(high, value))

	def _angle_to_duty(self, angle_deg: float) -> float:
		angle = self._clamp(angle_deg, self.min_deg, self.max_deg)
		if self.max_deg <= self.min_deg:
			return 0.0
		t = (angle - self.min_deg) / (self.max_deg - self.min_deg)
		pulse_us = self.min_pulse_us + t * (self.max_pulse_us - self.min_pulse_us)
		period_us = 1_000_000.0 / max(1, self.pwm_hz)
		return self._clamp(pulse_us / period_us, 0.0, 1.0)

	def _set_servo_angle(self, servo_index: int, angle_deg: float) -> None:
		if servo_index < 0 or servo_index >= len(self.channels):
			return
		duty = self._angle_to_duty(angle_deg)
		self.pca.set_channel_duty(self.channels[servo_index], duty)

	def _apply_defaults(self) -> None:
		count = min(len(self.channels), len(self.default_deg))
		for i in range(count):
			self._set_servo_angle(i, self.default_deg[i])

	def _on_servo_angles(self, msg: Float32MultiArray) -> None:
		if len(msg.data) == 0:
			self.get_logger().warn('/servo_angles_deg got empty array')
			return

		count = min(len(msg.data), len(self.channels))
		for i, angle in enumerate(msg.data[:count]):
			self._set_servo_angle(i, float(angle))

	def _on_servo4(self, msg: Float32) -> None:
		self._set_servo_angle(0, float(msg.data))

	def _on_servo5(self, msg: Float32) -> None:
		self._set_servo_angle(1, float(msg.data))

	def _on_servo6(self, msg: Float32) -> None:
		self._set_servo_angle(2, float(msg.data))

	def destroy_node(self) -> bool:
		self.pca.close()
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
