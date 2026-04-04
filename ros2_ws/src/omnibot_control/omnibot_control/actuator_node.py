from dataclasses import dataclass
from typing import Any
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int32MultiArray

import lgpio
import board
from smbus2 import SMBus

from adafruit_motor import servo


class PCA9685:
    MODE1 = 0x00
    MODE2 = 0x01
    PRESCALE = 0xFE
    LED0_ON_L = 0x06

    def __init__(self, bus_num: int, address: int = 0x40, pwm_hz: int = 1000):
        self._enabled = False
        self._bus = None
        self._address = address
        self._last_error = ''

        try:
            self._bus = SMBus(bus_num)
            # Restart + auto-increment
            self._write_reg(self.MODE1, 0x00)
            # Totem-pole output
            self._write_reg(self.MODE2, 0x04)
            self.set_pwm_freq(pwm_hz)
            # Disable all channels initially
            for ch in range(16):
                self.set_channel_duty(ch, 0.0)
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
        self._bus.write_byte_data(self._address, reg, value & 0xFF)

    def set_pwm_freq(self, pwm_hz: int) -> None:
        hz = max(24, min(1526, int(pwm_hz)))
        prescale = int(round(25_000_000.0 / (4096.0 * float(hz)) - 1.0))

        old_mode = self._bus.read_byte_data(self._address, self.MODE1)
        sleep_mode = (old_mode & 0x7F) | 0x10
        self._write_reg(self.MODE1, sleep_mode)
        self._write_reg(self.PRESCALE, prescale)
        self._write_reg(self.MODE1, old_mode)
        self._write_reg(self.MODE1, old_mode | 0xA1)

    def set_channel_duty(self, channel: int, duty: float) -> None:
        base = self.LED0_ON_L + 4 * channel
        if duty == 0.0:
            # FULL OFF
            self._write_reg(base + 0, 0x00)
            self._write_reg(base + 1, 0x00)
            self._write_reg(base + 2, 0x00)
            self._write_reg(base + 3, 0x10)
            return
        if duty == 1.0:
            # FULL ON
            self._write_reg(base + 0, 0x00)
            self._write_reg(base + 1, 0x10)
            self._write_reg(base + 2, 0x00)
            self._write_reg(base + 3, 0x00)
            return

        off = int(duty * 4095.0)
        self._write_reg(base + 0, 0x00)
        self._write_reg(base + 1, 0x00)
        self._write_reg(base + 2, off & 0xFF)
        self._write_reg(base + 3, (off >> 8) & 0x0F)

    def close(self) -> None:
        self._bus.close()


@dataclass
class MotorControl:
    pwm_channel: int
    pin_a: int
    pin_b: int
    pca: PCA9685
    chip: Any

    def set_dir(self, dir: bool):
        if dir:
            lgpio.gpio_write(self.chip, self.pin_a, 1)
            lgpio.gpio_write(self.chip, self.pin_b, 0)
        else:
            lgpio.gpio_write(self.chip, self.pin_b, 1)
            lgpio.gpio_write(self.chip, self.pin_a, 0)

    def set_speed(self, speed: float):
        duty = max(0.0, min(1.0, speed))
        self.pca.set_channel_duty(self.pwm_channel, duty)


class ActuatorNode(Node):
    def __init__(self):
        super().__init__('actuators')

        # пока без параметров

        try:
            self.pca = PCA9685(bus_num=1, address=0x40, pwm_hz=50)
            if not self.pca.enabled:
                self.get_logger().error(f'Ошибка инициализации PCA9685: {self.pca.last_error}')
                raise RuntimeError(self.pca.last_error)
            self.get_logger().info('PCA9685 успешно инициализирован по адресу 0x40, частота 50 Гц')
        except Exception as e:
            self.get_logger().error(f'Не удалось подключиться к PCA9685: {e}')
            raise

        self.chip = lgpio.gpiochip_open(4)
        lgpio.gpio_claim_output(self.chip, 5)
        lgpio.gpio_claim_output(self.chip, 6)
        lgpio.gpio_claim_output(self.chip, 17)
        lgpio.gpio_claim_output(self.chip, 22)
        lgpio.gpio_claim_output(self.chip, 26)
        lgpio.gpio_claim_output(self.chip, 27)
        
        self.motors = [
            MotorControl(pwm_channel=0, pin_a=5, pin_b=6, pca=self.pca, chip=self.chip),
            MotorControl(pwm_channel=1, pin_a=17, pin_b=22, pca=self.pca, chip=self.chip),
            MotorControl(pwm_channel=2, pin_a=26, pin_b=27, pca=self.pca, chip=self.chip),
        ]

        # self.servos = [
        #     servo.Servo(pwm_out=self.pca.channels[4], actuation_range=160),
        #     servo.Servo(pwm_out=self.pca.channels[5], actuation_range=160),
        #     servo.Servo(pwm_out=self.pca.channels[6], actuation_range=160)
        # ]

        # self.sub_motors = self.create_subscription(
        #     Float32MultiArray,
        #     '/motor_speeds',
        #     self.motor_callback,
        #     10
        # )

        # self.sub_servos = self.create_subscription(
        #     Int32MultiArray,
        #     '/servo_angles',
        #     self.servo_callback,
        #     10
        # )

        # pwm_channel = self.pca.channels[4]
        # serv = servo.Servo(pwm_channel, actuation_range=160)

        # # serv.angle = 120

        # for i in range(80, 120):
        #     serv.angle = i
        #     self.get_logger().info(f'angle = {i}')
        #     time.sleep(0.03)
        # self.pca.channels[6].duty_cycle = 0xFFFF
        # self.pca.channels[7].duty_cycle = 0xFFFF

        for motor in self.motors:
            motor.set_dir(1)
            motor.set_speed(0.3)

    def motor_callback(self, msg: Float32MultiArray):
        for motor, speed in zip(self.motors, msg.data):
            motor.set_dir(speed > 0)
            motor.set_speed(abs(speed))

    def servo_callback(self, msg: Int32MultiArray):
        for serv, angle in zip(self.servos, msg.data):
            serv.angle = angle

    def destroy_node(self) -> bool:
        for motor in self.motors:
            motor.set_speed(0)
        lgpio.gpiochip_close(self.chip)
        self.pca.close()
        return super().destroy_node()

def main(args=None) -> None:
    # pca = PCA9685(board.I2C(), address=0x40)
    # serv = servo.Servo(pca.channels[4], actuation_range=160)
    # for i in range(0, 120):
    #     serv.angle = i
    #     print(f'angle = {i}')
    #     time.sleep(0.03)
    rclpy.init(args=args)
    node = ActuatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()