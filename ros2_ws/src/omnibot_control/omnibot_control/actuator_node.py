from dataclasses import dataclass
from typing import Any
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int32MultiArray

import lgpio
import board

from adafruit_pca9685 import PCA9685
from adafruit_motor import servo

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
        # speed должен быть в [0.0, 1.0]
        # TODO проверка значений в топике?
        duty = int(speed * 65535)
        self.pca.channels[self.pwm_channel].duty_cycle = duty


class ActuatorNode(Node):
    def __init__(self):
        super().__init__('actuators')

        # пока без параметров
        try:
            self.pca = PCA9685(board.I2C(), address=0x40)
            self.pca.frequency = 50
            self.get_logger().info('PCA9685 успешно инициализирован по адресу 0x40')
        except Exception as e:
            self.get_logger().error(f'Не удалось подключиться к PCA9685: {e}')
            raise
        
        serv = servo.Servo(self.pca.channels[6])
        for i in range(0, 120):
            serv.angle = i
            print(f'angle = {i}')
            time.sleep(0.03)

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

        self.servos = [
            servo.Servo(pwm_out=self.pca.channels[4], actuation_range=160),
            servo.Servo(pwm_out=self.pca.channels[5], actuation_range=160),
            servo.Servo(pwm_out=self.pca.channels[6], actuation_range=160)
        ]

        # self.sub_motors = self.create_subscription(
        #     Float32MultiArray,
        #     '/motor_speeds',
        #     self.motor_callback,
        #     10
        # )

        self.sub_servos = self.create_subscription(
            Int32MultiArray,
            '/servo_angles',
            self.servo_callback,
            10
        )

        # for motor in self.motors:
        #     motor.set_dir(1)

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
        self.pca.deinit()
        return super().destroy_node()

def main(args=None) -> None:
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