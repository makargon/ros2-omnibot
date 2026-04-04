# actuator node
from dataclasses import dataclass
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int32MultiArray

import lgpio # возможно, не нужно - можно через board обращаться
import pwmio
import board

from adafruit_pca9685 import PCA9685
from adafruit_motor import servo, motor

@dataclass
class ServoControl:
    channel: int
    min_angle: int
    max_angle: int
    angel: int

@dataclass
class MotorControl:
    pwm_channel: int
    in_a: int
    in_b: int
    pca: PCA9685
    chip: lgpio.gpiod_chip
    # не уверена насчет стоит ли сохранять состояние
    def set_dir(self, dir: bool):
        if dir:
            pass

    def set_speed(self, speed: float):
        lgpio.gpio_write(self.chip, self.in_a, speed)


class ActuatorNode(Node):
    def __init__(self):
        super().__init__('actuators')

        # пока без параметров

        self.pca = PCA9685(board.I2C(), address=0x40)
        self.chip = lgpio.gpiochip_open(4)
        lgpio.gpio_claim_output(self.chip, 5)
        lgpio.gpio_claim_output(self.chip, 6)
        lgpio.gpio_claim_output(self.chip, 17)
        lgpio.gpio_claim_output(self.chip, 22)
        lgpio.gpio_claim_output(self.chip, 26)
        lgpio.gpio_claim_output(self.chip, 27)
        
        self.motors = [
            MotorControl(pwm_channel=0, in_a=5, in_b=6, pca=self.pca, chip=self.chip),
            MotorControl(pwm_channel=1, in_a=17, in_b=22, pca=self.pca, chip=self.chip),
            MotorControl(pwm_channel=2, in_a=26, in_b=27, pca=self.pca, chip=self.chip),
        ]

        self.servos = [
            servo.Servo(pwm_out=self.pca.channels[4], actuation_range=160),
            servo.Servo(pwm_out=self.pca.channels[5], actuation_range=160),
            servo.Servo(pwm_out=self.pca.channels[6], actuation_range=160)
        ]

        self.sub_motors = self.create_subscription(
            Float32MultiArray,
            '/motor_speeds',
            self.motor_callback,
            10
        )

        self.sub_servos = self.create_subscription(
            Int32MultiArray,
            '/servo_angles',
            self.servo_callback,
            10
        )

    def motor_callback(self, msg: Float32MultiArray):
        pass
        # for motor, speed in zip(self.motors, msg.data):
        #     motor.set_speed(speed)

    def servo_callback(self, msg: Int32MultiArray):
        for servo, angle in zip(self.servos, msg.data):
            servo.angle = angle

    def destroy_node(self) -> bool:
        # for i in range(3):
        #     self.set(i, 0.0)
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