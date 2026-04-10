from dataclasses import dataclass
from typing import Any

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

    def set_dir(self, dir: float) -> None:
        if dir > 0:
            lgpio.gpio_write(self.chip, self.pin_a, 1)
            lgpio.gpio_write(self.chip, self.pin_b, 0)
        elif  dir < 0:
            lgpio.gpio_write(self.chip, self.pin_b, 1)
            lgpio.gpio_write(self.chip, self.pin_a, 0)
        else:
            lgpio.gpio_write(self.chip, self.pin_b, 0)
            lgpio.gpio_write(self.chip, self.pin_a, 0)

    def set_speed(self, speed: float) -> None: # speed приходит в рад/с
        # макс скорость двигателей = 1.8 м/c => * r => рад/с
        speed /= 1.8 * 0.05
        
        if speed >= 1.0:
            speed = 1.0
        duty = int(speed * 65535)
        self.pca.channels[self.pwm_channel].duty_cycle = duty


class ActuatorNode(Node):
    def __init__(self):
        super().__init__('actuators')

        # self.declare_parameter('pca9685.address', 0x40)
        # self.declare_parameter('pca9685.frequency', 50)
        # self.declare_parameter('gpio_chip', 4)


        # pca_addr = int(self.get_parameter('pca9685.address').value)
        # pca_frec = int(self.get_parameter('pca9685.frequency').value)
        # gpio_chip = int(self.get_parameter('gpio_chip').value)

        try:
            self.pca = PCA9685(board.I2C(), address=0x40)
            self.pca.frequency = 50
            self.get_logger().info('PCA9685 успешно инициализирован по адресу 0x40')
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
            MotorControl(pwm_channel=2, pin_a=26, pin_b=27, pca=self.pca, chip=self.chip),
            MotorControl(pwm_channel=1, pin_a=17, pin_b=22, pca=self.pca, chip=self.chip),
            MotorControl(pwm_channel=0, pin_a=5, pin_b=6, pca=self.pca, chip=self.chip),
        ]

        self.servos = [
            servo.Servo(pwm_out=self.pca.channels[4], actuation_range=160), # grab
            servo.Servo(pwm_out=self.pca.channels[5], actuation_range=160),
            servo.Servo(pwm_out=self.pca.channels[6], actuation_range=120) # hand
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

        self.servos[2].angle = 100

    def motor_callback(self, msg: Float32MultiArray) -> None:
        for motor, speed in zip(self.motors, msg.data):
            motor.set_dir(speed)
            motor.set_speed(abs(speed))

    # TODO Для серв необходима калибровка - непонятно в каком положении сейчас находится -> нет смысла ставить ограничения
    def servo_callback(self, msg: Int32MultiArray) -> None:
        # i: int = 4
        for serv, angle in zip(self.servos, msg.data):
            # if i == 4 and angle > 30:
                serv.angle = angle
            # if i == 6 and angle > 64:
                # serv.angle = angle
            # i += 1

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