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

    def set_dir(self, dir: bool) -> None:
        if dir:
            lgpio.gpio_write(self.chip, self.pin_a, 1)
            lgpio.gpio_write(self.chip, self.pin_b, 0)
        else:
            lgpio.gpio_write(self.chip, self.pin_b, 1)
            lgpio.gpio_write(self.chip, self.pin_a, 0)

    def set_speed(self, speed: float) -> None:
        duty = int(speed * 65535)
        self.pca.channels[self.pwm_channel].duty_cycle = duty


class ActuatorNode(Node):
    def __init__(self):
        super().__init__('actuators')

        self.declare_parameter('i2c_address', 0x40)
        self.declare_parameter('pca_frequency', 50)
        self.declare_parameter('gpio_chip', 4)

        self.declare_parameter('motor_pwm_channels', [0, 1, 2])
        self.declare_parameter('motor_pins_a', [5, 17, 26])
        self.declare_parameter('motor_pins_b', [6, 22, 27])

        self.declare_parameter('servo_channels', [4, 5, 6])
        self.declare_parameter('servo_actuation_range', [270, 160, 270])

        i2c_address = self.get_parameter('i2c_address').value
        pca_frequency = self.get_parameter('pca_frequency').value
        gpio_chip = self.get_parameter('gpio_chip').value

        motor_pwm = self.get_parameter('motor_pwm_channels').get_parameter_value().integer_array_value
        motor_a = self.get_parameter('motor_pins_a').get_parameter_value().integer_array_value
        motor_b = self.get_parameter('motor_pins_b').get_parameter_value().integer_array_value

        servo_channels = self.get_parameter('servo_channels').get_parameter_value().integer_array_value
        servo_ranges = self.get_parameter('servo_actuation_range').get_parameter_value().integer_array_value

        try:
            self.pca = PCA9685(board.I2C(), address=i2c_address)
            self.pca.frequency = pca_frequency
            self.get_logger().info(f'PCA9685 успешно инициализирован по адресу {i2c_address}')
        except Exception as e:
            self.get_logger().error(f'Не удалось подключиться к PCA9685: {e}')
            raise
    
        self.chip = lgpio.gpiochip_open(gpio_chip)
        all_pins = set(motor_a) | set(motor_b)
        for pin in all_pins:
            lgpio.gpio_claim_output(self.chip, pin)
        
        self.motors = []
        for pwm, a, b in zip(motor_pwm, motor_a, motor_b):
            self.motors.append(MotorControl(pwm, a, b, self.pca, self.chip))

        self.servos = []
        for ch, rng in zip(servo_channels, servo_ranges):
            self.servos.append(servo.Servo(pwm_out=ch, actuation_range=rng))

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

    def motor_callback(self, msg: Float32MultiArray) -> None:
        for motor, speed in zip(self.motors, msg.data):
            motor.set_dir(speed > 0)
            motor.set_speed(abs(speed))

    def servo_callback(self, msg: Int32MultiArray) -> None:
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