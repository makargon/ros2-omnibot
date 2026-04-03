# actuator node
from dataclasses import dataclass
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

import lgpio

from ..chip_control.pca import PCA9685

class MotorControl:
    def __init__(self, pca: PCA9685, chip_handle, pwm_ch: int, in1_pin: int, in2_pin: int):
        self.pca = pca
        self.pwm_ch = pwm_ch
        self.in1 = in1_pin
        self.in2 = in2_pin
        # Запросить GPIO линии как выходы через lgpio
        self.handle = chip_handle
        lgpio.gpio_claim_output(self.handle, in1_pin)
        lgpio.gpio_claim_output(self.handle, in2_pin)
        # Начальное состояние: оба GPIO = 0
        lgpio.gpio_write(self.handle, in1_pin, 0)
        lgpio.gpio_write(self.handle, in2_pin, 0)

    def set_speed(self, speed: float):
        # Ограничение [-1, 1]
        speed = max(-1.0, min(1.0, speed))
        if speed > 0:
            # Вперёд
            lgpio.gpio_write(self.handle, self.in1, 1)
            lgpio.gpio_write(self.handle, self.in2, 0)
        elif speed < 0:
            # Назад
            lgpio.gpio_write(self.handle, self.in1, 0)
            lgpio.gpio_write(self.handle, self.in2, 1)
        else:
            # Стоп
            lgpio.gpio_write(self.handle, self.in1, 0)
            lgpio.gpio_write(self.handle, self.in2, 0)
        # Установка ШИМ (абсолютное значение скорости)
        duty = abs(speed)
        self.pca.set_channel_duty(self.pwm_ch, duty)   # используя ваш метод

class ActuatorNode(Node):
    def __init__(self):
        super().__init__('actuators')
        # Инициализация PCA и lgpio
        self.pca = PCA9685(bus_num=1, address=0x40, pwm_hz=1000)
        self.gpio_handle = lgpio.gpiochip_open(4)   # gpiochip4
        # Создать моторы
        self.motors = [
            MotorControl(self.pca, self.gpio_handle, pwm_ch=0, in1_pin=5, in2_pin=6),
            MotorControl(self.pca, self.gpio_handle, pwm_ch=1, in1_pin=17, in2_pin=22),
            MotorControl(self.pca, self.gpio_handle, pwm_ch=2, in1_pin=26, in2_pin=27),
        ]
        # Подписка на управление (пример: три скорости в одном сообщении)
        self.sub = self.create_subscription(
            Float32MultiArray,
            '/motor_speeds',
            self.speed_callback,
            10
        )

    def speed_callback(self, msg: Float32MultiArray):
        if len(msg.data) != 3:
            self.get_logger().warn('Ожидается 3 скорости')
            return
        for motor, speed in zip(self.motors, msg.data):
            motor.set_speed(speed)

def main(args=None) -> None:
    rclpy.init(args=args)
    node = ActuatorNode()
    try:
        # rclpy.spin_once(node, 10) # timtout for 10 sec
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()