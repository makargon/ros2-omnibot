#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int32MultiArray
import math

class TestActuatorNode(Node):
    def __init__(self):
        super().__init__('test_actuator')
        
        # Издатели
        self.motor_pub = self.create_publisher(Float32MultiArray, '/motor_speeds', 10)
        self.servo_pub = self.create_publisher(Int32MultiArray, '/servo_angles', 10)
        
        # Таймер: публикуем каждые 2 секунды
        self.timer = self.create_timer(2.0, self.timer_callback)
        self.counter = 0
        
        self.get_logger().info("Test actuator node started")
    
    def timer_callback(self):
        # --- Тестовые значения для моторов (3 мотора) ---
        # Скорость от -1.0 до 1.0 (знак задаёт направление)
        # Меняем значения синусоидально для наглядности
        t = self.counter * 0.5  # условное время
        motor_speeds = [
            0.8 * math.sin(t),          # мотор 0
            0.5 * math.sin(t + 2.0),    # мотор 1
            -0.6 * math.sin(t * 0.7)    # мотор 2
        ]
        motor_msg = Float32MultiArray()
        motor_msg.data = motor_speeds
        self.motor_pub.publish(motor_msg)
        
        # --- Тестовые значения для сервоприводов (3 сервы) ---
        # Углы в градусах, должны быть в пределах [0, 160] (actuation_range=160)
        servo_angles = [
            int(80 + 70 * math.sin(t)),           # серва 0: 10..150 град
            int(40 + 30 * math.sin(t + 1.5)),     # серва 1: 10..70 град
            int(120 + 20 * math.sin(t * 1.2))     # серва 2: 100..140 град
        ]
        # Ограничиваем диапазон 0..160
        servo_angles = [max(0, min(160, a)) for a in servo_angles]
        
        servo_msg = Int32MultiArray()
        servo_msg.data = servo_angles
        self.servo_pub.publish(servo_msg)
        
        self.get_logger().info(
            f"Published: motors={[round(v,2) for v in motor_speeds]}, servos={servo_angles}" # 
        )
        
        self.counter += 1

def main(args=None):
    rclpy.init(args=args)
    node = TestActuatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()