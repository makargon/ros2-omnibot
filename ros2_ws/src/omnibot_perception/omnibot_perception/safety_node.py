import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math

class SimpleSafetyStopNode(Node):
    """
    простая safety нода:
    - берёт вектор скорости (куда хочет ехать робот)
    - проверяет препятствия в направлении движения
    - останавливает если препятствие слишком близко
    """
    
    def __init__(self):
        super().__init__('simple_safety_stop_node')
        
        # === параметры ===
        self.declare_parameter('danger_distance', 0.1)    # опасная дистанция (метры)
        self.declare_parameter('warning_distance', 0.3)   # дистанция предупреждения
        self.declare_parameter('safety_angle', 30.0)      # угол обзора (градусы)
        self.declare_parameter('enable_logging', True)
        
        self.danger_distance = self.get_parameter('danger_distance').value
        self.warning_distance = self.get_parameter('warning_distance').value
        self.safety_angle = math.radians(self.get_parameter('safety_angle').value)
        self.enable_logging = self.get_parameter('enable_logging').value
        
        # === данные ===
        self.latest_scan = None
        self.latest_cmd_vel = None
        
        # === подписчики ===
        self.scan_subscriber = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            10
        )
        
        self.cmd_vel_subscriber = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # === публикатор (безопасная команда) ===
        self.safe_cmd_publisher = self.create_publisher(
            Twist,
            'cmd_vel_safe',
            10
        )
        
        # === таймер для периодической проверки ===
        self.create_timer(0.1, self.safety_check)
        
        self.get_logger().info('простая safety нода запущена')
        self.get_logger().info(f'опасная дистанция: {self.danger_distance}м')
        self.get_logger().info(f'угол обзора: {math.degrees(self.safety_angle):.0f}°')
    
    def scan_callback(self, msg):
        """сохраняет данные с лидара"""
        self.latest_scan = msg
    
    def cmd_vel_callback(self, msg):
        """сохраняет команду скорости"""
        self.latest_cmd_vel = msg
        self.safety_check()  # сразу проверяем при новой команде
    
    def get_min_distance_in_direction(self, direction_angle):
        """
        ищет минимальное расстояние до препятствия в заданном направлении
        
        аргументы:
            direction_angle: угол направления движения (радианы)
                            0 = вперёд, π = назад, π/2 = вправо и т.д.
        
        возвращает:
            минимальное расстояние в метрах или None если нет препятствий
        """
        if self.latest_scan is None:
            return None
        
        angle_min = self.latest_scan.angle_min
        angle_increment = self.latest_scan.angle_increment
        
        min_distance = float('inf')
        
        for i, distance in enumerate(self.latest_scan.ranges):
            # пропускаем точки без данных
            if math.isinf(distance) or math.isnan(distance):
                continue
            
            # угол текущей точки в системе лидара (и робота, т.к. данные уже трансформированы)
            point_angle = angle_min + i * angle_increment
            
            # нормализуем угол в диапазон [-π, π]
            if point_angle > math.pi:
                point_angle -= 2 * math.pi
            elif point_angle < -math.pi:
                point_angle += 2 * math.pi
            
            # считаем разницу между направлением движения и направлением на точку
            angle_diff = abs(point_angle - direction_angle)
            
            # нормализуем разницу (угол не может быть больше π)
            if angle_diff > math.pi:
                angle_diff = 2 * math.pi - angle_diff
            
            # проверяем, попадает ли точка в сектор обзора
            if angle_diff <= self.safety_angle:
                if distance < min_distance:
                    min_distance = distance
        
        if min_distance == float('inf'):
            return None
        
        return min_distance
    
    def get_movement_direction(self):
        """
        определяет направление движения из вектора скорости
        
        возвращает:
            угол в радианах
            или None если робот стоит на месте
        """
        if self.latest_cmd_vel is None:
            return None
        
        linear_x = self.latest_cmd_vel.linear.x
        linear_y = self.latest_cmd_vel.linear.y
        
        # если скорость почти нулевая - робот стоит
        speed = math.sqrt(linear_x**2 + linear_y**2)
        if speed < 0.01:
            return None
        
        # вычисляем угол направления движения
        direction = math.atan2(linear_y, linear_x)
        
        return direction
    
    def safety_check(self):
        """главная логика проверки безопасности"""
        
        # проверяем наличие данных
        if self.latest_scan is None:
            if self.enable_logging:
                self.get_logger().warn('нет данных с лидара')
            return
        
        if self.latest_cmd_vel is None:
            return
        
        # определяем куда хочет ехать робот
        direction = self.get_movement_direction()
        
        # если робот стоит - ничего не делаем
        if direction is None:
            self.publish_safe_command()
            return
        
        # получаем расстояние до препятствия в направлении движения
        distance = self.get_min_distance_in_direction(direction)
        
        # если нет препятствий в этом направлении
        if distance is None:
            if self.enable_logging:
                speed = math.sqrt(
                    self.latest_cmd_vel.linear.x**2 + 
                    self.latest_cmd_vel.linear.y**2
                )
                self.get_logger().info(
                    f'безопасно: скорость {speed:.2f}м/с, '
                    f'препятствий нет'
                )
            self.publish_safe_command()
            return
        
        # преобразуем угол направления в градусы для вывода
        direction_deg = math.degrees(direction)
        
        # === логика остановки ===
        if distance < self.danger_distance:
            # опасное препятствие - останавливаем
            self.get_logger().error(
                f'🚨 АВАРИЙНАЯ ОСТАНОВКА! '
                f'препятствие на {distance:.2f}м '
                f'в направлении {direction_deg:.0f}°'
            )
            self.publish_stop_command()
            
        elif distance < self.warning_distance:
            # предупреждение, но не останавливаем
            if self.enable_logging:
                self.get_logger().warn(
                    f'⚠️ предупреждение: препятствие на {distance:.2f}м '
                    f'в направлении {direction_deg:.0f}°'
                )
            self.publish_safe_command()
        else:
            # безопасно
            if self.enable_logging:
                speed = math.sqrt(
                    self.latest_cmd_vel.linear.x**2 + 
                    self.latest_cmd_vel.linear.y**2
                )
                self.get_logger().info(
                    f'безопасно: скорость {speed:.2f}м/с, '
                    f'ближайшее препятствие {distance:.2f}м'
                )
            self.publish_safe_command()
    
    def publish_stop_command(self):
        """публикует команду полной остановки"""
        stop_msg = Twist()
        # все поля уже 0.0 по умолчанию
        self.safe_cmd_publisher.publish(stop_msg)
    
    def publish_safe_command(self):
        """публикует оригинальную команду скорости"""
        if self.latest_cmd_vel is not None:
            self.safe_cmd_publisher.publish(self.latest_cmd_vel)
    
    def destroy_node(self):
        """останавливает робота при выключении ноды"""
        self.publish_stop_command()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SimpleSafetyStopNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('выключение safety ноды...')
        node.publish_stop_command()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()