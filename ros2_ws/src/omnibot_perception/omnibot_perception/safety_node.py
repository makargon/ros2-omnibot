import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from tf2_ros import TransformListener, Buffer
import math

class FullSafetyStopNode(Node):
    """
    нода безопасности, которая останавливает робота при препятствиях:
    - спереди, если робот хочет ехать вперёд
    - сзади, если робот хочет ехать назад
    """
    
    def __init__(self):
        super().__init__('full_safety_stop_node')
        
        # === параметры (настройки которые можно менять) ===
        self.declare_parameter('danger_distance', 0.1)      # опасная дистанция (метры)
        self.declare_parameter('warning_distance', 0.3)     # дистанция предупреждения
        self.declare_parameter('front_angle', 30.0)         # угол обзора спереди (градусы)
        self.declare_parameter('rear_angle', 30.0)          # угол обзора сзади (градусы)
        self.declare_parameter('robot_radius', 0.2)         # радиус робота (добавка безопасности)
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('lidar_frame', 'lidar_link')
        self.declare_parameter('enable_logging', True)
        
        self.danger_distance = self.get_parameter('danger_distance').value
        self.warning_distance = self.get_parameter('warning_distance').value
        self.front_angle = math.radians(self.get_parameter('front_angle').value)
        self.rear_angle = math.radians(self.get_parameter('rear_angle').value)
        self.robot_radius = self.get_parameter('robot_radius').value
        self.base_frame = self.get_parameter('base_frame').value
        self.lidar_frame = self.get_parameter('lidar_frame').value
        self.enable_logging = self.get_parameter('enable_logging').value
        
        # === tf (чтобы знать где лидар относительно робота) ===
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # === данные ===
        self.latest_scan = None
        self.latest_cmd_vel = None
        self.lidar_transform = None  # положение лидара относительно центра робота
        
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
        
        # === публикатор ===
        self.safe_cmd_publisher = self.create_publisher(
            Twist,
            'cmd_vel_safe',
            10
        )
        
        # === таймеры ===
        self.create_timer(0.1, self.safety_check)           # проверка 10 раз в секунду
        self.create_timer(1.0, self.update_lidar_transform) # обновление позиции лидара раз в секунду
        
        self.get_logger().info('=== полная safety нода запущена ===')
        self.get_logger().info(f'опасная дистанция: {self.danger_distance}м')
        self.get_logger().info(f'передний угол: {math.degrees(self.front_angle):.0f}°')
        self.get_logger().info(f'задний угол: {math.degrees(self.rear_angle):.0f}°')
    
    def update_lidar_transform(self):
        """получает текущее положение лидара относительно центра робота"""
        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.lidar_frame,
                rclpy.time.Time()
            )
            self.lidar_transform = transform
            if self.enable_logging:
                self.get_logger().debug(
                    f'лидар находится: x={transform.transform.translation.x:.2f}, '
                    f'y={transform.transform.translation.y:.2f}'
                )
        except Exception as e:
            if self.enable_logging:
                self.get_logger().warn(f'не удалось получить tf лидара: {e}')
            # если tf нет, считаем что лидар в центре робота
            if self.lidar_transform is None:
                from geometry_msgs.msg import TransformStamped
                default_tf = TransformStamped()
                default_tf.transform.translation.x = 0.0
                default_tf.transform.translation.y = 0.0
                default_tf.transform.translation.z = 0.45
                self.lidar_transform = default_tf
    
    def scan_callback(self, msg):
        """сохраняет последние данные с лидара"""
        self.latest_scan = msg
    
    def cmd_vel_callback(self, msg):
        """сохраняет последнюю команду скорости и сразу проверяет"""
        self.latest_cmd_vel = msg
        self.safety_check()  # проверяем сразу при новой команде
    
    def transform_point_to_robot_frame(self, angle, distance):
        """
        преобразует точку из системы координат лидара в систему робота
        
        аргументы:
            angle: угол в системе лидара (радианы)
            distance: расстояние от лидара до точки (метры)
        
        возвращает:
            x, y: координаты точки относительно центра робота
        """
        if self.lidar_transform is None:
            # нет данных tf - считаем что лидар в центре
            x = distance * math.cos(angle)
            y = distance * math.sin(angle)
            return x, y
        
        # позиция точки в системе лидара
        lidar_x = distance * math.cos(angle)
        lidar_y = distance * math.sin(angle)
        
        # смещение лидара относительно центра робота
        lidar_offset_x = self.lidar_transform.transform.translation.x
        lidar_offset_y = self.lidar_transform.transform.translation.y
        
        # поворот лидара относительно робота (если есть)
        q = self.lidar_transform.transform.rotation
        # конвертируем кватернион в угол рыскания (yaw)
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        lidar_yaw = math.atan2(siny_cosp, cosy_cosp)
        
        # поворачиваем точку из системы лидара в систему робота
        rotated_x = lidar_x * math.cos(lidar_yaw) - lidar_y * math.sin(lidar_yaw)
        rotated_y = lidar_x * math.sin(lidar_yaw) + lidar_y * math.cos(lidar_yaw)
        
        # добавляем смещение лидара
        robot_x = rotated_x + lidar_offset_x
        robot_y = rotated_y + lidar_offset_y
        
        return robot_x, robot_y
    
    def get_obstacle_info(self):
        """
        анализирует препятствия спереди и сзади робота
        
        возвращает:
            front_distance: минимальное расстояние до препятствия спереди (метры) или None
            rear_distance: минимальное расстояние до препятствия сзади (метры) или None
            front_angle: угол до ближайшего препятствия спереди
            rear_angle: угол до ближайшего препятствия сзади
        """
        if self.latest_scan is None:
            return None, None, None, None
        
        angle_min = self.latest_scan.angle_min
        angle_increment = self.latest_scan.angle_increment
        
        front_min_distance = float('inf')
        rear_min_distance = float('inf')
        front_min_angle = 0.0
        rear_min_angle = 0.0
        
        # проходим по всем точкам лидара
        for i, distance in enumerate(self.latest_scan.ranges):
            # пропускаем точки без данных
            if math.isinf(distance) or math.isnan(distance):
                continue
            
            # угол в системе лидара
            angle_lidar = angle_min + i * angle_increment
            
            # преобразуем точку в координаты робота
            robot_x, robot_y = self.transform_point_to_robot_frame(angle_lidar, distance)
            
            # вычисляем угол точки относительно центра робота
            angle_robot = math.atan2(robot_y, robot_x)
            
            # нормализуем угол в диапазон [-π, π]
            if angle_robot > math.pi:
                angle_robot -= 2 * math.pi
            elif angle_robot < -math.pi:
                angle_robot += 2 * math.pi
            
            # расстояние от центра робота до точки
            distance_robot = math.sqrt(robot_x**2 + robot_y**2)
            
            # вычитаем радиус робота для более точной оценки
            effective_distance = distance_robot - self.robot_radius
            
            # === проверяем препятствия спереди (угол около 0) ===
            if abs(angle_robot) <= self.front_angle:
                if effective_distance < front_min_distance:
                    front_min_distance = effective_distance
                    front_min_angle = math.degrees(angle_robot)
            
            # === проверяем препятствия сзади (угол около π или -π) ===
            # задний сектор: углы близкие к π (180°) или -π (-180°)
            rear_angle_diff = min(abs(angle_robot - math.pi), abs(angle_robot + math.pi))
            if rear_angle_diff <= self.rear_angle:
                if effective_distance < rear_min_distance:
                    rear_min_distance = effective_distance
                    rear_min_angle = math.degrees(angle_robot)
                    # нормализуем отображение угла для зада
                    if rear_min_angle > 90:
                        rear_min_angle = 180 - rear_min_angle
                    elif rear_min_angle < -90:
                        rear_min_angle = -180 - rear_min_angle
        
        # преобразуем бесконечность в None
        front_distance = front_min_distance if front_min_distance != float('inf') else None
        rear_distance = rear_min_distance if rear_min_distance != float('inf') else None
        
        return front_distance, rear_distance, front_min_angle, rear_min_angle
    
    def get_robot_motion_intent(self):
        """
        определяет, куда хочет ехать робот
        
        возвращает:
            'forward': хочет ехать вперёд
            'backward': хочет ехать назад
            'stopped': стоит на месте
        """
        if self.latest_cmd_vel is None:
            return 'stopped'
        
        forward_speed = self.latest_cmd_vel.linear.x
        
        if forward_speed > 0.01:
            return 'forward'
        elif forward_speed < -0.01:
            return 'backward'
        else:
            return 'stopped'
    
    def safety_check(self):
        """главная логика безопасности - проверяет препятствия и останавливает при необходимости"""
        
        # проверяем наличие данных
        if self.latest_scan is None:
            if self.enable_logging:
                self.get_logger().warn('нет данных с лидара')
            return
        
        if self.latest_cmd_vel is None:
            return
        
        # получаем информацию о препятствиях
        front_distance, rear_distance, front_angle, rear_angle = self.get_obstacle_info()
        
        # определяем намерение робота
        motion_intent = self.get_robot_motion_intent()
        
        # флаг остановки и причина
        should_stop = False
        stop_reason = ""
        
        # === проверка препятствий спереди (если робот хочет ехать вперёд) ===
        if motion_intent == 'forward' and front_distance is not None:
            if front_distance < self.danger_distance:
                should_stop = True
                stop_reason = f"препятствие спереди на {front_distance:.2f}м (угол {front_angle:.0f}°)"
            elif front_distance < self.warning_distance and self.enable_logging:
                self.get_logger().warn(
                    f"предупреждение: препятствие спереди на {front_distance:.2f}м"
                )
        
        # === проверка препятствий сзади (если робот хочет ехать назад) ===
        elif motion_intent == 'backward' and rear_distance is not None:
            if rear_distance < self.danger_distance:
                should_stop = True
                stop_reason = f"препятствие сзади на {rear_distance:.2f}м (угол {rear_angle:.0f}°)"
            elif rear_distance < self.warning_distance and self.enable_logging:
                self.get_logger().warn(
                    f"предупреждение: препятствие сзади на {rear_distance:.2f}м"
                )
        
        # === логирование текущего состояния ===
        if self.enable_logging:
            front_info = f"{front_distance:.2f}" if front_distance else "нет"
            rear_info = f"{rear_distance:.2f}" if rear_distance else "нет"
            speed = self.latest_cmd_vel.linear.x
            
            status = "🚨 остановка" if should_stop else "✅ безопасно"
            self.get_logger().info(
                f'[безопасность] спереди: {front_info}м | '
                f'сзади: {rear_info}м | '
                f'скорость: {speed:.2f} | '
                f'намерение: {motion_intent} | '
                f'{status}'
            )
        
        # === применяем решение ===
        if should_stop:
            self.get_logger().error(f'🚨 АВАРИЙНАЯ ОСТАНОВКА! {stop_reason}')
            self.publish_stop_command()
        else:
            self.publish_safe_command()
    
    def publish_stop_command(self):
        """публикует команду полной остановки"""
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.linear.y = 0.0
        stop_msg.linear.z = 0.0
        stop_msg.angular.x = 0.0
        stop_msg.angular.y = 0.0
        stop_msg.angular.z = 0.0
        self.safe_cmd_publisher.publish(stop_msg)
    
    def publish_safe_command(self):
        """публикует безопасную команду (с уменьшением скорости при приближении)"""
        if self.latest_cmd_vel is None:
            return
        
        # получаем расстояния до препятствий
        front_distance, rear_distance, _, _ = self.get_obstacle_info()
        motion_intent = self.get_robot_motion_intent()
        
        # определяем, с какой стороны нужно снизить скорость
        relevant_distance = None
        if motion_intent == 'forward' and front_distance is not None:
            relevant_distance = front_distance
        elif motion_intent == 'backward' and rear_distance is not None:
            relevant_distance = rear_distance
        
        # если препятствие близко - плавно уменьшаем скорость
        if relevant_distance is not None and relevant_distance < self.warning_distance:
            safe_cmd = Twist()
            # скорость уменьшается пропорционально расстоянию
            speed_factor = max(0.1, (relevant_distance - self.robot_radius) / self.warning_distance)
            safe_cmd.linear.x = self.latest_cmd_vel.linear.x * speed_factor
            safe_cmd.angular.z = self.latest_cmd_vel.angular.z
            self.safe_cmd_publisher.publish(safe_cmd)
        else:
            # отправляем оригинальную команду
            self.safe_cmd_publisher.publish(self.latest_cmd_vel)
    
    def destroy_node(self):
        """очистка при остановке ноды"""
        self.publish_stop_command()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = FullSafetyStopNode()
    
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