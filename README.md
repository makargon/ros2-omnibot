# Omnibot ROS 2 Humble (Docker, Raspberry Pi)

Скелет проекта для ROS 2 Humble в Docker с нодой управления 3-мя моторами омнибота.

## Структура

- `docker/Dockerfile` — образ ROS 2 Humble для Raspberry Pi (arm64).
- `docker-compose.yml` — запуск контейнеров с доступом к GPIO/SPI/I2C и USB.
- `ros2_ws/src/omnibot_control` — ROS 2 пакет управления моторами.
- `ros2_ws/src/omnibot_perception` — ROS 2 пакет для LIDAR и SLAM.
- `ros2_ws/src/omnibot_vision` — ROS 2 пакет для USB-камеры и ArUco-меток.

## Нода motor_controller

- Подписка: `/cmd_vel` (`geometry_msgs/Twist`)
- Публикация: `/wheel_encoder_counts` (`std_msgs/Int32MultiArray`)
- Кинематика: 3-колёсный омнибот (120°)
- PWM: PCA9685 (I2C)
- Энкодеры: через MCP3008 (CH0/1, CH3/4, CH5/6)

## Нода servo_controller

- Управляет сервами на PCA9685 каналах PWM4, PWM5, PWM6
- Подписка: `/servo_angles_deg` (`std_msgs/Float32MultiArray`)
- Формат: `[servo4_deg, servo5_deg, servo6_deg]`
- Также принимает одиночные топики: `/servo4_deg`, `/servo5_deg`, `/servo6_deg` (`std_msgs/Float32`)

## Пакет omnibot_perception — LIDAR и SLAM

Новый пакет для работы с RPLiDAR A1M8 и создания карт робота в реальном времени.

### Узлы perception

1. **rplidar_node** — драйвер лидара (используется стандартный пакет `rplidar_ros`)
   - Публикация: `/scan` (`sensor_msgs/LaserScan`)
   - Подключение: USB (`/dev/ttyUSB0`, по умолчанию)
   - Скорость сканирования: 5.5 Hz (A1M8)
   - Дальность: 0.15-12 м

2. **tf_broadcaster** — трансляция фреймов робота
   - Публикация трансформов: `odom` → `base_link` → `lidar_link`
   - Позиция лидара относительно робота (x, y, z)

3. **slam_toolbox** — SLAM (одновременная локализация и картирование)
   - Принимает: `/scan`, `/odom`
   - Публикует: `/map`, `/map_metadata`
   - Создание и обновление карты в реальном времени
   - Сохранение/загрузка карт

### Запуск perception

**На робота (RPI 5 в Docker):**
```bash
docker compose up omnibot_perception
```

Это запустит лидар, SLAM и Foxglove Bridge на RPI.

**На отдельном устройстве (для визуализации):**

1. Убедитесь, что оба устройства в одной сети
2. Откройте Foxglove Desktop (или web-версию)
3. Подключитесь к WebSocket:
   - `ws://<IP_RPI>:8765`
   - пример: `ws://192.168.1.42:8765`

Foxglove получит топики `/scan`, `/map`, `/tf`, `/odom` через `foxglove_bridge`.

### Конфигурация LIDAR

Параметры в `ros2_ws/src/omnibot_perception/config/lidar.yaml`:
- `port`: `/dev/ttyUSB0` (изменить если лидар на другом порту)
- `baudrate`: `115200` (стандарт RPLiDAR A1M8)
- `frame_id`: `lidar_link`
- `angle_min/max`: полный оборот (−π до +π)
- `range_min/max`: 0.15–12 м

Проверить доступность лидара:
```bash
ls -la /dev/ttyUSB*
```

Если `/dev/ttyUSB0` не виден, может потребоваться USB-адаптер или другой порт.

### Конфигурация SLAM

Параметры SLAM в `config/slam.yaml`:
- `resolution`: 0.05 м (5 см на пиксель карты)
- `do_loop_closing`: `true` (закрытие циклов для более точной карты)
- `loop_search_max_linear_radius`: 3 м
- `scan_matcher_variance`: точность сопоставления сканов

### Сетевое взаимодействие

Для визуализации на удалённом устройстве:

**На RPI в docker-compose.yml:**
```yaml
environment:
  - ROS_DOMAIN_ID=0
  - RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

**На клиентском устройстве (Foxglove):**
- ROS 2 устанавливать не обязательно
- Достаточно открыть `ws://<IP_RPI>:8765`

Убедитесь, что:
1. Оба устройства в одной подсети (например, `192.168.1.0/24`)
2. Порт `8765/TCP` открыт
3. Firewall не блокирует входящие на RPI для `foxglove_bridge`

### Сохранение и загрузка карт

**Сохранить карту:**
```bash
docker exec -it omnibot_perception bash -lc "
source /opt/ros/humble/setup.bash &&
source /workspaces/omnibot/ros2_ws/install/setup.bash &&
ros2 run slam_toolbox serialize_map -f /tmp/my_map
"
```

**Загрузить карту:**
Отредактировать `config/slam.yaml`:
```yaml
map_file_name: '/tmp/my_map'
use_saved_map: true
```

## Назначение пинов

MCP3008:
- CLK -> GPIO11
- DOUT -> GPIO9
- DIN -> GPIO10
- CS# -> GPIO8

PCA9685:
- OE -> GPIO16
- SDA -> GPIO23
- SCL -> GPIO24
- PWM0 -> Motor1 PWM
- PWM1 -> Motor2 PWM
- PWM2 -> Motor3 PWM

Мотор 1:
- PWM -> PCA9685 CH0
- IN1 -> GPIO5
- IN2 -> GPIO6
- Enc A -> CH0
- Enc B -> CH1

Мотор 2:
- PWM -> PCA9685 CH1
- IN1 -> GPIO17
- IN2 -> GPIO22
- Enc A -> CH3
- Enc B -> CH4

Мотор 3:
- PWM -> PCA9685 CH2
- IN1 -> GPIO26
- IN2 -> GPIO27
- Enc A -> CH5
- Enc B -> CH6

## Запуск

1. Включить SPI и I2C в Raspberry Pi OS (`raspi-config`).
2. В корне проекта выполнить:
   - `docker compose up --build`

Контейнер соберёт `colcon` workspace и запустит launch-файл:
- `ros2 launch omnibot_control motor_controller.launch.py`

## Важно

- Используется **PCA9685 по I2C** для PWM и **libgpiod** для IN1/IN2/OE.
- Если GPIO/I2C недоступны, нода стартует в `dry-run` режиме.
- Параметры ноды находятся в `config/motor_controller.yaml`.

### Если есть ошибка с доступом к GPIO/I2C

Если контейнер не может открыть `/dev/gpiochip0` или `/dev/i2c-1`, убедитесь:
1. SPI и I2C включены в `raspi-config`
2. Контейнер имеет доступ к устройствам (опция `privileged: true` в docker-compose.yml)
3. PCA9685 виден на шине (обычно `0x40`):
   - `docker exec -it omnibot_ros2 bash -lc "i2cdetect -y 1"`

Для Raspberry Pi 5 GPIO-банк обычно находится на `/dev/gpiochip4` (RP1), а не на `/dev/gpiochip0`. Если нужно принудительно выбрать chip, задайте `OMNIBOT_GPIOCHIP=/dev/gpiochip4`.

После изменений Dockerfile/docker-compose обязательно пересоберите образ без кэша:
- `docker compose build --no-cache`
- `docker compose up`

### Быстрый тест колёс (по очереди)

- `docker exec -it omnibot_ros2 bash -lc "source /opt/ros/humble/setup.bash && cd /workspaces/omnibot/ros2_ws && colcon build --symlink-install --packages-select omnibot_control && source install/setup.bash && ros2 run omnibot_control wheel_test"`

### Запуск servo_controller

- `docker exec -it omnibot_ros2 bash -lc "source /opt/ros/humble/setup.bash && cd /workspaces/omnibot/ros2_ws && colcon build --symlink-install --packages-select omnibot_control && source install/setup.bash && ros2 launch omnibot_control servo_controller.launch.py"`

Пример команды на сервы:
- `docker exec -it omnibot_ros2 bash -lc "source /opt/ros/humble/setup.bash && source /workspaces/omnibot/ros2_ws/install/setup.bash && ros2 topic pub /servo_angles_deg std_msgs/msg/Float32MultiArray '{data: [90.0, 45.0, 135.0]}' -1"`

Одиночный канал (например PWM4):
- `docker exec -it omnibot_ros2 bash -lc "source /opt/ros/humble/setup.bash && source /workspaces/omnibot/ros2_ws/install/setup.bash && ros2 topic pub /servo4_deg std_msgs/msg/Float32 '{data: 120.0}' -1"`

### Docker Compose сервисы

- `omnibot_ros2` — запускает `motor_controller`
- `omnibot_servo` — запускает `servo_controller`
- `omnibot_perception` — запускает LIDAR, SLAM и `foxglove_bridge`
- `omnibot_vision` — запускает USB-камеру, fisheye-коррекцию и ArUco-детектор

Запуск всех сервисов:
- `docker compose up -d`

Запуск только perception:
- `docker compose up omnibot_perception`

### Ручное управление с клавиатуры

Добавлен скрипт `manual_control`:
- WASD — линейное движение (`/cmd_vel`)
- Q/E — разворот (`/cmd_vel`)
- I/K — servo PWM4 +/−
- J/L — servo PWM5 +/−
- U/O — servo PWM6 +/−
- Space или X — стоп движения

Запуск:
- `docker exec -it omnibot_ros2 bash -lc "source /opt/ros/humble/setup.bash && cd /workspaces/omnibot/ros2_ws && colcon build --symlink-install --packages-select omnibot_control && source install/setup.bash && ros2 run omnibot_control manual_control"`

### Калибровка кинематики

Если `W/S` крутит робота, а `A/D` едет вперёд/назад:
1. Проверьте `motor3.direction_sign` в [ros2_ws/src/omnibot_control/config/motor_controller.yaml](ros2_ws/src/omnibot_control/config/motor_controller.yaml)
2. Если оси всё ещё перепутаны, выставьте `cmd_frame_rotation_deg: 90.0` (или `-90.0`) в том же файле

После изменения параметров перезапустите сервис:
- `docker compose restart omnibot_ros2`

## Пакет omnibot_vision — USB-камера и ArUco

Пакет предназначен для второй RPI с USB-камерой. Он состоит из двух нод:

1. **camera_node** — захватывает кадры с камеры и публикует `image_raw`, `image_rect` и `camera_info`
2. **aruco_pose_node** — ищет ArUco-метки, вычисляет позы, переводит координаты в мировой фрейм и публикует TF

### Что публикуется

- `/camera/image_raw` — сырой кадр
- `/camera/image_rect` — кадр после fisheye-коррекции
- `/camera/camera_info` — матрица камеры и коэффициенты дисторсии
- `/aruco/detections` — позы всех видимых меток в фрейме камеры
- `/aruco/detection_ids` — список ID найденных меток
- `/aruco/field_markers` — позы меток в мировом фрейме
- `/aruco/field_marker_ids` — ID меток, попавших в `field_markers`
- `/aruco/self_robot_pose` — поза своей метки робота в мире
- `/aruco/opponent_robot_pose` — поза метки противника в мире

### Что публикуется в TF

- `map -> camera_link` (или `camera_tf_frame`)
- `map -> robot_self` (если видна своя метка)
- `map -> robot_opponent` (если видна метка противника)
- `map -> aruco_<ID>` для видимых меток (если `publish_marker_tfs: true`)

### Конфигурация

Файлы находятся в `ros2_ws/src/omnibot_vision/config`:

- `camera.yaml` — параметры камеры
- `calibration.yaml` — пример калибровки fisheye
- `aruco.yaml` — ID и длина меток
- `field_map_example.yaml` — пример карты поля на 4 ArUco

### Запуск на второй RPI

```bash
docker compose up omnibot_vision
```

Если нужно использовать другую камеру или другой calib-файл:

```bash
ros2 launch omnibot_vision vision.launch.py device:=1 calibration_file:=/path/to/calibration.yaml
```

Также можно передать свою карту поля:

```bash
ros2 launch omnibot_vision vision.launch.py field_map_file:=/path/to/field_map.yaml
```

### Калибровка камеры (инструкция)

1. Распечатайте шахматную доску (обычно 9x6 внутренних углов) и измерьте размер клетки в метрах.
2. Соберите пакет `omnibot_vision`.
3. Снимите 20–30 кадров с разных углов:

```bash
ros2 run omnibot_vision camera_calibration_capture --device 0 --output_dir /tmp/calib_images
```

4. Запустите калибровку:

```bash
ros2 run omnibot_vision fisheye_calibrate --input_dir /tmp/calib_images --output /tmp/calibration.yaml --board_cols 9 --board_rows 6 --square_size 0.025
```

5. Подставьте результат в launch:

```bash
ros2 launch omnibot_vision vision.launch.py calibration_file:=/tmp/calibration.yaml
```

Если RMS-ошибка большая, повторите шаги 3–4 с более разнообразными ракурсами.

### Пример карты поля (4 ArUco)

Готовый пример: `ros2_ws/src/omnibot_vision/config/field_map_example.yaml`

Формат:
- `frame_id`: мировой фрейм (обычно `map`)
- `markers[].id`: ID ArUco
- `markers[].pose`: `[x, y, z, roll, pitch, yaw]`, где углы в радианах

### Как это работает

- Нода читает карту поля (`marker_map_file`) и выбирает видимую метку-якорь из этой карты
- По якорю вычисляется преобразование `camera -> map`
- Все обнаруженные метки пересчитываются в мировой фрейм
- Поза своей и чужой метки публикуется и в топики, и в TF
