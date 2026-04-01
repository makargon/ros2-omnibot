# Omnibot ROS 2 Humble (Docker, Raspberry Pi)

Скелет проекта для ROS 2 Humble в Docker с нодой управления 3-мя моторами омнибота.

## Структура

- `docker/Dockerfile` — образ ROS 2 Humble для Raspberry Pi (arm64).
- `docker-compose.yml` — запуск контейнеров с доступом к GPIO/SPI/I2C.
- `ros2_ws/src/omnibot_control` — ROS 2 пакет (`ament_python`) с нодой `motor_controller`.

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

Запуск обоих:
- `docker compose up -d omnibot_ros2 omnibot_servo`

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
