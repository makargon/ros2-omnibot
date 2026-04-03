#!/usr/bin/env python3
import time
import errno
from dataclasses import dataclass
from typing import Dict, List

# ---------- Проверка и импорт gpiod ----------
try:
    import gpiod  # type: ignore
except ImportError:
    gpiod = None

import board
from adafruit_pca9685 import PCA9685


@dataclass
class Motor:
    name: str
    pwm_channel: int
    in1_gpio: int
    in2_gpio: int


# Настройки моторов
MOTORS: List[Motor] = [
    Motor('motor1', 0, 5, 6),
    Motor('motor2', 1, 17, 22),
    Motor('motor3', 2, 26, 27),
]

OE_GPIO = 16          # active-low, 0 – включить выходы PCA9685


def set_pwm(pca: PCA9685, channel: int, value: float) -> None:
    """
    Установить ШИМ на канале PCA9685.
    value: 0.0 (выкл) .. 1.0 (полная мощность)
    """
    # duty_cycle принимает значения от 0 до 65535
    duty = int(max(0.0, min(1.0, value)) * 65535)
    pca.channels[channel].duty_cycle = duty


def main() -> None:
    # Проверка наличия gpiod
    if gpiod is None:
        raise ImportError(
            "Библиотека gpiod не установлена.\n"
            "Установите: sudo apt install python3-libgpiod"
        )

    # Инициализация I2C и PCA9685
    try:
        i2c = board.I2C()
    except Exception as e:
        raise RuntimeError(f"Не удалось инициализировать I2C. Проверьте подключение и включите I2C.\nОшибка: {e}")

    pca = PCA9685(i2c, address=0x40)
    pca.frequency = 1000          # 1 кГц – подходит для большинства драйверов моторов

    # Определяем правильный gpiochip (обычно gpiochip0 на Raspberry Pi)
    # Можно попробовать несколько вариантов
    chip_path = 'gpiochip0'
    try:
        chip = gpiod.Chip(chip_path)
    except Exception:
        chip_path = 'gpiochip4'
        try:
            chip = gpiod.Chip(chip_path)
        except Exception as e:
            pca.deinit()
            raise RuntimeError(f"Не найден gpiochip. Попробуйте 'gpiochip0' или 'gpiochip4'.\nОшибка: {e}")

    lines: Dict[int, object] = {}

    def request(pin: int):
        """Запросить GPIO-линию как выход"""
        if pin in lines:
            return
        line = chip.get_line(pin)
        try:
            line.request(consumer='wheel_test', type=gpiod.LINE_REQ_DIR_OUT)
        except OSError as e:
            if e.errno == errno.EBUSY:
                raise RuntimeError(
                    f'GPIO{pin} занят. Остановите motor_controller (он использует те же линии), затем запустите wheel_test.'
                ) from e
            raise
        line.set_value(0)
        lines[pin] = line

    def write(pin: int, value: int):
        """Установить значение GPIO (0 или 1)"""
        if pin in lines:
            lines[pin].set_value(1 if value else 0)

    try:
        # Запрашиваем все нужные GPIO
        for m in MOTORS:
            request(m.in1_gpio)
            request(m.in2_gpio)
        request(OE_GPIO)          # пин разрешения PCA9685

        # Включаем выходы PCA9685 (OE active-low)
        write(OE_GPIO, 0)

        def stop_all():
            """Остановить все моторы (GPIO=0, ШИМ=0)"""
            for m in MOTORS:
                write(m.in1_gpio, 0)
                write(m.in2_gpio, 0)
                set_pwm(pca, m.pwm_channel, 0.0)

        stop_all()
        print('Тест колёс: каждое колесо вперёд 2с, стоп 1с, назад 2с, стоп 1с')

        # Основной цикл тестирования
        for m in MOTORS:
            print(f'\n=== {m.name} ===')

            print('вперёд')
            write(m.in1_gpio, 1)
            write(m.in2_gpio, 0)
            set_pwm(pca, m.pwm_channel, 0.6)
            time.sleep(2.0)

            stop_all()
            time.sleep(1.0)

            print('назад')
            write(m.in1_gpio, 0)
            write(m.in2_gpio, 1)
            set_pwm(pca, m.pwm_channel, 0.6)
            time.sleep(2.0)

            stop_all()
            time.sleep(1.0)

        print('\nТест колёс успешно завершён')

    except KeyboardInterrupt:
        print('\nПрерывание пользователем')
    finally:
        # Выключение всего оборудования
        stop_all()
        # Отключаем выходы PCA9685 (OE = 1)
        try:
            write(OE_GPIO, 1)
        except Exception:
            pass
        # Освобождаем PCA9685
        pca.deinit()
        # Освобождаем GPIO
        for line in lines.values():
            try:
                line.set_value(0)
                line.release()
            except Exception:
                pass
        chip.close()
        print("Ресурсы освобождены, тест завершён.")


if __name__ == '__main__':
    main()