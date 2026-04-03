from enum import IntEnum

import board
from adafruit_pca9685 import PCA9685
from adafruit_motor.servo import Servo

import time
import errno
from dataclasses import dataclass
from typing import Dict, List

# Пины
class ServoIndex(IntEnum):
	HAND = 4
	ROTATE = 5
	GRAB = 6

def test_servos(servos: Dict[str, Servo], angles: List[int] = [0, 90, 160]) -> None:
    print("Начинаем проверку сервоприводов...")
    for name, servo in servos.items():
        # print(f"\nТестируем сервопривод: {name} (канал {servo._channel.channel})")
        for angle in angles:
            print(f"  Устанавливаем угол {angle}°")
            servo.angle = angle
            time.sleep(1.0)
        servo.angle = 90
        time.sleep(0.5)
    print("\nПроверка завершена.")

def main() -> None:
    i2c = board.I2C()
    pca = PCA9685(i2c, address=0x40)
    pca.frequency = 50
    servos = {
        "HAND": Servo(pca.channels[ServoIndex.HAND], min_pulse=0.75, max_pulse=2.25),
        "ROTATE": Servo(pca.channels[ServoIndex.ROTATE], min_pulse=0.75, max_pulse=2.25),
        "GRAB": Servo(pca.channels[ServoIndex.GRAB], min_pulse=0.75, max_pulse=2.25),
    }
    
    try:
        # Выключаем все сервы (ставим angle=None — отключаем сигнал)
        for s in servos.values():
            s.angle = None
        time.sleep(0.5)
        
        # Запускаем проверку
        test_servos(servos)
        
        # После проверки отключаем сигналы
        for s in servos.values():
            s.angle = None
        print("Сервы отключены.")
        
    except KeyboardInterrupt:
        print("\nПрерывание пользователем.")
    finally:
        pca.deinit()  # Освобождаем I2C и выключаем ШИМ
        print("PCA9685 выключен.")

if __name__ == '__main__':
    main()