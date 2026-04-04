import board
import time
import lgpio

from adafruit_pca9685 import PCA9685
from adafruit_motor import servo

def servo_test(pca: PCA9685):
    serv = servo.Servo(pca.channels[6])
    serv.angle = 90
    # for i in range(50, 120):
    #     time.sleep(0.03)
    # for i in range(30, 120):
    #     serv.angle = 120 - i
    #     time.sleep(0.03)

def wheel_test(pca: PCA9685):
    chip = lgpio.gpiochip_open(4)

    # направление
    lgpio.gpio_claim_output(chip, 5)
    lgpio.gpio_claim_output(chip, 6)
    lgpio.gpio_write(chip, 5, 1)
    lgpio.gpio_write(chip, 6, 0)

    pca.channels[0].duty_cycle = 65535 * 0.5
    time.sleep(5)

    
    lgpio.gpiochip_close(chip)
    pca.deinit()


def main():
    pca = PCA9685(board.I2C(), address=0x40)
    pca.frequency = 50
    
    pca.deinit()

    

if __name__ == "__main__":
    main()