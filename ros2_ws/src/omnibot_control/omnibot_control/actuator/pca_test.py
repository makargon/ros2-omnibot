import board
import time
from adafruit_pca9685 import PCA9685

def main():
    pca = PCA9685(board.I2C(), address=0x40)
    pca.frequency = 50
    while True:
        pca.channels[4].duty_cycle = 0x7FFF
        time.sleep(1)
        pca.channels[4].duty_cycle = 0
        time.sleep(1)

    

if __name__ == "__main__":
    main()