import board
import time

from adafruit_pca9685 import PCA9685
from adafruit_motor import servo

def main():
    pca = PCA9685(board.I2C(), address=0x40)
    pca.frequency = 50
    serv = servo.Servo(pca.channels[4])
    for i in range(30, 120):
        serv.angle = i
        time.sleep(0.03)
    for i in range(30, 120):
        serv.angle = 120 - i
        time.sleep(0.03)
    
    pca.deinit()

    

if __name__ == "__main__":
    main()