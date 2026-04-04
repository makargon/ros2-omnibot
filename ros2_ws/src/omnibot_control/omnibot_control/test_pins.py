# import board
# from adafruit_blinka.microcontroller.rp2040_u2if import pin
# from adafruit_motor import motor
import lgpio
import time

PIN = 0

# def main():
h = lgpio.gpiod_chip(4)
lgpio.gpio_claim_output(h, PIN)
# try:
while True:
    lgpio.gpio_write(h, PIN, 1)
    time.sleep(10)
    lgpio.gpio_write(h, PIN, 0)
    time.sleep(10)
# except KeyboardInterrupt:
#     lgpio.gpio_write(h, PIN, 0)
#     lgpio.gpiochip_close(h)

# if __name__ == "main":
#     main()