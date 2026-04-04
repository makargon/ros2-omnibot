import board
from adafruit_blinka.microcontroller.rp2040_u2if import pin
from adafruit_motor import motor



def main(): 
    GP0 = pin.GP0
    GP0.init(mode=pin.Pin.mode.OUT)
    GP0.value = 1

    pass
    # a = 
    # b = 
    # motor1 = motor.DCMotor()

if __name__ == "main":
    main()