import RPi.GPIO as GPIO
import board # Not setting GPIO.setmode(GPIO.BCM) manually because this already does it
import busio
import adafruit_pca9685

# Define pins for motor controls
STBY = 13
AIN1 = 6
AIN2 = 5
BIN1 = 19
BIN2 = 26

OUT_PINS = [STBY, AIN1, AIN2, BIN1, BIN2]

class Motors:

    def __init__(self):
        self._speed = [0,0]
        # Init all the pins
        for pin in OUT_PINS:
            GPIO.setup(pin, GPIO.OUT)
            GPIO.output(pin, GPIO.LOW)
        GPIO.output(STBY, GPIO.HIGH)
        # Init the PCA9685 I2C PWM board
        i2c = busio.I2C(board.SCL, board.SDA)
        self.pca = adafruit_pca9685.PCA9685(i2c)
        # Set the frequency for it
        self.pca.frequency = 500
        # Set it to 0 for good measure
        self.pca.channels[0].duty_cycle = self._speed[0]
        self.pca.channels[1].duty_cycle = self._speed[1]

    def enable(self,state=True):
        if state:
            GPIO.output(AIN1, GPIO.HIGH)
            GPIO.output(BIN1, GPIO.HIGH)
        else:
            GPIO.output(AIN1, GPIO.LOW)
            GPIO.output(BIN1, GPIO.LOW)
    
    def deinit(self):
        self.pca.channels[0].duty_cycle = 0
        self.pca.channels[1].duty_cycle = 0
        for pin in OUT_PINS:
            GPIO.output(pin, GPIO.LOW)
        self.pca.deinit()
        GPIO.cleanup()

    @property
    def speed(self):
        return self._speed

    @speed.setter
    def speed(self, newSpeed):
        self._speed = newSpeed
        self.pca.channels[0].duty_cycle = self._speed[0]
        self.pca.channels[1].duty_cycle = self._speed[1]
