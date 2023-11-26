import time

import RPi.GPIO as GPIO
import board # Not setting GPIO.setmode(GPIO.BCM) manually because this already does it
import busio
import adafruit_pca9685

# Define pins for motor controls
STBY = 16
AIN1 = 5
AIN2 = 6
BIN1 = 26
BIN2 = 19

OUT_PINS = [STBY, AIN1, AIN2, BIN1, BIN2]

class Motors:

    def __init__(self):
        self._speed = [0,0]
        self._angle = 0
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
        self.servo_range = 1000
        self.servo_step = self.servo_range / float(180)
        self.period = (1000000 / self.pca.frequency)
        # Set it to 0 for good measure
        self.pca.channels[0].duty_cycle = self._speed[0]
        self.pca.channels[1].duty_cycle = self._speed[1]

    def enable(self,state=True): # TODO make this a property
        if state:
            GPIO.output(AIN2, GPIO.HIGH)
            GPIO.output(BIN1, GPIO.HIGH)
        else:
            GPIO.output(AIN2, GPIO.LOW)
            GPIO.output(BIN1, GPIO.LOW)
    
    def deinit(self):
        self.pca.channels[0].duty_cycle = 0
        self.pca.channels[1].duty_cycle = 0
        for pin in OUT_PINS:
            GPIO.output(pin, GPIO.LOW)
        self.pca.deinit()
        GPIO.cleanup()

    def tocka(self):
        self.enable(False)
        self.speed = [0x2222,0x2222]
        GPIO.output(AIN2, GPIO.HIGH)
        GPIO.output(BIN2, GPIO.HIGH)
        time.sleep(2.4)
        self.speed = [0x0000,0x0000]
        self.enable(False)

    @property
    def speed(self):
        return self._speed

    @property
    def angle(self):
        return self._angle

    @speed.setter
    def speed(self, newSpeed):
        self._speed = newSpeed
        self.pca.channels[0].duty_cycle = self._speed[0]
        self.pca.channels[1].duty_cycle = self._speed[1]

    @angle.setter
    def angle(self, newAngle):
        self._angle = newAngle
        if (self._angle < 0):
            self._angle = 0
        elif (self._angle > 180):
            self._angle = 180
        pulseWidth = self._angle * self.servo_step + 1000 # Angle * Degree represented in microseconds + Pulse for 0 degrees
        duty = int((pulseWidth * 100) / float(self.period) * 0xFFFF)
        self.pca.channels[2].duty_cycle = duty