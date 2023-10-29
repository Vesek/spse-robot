import RPi.GPIO as GPIO
import board # Not setting GPIO.setmode(GPIO.BCM) manually because this already does it
import busio
import adafruit_pca9685
import sys
import time

STBY = 13
AIN1 = 6
AIN2 = 5
BIN1 = 19
BIN2 = 26

OUT_PINS = [STBY, AIN1, AIN2, BIN1, BIN2]

for pin in OUT_PINS:
    GPIO.setup(pin, GPIO.OUT)
    GPIO.output(pin, GPIO.LOW)

GPIO.output(STBY, GPIO.HIGH)

i2c = busio.I2C(board.SCL, board.SDA)
pca = adafruit_pca9685.PCA9685(i2c)

pca.frequency = 500

pca.channels[0].duty_cycle = 0
pca.channels[1].duty_cycle = 0

input("Press enter to start motors...")

power = 0x2222

try:
    pca.channels[0].duty_cycle = power
    pca.channels[1].duty_cycle = power
    while True:
        # for i in range(0,0xFFFF):
        #     pca.channels[0].duty_cycle = i
        #     pca.channels[1].duty_cycle = i
        #     time.sleep(0.01)
        GPIO.output(AIN1, GPIO.HIGH)
        GPIO.output(BIN1, GPIO.HIGH)
        time.sleep(2.4)
        GPIO.output(AIN1, GPIO.LOW)
        GPIO.output(BIN1, GPIO.LOW)
        time.sleep(2.4)
        # GPIO.output(AIN1, GPIO.HIGH)
        # GPIO.output(BIN2, GPIO.HIGH)
        # time.sleep(2)
        # GPIO.output(AIN1, GPIO.LOW)
        # GPIO.output(BIN2, GPIO.LOW)
        # time.sleep(2)
except KeyboardInterrupt:
    pca.channels[0].duty_cycle = 0
    pca.channels[1].duty_cycle = 0
    pca.deinit()
    GPIO.cleanup()
    sys.exit(130)
