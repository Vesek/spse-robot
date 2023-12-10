import RPi.GPIO as GPIO
import systemd.daemon
import time
import os
from spse_robot import Robot
from src.picam import Camera
from multiprocessing import Process
import signal
import sys

class namespace:
    headless=True
    use_fb=True
    motors=False
    show_raw=False
    show_preprocessed=False
    image=None
    stop_on_line=True
    detect_colors=True
    speed=0x6666
    accel=speed/2
    servo=False
    running_on_rpi=True


def handler(signal, frame):
    sys.exit(0)

signal.signal(signal.SIGINT, handler)
camera = Camera()
robot = Robot(camera, namespace)

GPIO.setmode(GPIO.BCM)
GPIO.setup(21, GPIO.IN, GPIO.PUD_UP)

process = Process(target=robot.main_loop)

def onButton(channel):
    global process
    print("Button pressed!")
    print(process.is_alive())
    if not process.is_alive() and os.system("systemctl --user is-active --quiet spse-robot-audio") != 0:
        print("Starting main loop")
        process.start()
        print("Starting spse-robot-audio")
        os.system("systemctl --user start spse-robot-audio")
    else:
        print("Stopping main loop")
        process.terminate()
        print("Stopping spse-robot-audio")
        os.system("systemctl --user stop spse-robot-audio")

GPIO.add_event_detect(21, GPIO.FALLING, callback=onButton, bouncetime=1000)
print("Initialzed!")
systemd.daemon.notify('READY=1')
while True:
    time.sleep(1)
