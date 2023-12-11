import RPi.GPIO as GPIO
import systemd.daemon
import os
from spse_robot import Robot
from src.picam import Camera
from src.threadcamera import ThreadCamera
from multiprocessing import Process
from multiprocessing import shared_memory
import signal
import sys
import math
import numpy as np

os.environ["LIBCAMERA_LOG_LEVELS"] = "3"

class namespace:
    headless=True
    use_fb=True
    motors=True
    show_raw=False
    show_preprocessed=False
    image=None
    stop_on_line=True
    detect_colors=True
    speed=0x6666
    accel=speed/2
    servo=False
    verbose=False
    running_on_rpi=True
    fb_size=(1920, 1080)

real_camera = Camera()
frame = real_camera.capture()
shm = shared_memory.SharedMemory(create=True, size=frame.nbytes)
buffer = np.ndarray(frame.shape, dtype=frame.dtype, buffer=shm.buf)

camera = ThreadCamera(shm.name, frame.shape, frame.dtype)
robot = Robot(camera, namespace)

GPIO.setmode(GPIO.BCM)
GPIO.setup(21, GPIO.IN, GPIO.PUD_UP)

process = None

pressed = False

def signalHandler(signal, frame):
    try:
        print("Stopping main loop")
        process.terminate()
        process.join()
    except AttributeError:
        pass
    print("Stopping spse-robot-audio")
    os.system("systemctl --user stop spse-robot-audio")
    shm.close()
    shm.unlink()
    sys.exit(0)

def onButton(channel):
    global pressed
    print("Button pressed!")
    pressed = True

signal.signal(signal.SIGINT, signalHandler)
GPIO.add_event_detect(21, GPIO.FALLING, callback=onButton, bouncetime=1000)
print("Initialzed!")
systemd.daemon.notify('READY=1')
while True:
    frame = real_camera.capture()
    buffer[:] = frame[:]
    if pressed:
        pressed = False
        if process is None and os.system("systemctl --user is-active --quiet spse-robot-audio") != 0:
            process = Process(target=robot.main_loop)
            print("Starting main loop")
            process.start()
            print("Starting spse-robot-audio")
            os.system("systemctl --user start spse-robot-audio")
        else:
            try:
                print("Stopping main loop")
                process.terminate()
                process.join()
            except AttributeError:
                pass
            process = None
            print("Stopping spse-robot-audio")
            os.system("systemctl --user stop spse-robot-audio")