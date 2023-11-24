if __name__ == '__main__':
    import RPi.GPIO as GPIO
    import systemd.daemon
    import time
    import os

    GPIO.setmode(GPIO.BCM)
    GPIO.setup(21, GPIO.IN, GPIO.PUD_UP)
    def onButton(channel):
        print("Button pressed! Starting script")
        # os.system("bash /home/pi/spse-robot/")
    GPIO.add_event_detect(21, GPIO.FALLING, callback=onButton, bouncetime=500)
    systemd.daemon.notify('READY=1')
    while True:
        time.sleep(1)
