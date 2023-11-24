if __name__ == '__main__':
    import RPi.GPIO as GPIO
    import systemd.daemon
    import time
    import os

    GPIO.setmode(GPIO.BCM)
    GPIO.setup(21, GPIO.IN, GPIO.PUD_UP)
    def onButton(channel):
        print("Button pressed!")
        if os.system("systemctl --user is-active --quiet spse-robot") != 0:
            print("Starting spse-robot")
            os.system("systemctl --user start spse-robot")
        else:
            print("Stopping spse-robot")
            os.system("systemctl --user stop spse-robot")
        # os.system("")
    GPIO.add_event_detect(21, GPIO.FALLING, callback=onButton, bouncetime=1000)
    systemd.daemon.notify('READY=1')
    while True:
        time.sleep(1)