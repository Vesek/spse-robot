# spse-robot
Software for a line following robot that me and my friend Honza Ocelík built for our school (VOŠ a SPŠE Plzeň) in the school year 2023/24.

The software for written for running on a Raspberry Pi (tested on RPi 4 and Zero 2 W) but can also run on a PC with a webcam.

It was written with Python 3.11 in mind, so if it stops working in future versions, create an issue.

### Setup on PC

I recommend using a Python venv for testing on your PC:
```
python -m venv env
pip install -r requirements.txt
```

### Setup on RPi

You can use a venv but in the case of the Pi I don't think it's necessary.
```
sudo apt install python3-picamera2 python3-rpi.gpio python3-systemd python3-opencv python3-numpy
```

For the framebuffer output to work correctly, it needs this added to `/boot/cmdline.txt` of your RPi:
```
video=HDMI-A-1:-32
```
This argument sets the color depth to 32 bits so the frames can be rendered in full color. My attempts to make it work in 16 bit were unsuccesful.

## Usage
For usage on you piece of hardware, you should review the code first, it is very much coded for our robot first and there are a lot of hardcoded values. Even the software part of things isn't much better, the systemd services have a hardcoded path to /home/pi/spse-robot (might change that).

You can check the script's built-in help using
```
python spse_robot.py --help
```

If you want a clean output to a framebuffer use the `blank_fb.sh` script.