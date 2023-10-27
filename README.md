# spse-robot
Software for a line following robot that me and my friend Honza Ocelík built for our school (VOŠ a SPŠE Plzeň) in the school year 2023/24.

## Setup
The software for written for running on a Raspberry Pi but can also run on a PC with a webcam.

It was written with Python 3.11 in mind so if it stops working in future versions create an issue.

I recommend using a Python venv for the requirements (even tho there aren't many):
```
python -m venv --system-site-packages env
pip install -r requirements.txt
sudo apt install python3-picamera2
```
It needs `--system-site-packages` because picamera2 cannot be installed from pip (at the time of writing it crashes).

For the framebuffer output to work correctly it need this added to `/boot/cmdline.txt` on your RPi:
```
video=HDMI-A-1:1920x1080M-32@60
```
It sets the color depth on the connector HDMI0 to 32 bits so the frames can be rendered in full color. My attempts to make it work in 16 bit were unsuccesful because of a lot of artifacts.

## Usage
You can check the script's built in help using
```
python main.py -h
```

If you want a clean output to a framebuffer use the `blank_fb.sh` script.

## TODO

- [ ] Make a C++ version (maaaaaybe)
