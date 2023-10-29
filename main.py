import time

import cv2
import numpy as np
from src.analyzer import Analyzer
import sys

INP = 69  # CHANGE!!!

def print_help():
    print("Line following robot for SPŠE by Plajta <3")
    print("Usage: main.py [arg1] [arg2] ...")
    print("When no args are entered the script just starts with default settings")
    print("\nArgs:")
    print("\t-h\t\tPrint this help and exits")
    print("\t-g\t\tStarts with a graphics output, if on a Raspberry Pi, automatically assumes that framebuffer should be used")
    print("\t--nofb\t\tForcibly disables framebuffer output")
    print("\t--nomotors\tForcibly disables motors")
    print("\t-m\t\tPrints frametimes after every frame")
    print("\t--hough\t\tSwitches processing from counting pixels in a column to Hough transform")

def main(args):
    # Default flags
    running_on_rpi = True
    headless = True
    in_fb = False
    perf_metrics = False
    enable_motors = False
    use_hough = False

    try:
        with open('/sys/firmware/devicetree/base/model') as f: # Check if running on an Raspberry Pi
            model = f.read()
            if "Raspberry" in model:
                running_on_rpi = True
                in_fb = True
                enable_motors = True
                try:
                    with open('/sys/class/graphics/fb0/virtual_size') as f: # Get framebuffer size
                        size = f.read()
                        fb_size = size[:-1].split(",")
                        fb_size = [int(side) for side in fb_size]
                        fb_size = tuple(fb_size)
                except FileNotFoundError:
                    in_fb = False
            else:
                running_on_rpi = False
    except FileNotFoundError: # If the file doesn't exist automatically assume a PC and disable framebuffer output
        running_on_rpi = False

    print(f"Running on a Raspberry Pi: {running_on_rpi}")

    if args is not []: # Check all args
        if "-h" in args:
            print_help()
            sys.exit()
        if "-g" in args:
            headless = False
        if "--nofb" in args:
            in_fb = False
        if "--nomotors" in args:
            enable_motors = False
        if "-m" in args:
            perf_metrics = False
        if "--hough" in args:
            use_hough = True

    # Import and init platform specific packages
    if running_on_rpi:
        import RPi.GPIO as GPIO
        from src.picam import Camera
        GPIO.setup(INP, GPIO.IN)
        if enable_motors:
            from src.motors import Motors
            motors = Motors()
            motors.enable()
    else:
        from src.webcam import Camera

    # Init general packages
    camera = Camera()
    analyzer = Analyzer(perf_metrics)

    try:
        while True: # Main loop
            if running_on_rpi and not GPIO.input(INP):
                motors.tocka()

            frame = camera.capture()

            preprocessed_frame = analyzer.preprocessing(frame)

            if use_hough:
                lines = analyzer.detect_lines(preprocessed_frame)

                line_image = np.copy(frame)

                if headless:
                    eccentricity, _ = analyzer.process_lines(lines)
                else:
                    eccentricity, out_image = analyzer.process_lines(lines, line_image)
            else:
                analyzer.count_columns(preprocessed_frame)
                out_image = preprocessed_frame
                eccentricity = None

            if eccentricity is not None:
                speed = [0x2222,0x2222]
                coefficent = 0.6
                output = (1-abs(eccentricity)*coefficent)
                if eccentricity < 0:
                    speed[0] = round(speed[0] * output)
                else:
                    speed[1] = round(speed[1] * output)
                if enable_motors: motors.speed = speed
                print(speed)

            if not headless: # Display output
                if in_fb:
                    frame32 = cv2.cvtColor(out_image, cv2.COLOR_BGR2BGRA)
                    fbframe = cv2.resize(frame32, fb_size)
                    with open('/dev/fb0', 'rb+') as buf:
                        buf.write(fbframe)
                else:
                    cv2.imshow('video', out_image)
                    if cv2.waitKey(1) == 27:
                        break
            
            if perf_metrics: print(analyzer.framecounter,analyzer.times)
    except KeyboardInterrupt:
        pass

    print("\nQuitting because of a keyboard interrupt\n")
    if running_on_rpi and enable_motors: motors.deinit()
    camera.deinit()
    if not headless and not in_fb: cv2.destroyAllWindows()
    sys.exit()

if __name__ == "__main__":
   main(sys.argv[1:])