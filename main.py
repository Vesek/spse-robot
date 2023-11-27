import time
import cv2
import numpy as np
from src.analyzer import Analyzer
import sys
import time

INP = 21  # CHANGE!!!


def print_help():
    print("Line following robot for SPŠE by Plajta <3")
    print("Usage: main.py [arg1] [arg2] ...")
    print("Or: main.py -i [arg1] [arg2] ... [input]")
    print("When no args are entered the script just starts with default settings")
    print("\nArgs:")
    print("\t-h\t\tPrint this help and exits")
    print("\t-g\t\tStarts with a graphics output, if on a Raspberry Pi, automatically assumes that framebuffer should be used")
    print("\t--nofb\t\tForcibly disables framebuffer output")
    print("\t--nomotors\tForcibly disables motors")
    print("\t-m\t\tPrints frametimes after every frame")
    print("\t--hough\t\tSwitches processing from counting pixels in a column to Hough transform")
    print("\t-pi\t\tIf graphics is enabled, forces rendering of the preprocessed image for debugging use")
    print("\t-i\t\tWhen this is enabled, the last argument willbe treated like a path to an input image")
    print("\t--stop\t\tWhen enabled the robot will try to stop on the finish line")
    print("\t--colors\t\tWhen enabled the robot will try to react to colored markings")


def main(args):
    # Default flags
    running_on_rpi = True
    headless = True
    in_fb = False
    perf_metrics = False
    enable_motors = False
    use_hough = False
    do_tocka = False
    show_preprocessed_image = False
    virt_camera = False
    stop_on_line = False
    detect_colors = False
    start_speed = 0x6666
    int_speed = 0x0000
    acceleration = start_speed / 3 # Reach full speed in half a second
    radial_speed_servo = 90 # In degrees per second
    max_angle = 110
    path = ""

    try:
        with open('/sys/firmware/devicetree/base/model') as f:  # Check if running on an Raspberry Pi
            model = f.read()
            if "Raspberry" in model:
                running_on_rpi = True
                in_fb = True
                enable_motors = True
                try:
                    with open('/sys/class/graphics/fb0/virtual_size') as f:  # Get framebuffer size
                        size = f.read()
                        fb_size = size[:-1].split(",")
                        fb_size = [int(side) for side in fb_size]
                        fb_size = tuple(fb_size)
                except FileNotFoundError:
                    in_fb = False
            else:
                running_on_rpi = False
    except FileNotFoundError:  # If the file doesn't exist automatically assume a PC and disable framebuffer output
        running_on_rpi = False

    print(f"Running on a Raspberry Pi: {running_on_rpi}")

    if args is not []:  # Check all args
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
        if "--tocka" in args:
            do_tocka = True
        if "-pi" in args:
            show_preprocessed_image = True
        if "-i" in args:
            virt_camera = True
        if "--stop" in args:
            stop_on_line = True
        if "--colors" in args:
            detect_colors = True

    if virt_camera:
        path = args[-1]

    # Import and init platform specific packages
    if running_on_rpi:
        import RPi.GPIO as GPIO
        if enable_motors:
            from src.motors import Motors
            motors = Motors()
            motors.enable()
            if do_tocka:
                GPIO.setup(INP, GPIO.IN)
    else:
        from src.webcam import Camera

    # Camera import
    if virt_camera:
        from src.virtcam import Camera
    else:
        if running_on_rpi:
            from src.picam import Camera
        else:
            from src.webcam import Camera

    # Init general packages
    camera = Camera(path)
    analyzer = Analyzer(perf_metrics)
    last_E = 0
    start_time = time.time()
    last_time = time.time()
    stop_time = None
    desired_speed = start_speed
    if enable_motors: motors.angle = 0
    if detect_colors:
        verdict_o_meter = [0,0,0] # Color, Number of frames with color, Number of frames from last color
        min_color_frames = 3
        max_noncolor_frames = 2


    try:
        while True:  # Main loop
            if running_on_rpi and enable_motors and do_tocka and not GPIO.input(INP):
                motors.tocka()

            frame = camera.capture()

            if use_hough:
                preprocessed_frame = analyzer.preprocessing(frame)

                lines = analyzer.detect_lines(preprocessed_frame)

                line_image = np.copy(frame)

                if headless:
                    deviation, _ = analyzer.process_lines(lines)
                else:
                    deviation, out_image = analyzer.process_lines(lines, line_image)
            else:
                preprocessed_frame, thresh = analyzer.preprocessing(frame, otsu=True, kernel_size=(5, 5))
                deviation, out_image, contour = analyzer.find_centroid(preprocessed_frame, not headless)
                if detect_colors: verdict, color = analyzer.find_colors(frame, render=not headless, otsu=True, centroid=deviation, thresh=thresh)
                if stop_on_line:
                    sf_detect = analyzer.stop_line_detect(contour, (int(frame.shape[0]*0.15), int(frame.shape[1]*0.42)), (int(frame.shape[0]*0.85), int(frame.shape[1]*0.42))) # Completely ✨ arbitrary ✨ numbers
                    if sf_detect and ((time.time() - start_time) > 10):
                        stop_time = time.time()
                    if stop_time is not None and ((time.time() - stop_time) > 0.5):
                        break
                # if not headless and detect_colors: out_image = (color[:,:,:3] + out_image)[:,:,:3]
                if not headless and detect_colors: out_image = color
                if detect_colors:
                    if verdict[0] != 0:
                        if verdict_o_meter[0] == verdict[0]:
                            verdict_o_meter[1] +=1
                        elif verdict_o_meter[0] == 0:
                            verdict_o_meter = [verdict[0],1,0]
                    if verdict_o_meter[0] != verdict[0]:
                        verdict_o_meter[2] += 1
                    if verdict_o_meter[0] != 0 and verdict_o_meter[1] >= min_color_frames and verdict_o_meter[2] >= max_noncolor_frames:
                        if verdict_o_meter[0] == 1:
                            print("Red, new desired speed")
                            desired_speed = 0x4444
                            verdict_o_meter = [0,0,0]
                        if verdict_o_meter[0] == 2:
                            desired_speed = start_speed
                            print("Green, new desired speed")
                            verdict_o_meter = [0,0,0]
                    if verdict_o_meter[0] != 0 and verdict_o_meter[1] <= min_color_frames and verdict_o_meter[2] >= max_noncolor_frames:
                        verdict_o_meter = [0,0,0]
                    # print(verdict,verdict_o_meter)
                # out_image[:,:,0] = color[:,:,0]
                # np.logical_or(color[:,:,0],out_image[:,:,0],out_image[:,:,0])
                # cv2.addWeighted(color,0.5,out_image,0.5,0)
                # out_image = frame
            if deviation is not None:
                now_time = time.time()
                # print(now_time-last_time)
                if int_speed < desired_speed:
                    int_speed += acceleration*(now_time-last_time)
                if int_speed > desired_speed:
                    int_speed = desired_speed
                int_speed = int(int_speed)
                speed = [int_speed,int_speed]
                Kp = 1.3
                Kd = 0.15
                E = 1 - abs(deviation)
                P = E * Kp
                D = ((E - last_E) / (now_time - last_time)) * Kd
                output = max(min(P + D, 1), 0)
                if deviation < 0:
                    speed[0] = round(speed[0] * output)
                else:
                    speed[1] = round(speed[1] * output)
                if enable_motors:
                    motors.speed = speed
                    motors.angle += radial_speed_servo*(now_time-last_time)
                    if motors.angle > max_angle:
                        motors.angle = max_angle
                        radial_speed_servo *= -1
                    elif motors.angle <= 0:
                        motors.angle = 0
                        radial_speed_servo *= -1
                last_time = now_time
                last_E = E
                # print(speed)

            if not headless:  # Display output
                if show_preprocessed_image: out_image = preprocessed_frame
                if in_fb:
                    frame32 = cv2.cvtColor(out_image, cv2.COLOR_BGR2BGRA)
                    fbframe = cv2.resize(frame32, fb_size)
                    with open('/dev/fb0', 'rb+') as buf:
                        buf.write(fbframe)
                else:
                    cv2.imshow('video', out_image)
                    if cv2.waitKey(1) == 27:
                        break

            if perf_metrics: print(analyzer.framecounter, analyzer.times)
    except KeyboardInterrupt:
        pass

    print("\nQuitting because of a keyboard interrupt\n")
    if running_on_rpi and enable_motors: motors.deinit()
    camera.deinit()
    if not headless and not in_fb: cv2.destroyAllWindows()
    sys.exit()


if __name__ == "__main__":
    main(sys.argv[1:])
