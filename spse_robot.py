import time
import cv2
import numpy as np
from src.analyzer import Analyzer
import sys
import argparse
import signal
import os
from multiprocessing import shared_memory

class Robot:
    def __init__(self, camera, args, mp_analyzer=None):
        self.camera = camera
        self.args = args
        self.mp_analyzer = mp_analyzer

        if self.args.servo:
            self.radial_speed_servo = 90 # In degrees per second
            self.max_angle = 110

    def main_loop(self):
        if self.args.running_on_rpi and self.args.motors:
            from src.motors import Motors
            motors = Motors()
            motors.enable()

        analyzer = Analyzer()
        if self.mp_analyzer is not None:
            analyzer_shm = shared_memory.SharedMemory(name=self.mp_analyzer)
            color_placeholder = np.zeros((640,480,3),dtype=np.uint8)
            cv2.putText(img=color_placeholder, text="MULTITHREADED COLORS", org=(20, 30), fontFace=cv2.FONT_HERSHEY_TRIPLEX, fontScale=1, color=(255, 0, 0),thickness=2)

        last_E = 0
        start_time = time.time()
        last_time = start_time

        stop_time = None
        desired_speed = self.args.speed
        speed = 0
        if self.args.running_on_rpi and self.args.motors: motors.angle = 0
        if self.args.detect_colors:
            verdict_o_meter = [0,0,0] # Color, Number of frames with color, Number of frames from last color
            min_color_frames = 3
            max_noncolor_frames = 2

        self.run = True

        def handler(signal, frame):
            self.run = False

        signal.signal(signal.SIGTERM, handler)

        try:

            while self.run:  # Main loop
                frame = self.camera.capture()

                preprocessed_frame, thresh = analyzer.preprocessing(frame, otsu=True, kernel_size=(5, 5))
                deviation, out_image, contour = analyzer.find_centroid(preprocessed_frame,render=not self.args.headless)
                if self.args.detect_colors:
                    if self.mp_analyzer is None:
                        verdict, color = analyzer.find_colors(frame, render=not self.args.headless, otsu=True, centroid=deviation, thresh=thresh)
                    else:
                        verdict = [int.from_bytes(analyzer_shm.buf),0]
                        color = color_placeholder
                        

                if self.args.stop_on_line:
                    sf_detect = analyzer.stop_line_detect(contour, (int(frame.shape[0]*0.15), int(frame.shape[1]*0.42)), (int(frame.shape[0]*0.85), int(frame.shape[1]*0.42))) # Completely ✨ arbitrary ✨ numbers
                    if sf_detect and ((time.time() - start_time) > 10):
                        stop_time = time.time()
                    if stop_time is not None and ((time.time() - stop_time) > 0.5):
                        break
                
                if self.args.detect_colors:
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
                        if verdict_o_meter[0] == 2:
                            print("Green, new desired speed")
                            desired_speed = start_speed
                            
                        verdict_o_meter = [0,0,0]
                    if verdict_o_meter[0] != 0 and verdict_o_meter[1] <= min_color_frames and verdict_o_meter[2] >= max_noncolor_frames:
                        verdict_o_meter = [0,0,0]
                    # print(verdict,verdict_o_meter)
                
                # out_image[:,:,0] = color[:,:,0]
                # np.logical_or(color[:,:,0],out_image[:,:,0],out_image[:,:,0])
                # cv2.addWeighted(color,0.5,out_image,0.5,0)

                if deviation is not None:
                    now_time = time.time()

                    if speed < desired_speed:
                        speed += self.args.accel*(now_time-last_time)
                    if speed > desired_speed:
                        speed = desired_speed

                    Kp = 1.3
                    Kd = 0.15
                    E = 1 - abs(deviation)

                    P = E * Kp

                    D = ((E - last_E) / (now_time - last_time)) * Kd
                    output = max(min(P + D, 1), 0)
                    if deviation < 0:
                        out_speed = [round(speed * output), round(speed)]
                    else:
                        out_speed = [round(speed) , round(speed * output)]
                    if self.args.running_on_rpi and self.args.motors:
                        motors.speed = out_speed
                        motors.angle += radial_speed_servo*(now_time-last_time)
                        if motors.angle > max_angle:
                            motors.angle = max_angle
                            radial_speed_servo *= -1
                        elif motors.angle <= 0:
                            motors.angle = 0
                            radial_speed_servo *= -1
                    last_time = now_time
                    last_E = E
                    if self.args.verbose:
                        print(out_speed)

                if not self.args.headless:  # Display output
                    if self.args.show_preprocessed: out_image = preprocessed_frame
                    if self.args.show_raw: out_image = frame
                    if self.args.running_on_rpi and self.args.use_fb:
                        frame32 = cv2.cvtColor(out_image, cv2.COLOR_BGR2BGRA)
                        fbframe = cv2.resize(frame32, self.args.fb_size)
                        with open('/dev/fb0', 'rb+') as buf:
                            buf.write(fbframe)
                    else:
                        cv2.imshow('video', out_image)
                        if cv2.waitKey(1) == 27:
                            break
        except KeyboardInterrupt:
            print("\nStopped the main loop due to a keyboard interrupt\n")

        print("\nStopped the main loop\n")
        if not self.args.headless and not self.args.use_fb: cv2.destroyAllWindows()


if __name__ == "__main__":
    os.environ["LIBCAMERA_LOG_LEVELS"] = "3"
    description = "Line following robot by Plajta made for SPŠE\nUsage without prior code adjustments is not recommended, the code is made specifically for our hardware configuration."
    parser = argparse.ArgumentParser(description=description,formatter_class=argparse.RawTextHelpFormatter)

    parser.add_argument('-g', '--graphics', help="Starts with a graphics output, if on a Raspberry Pi, automatically assumes that framebuffer should be used", action='store_false', dest="headless")
    parser.add_argument('--nofb', help="Forcibly disables framebuffer output", action='store_false', dest="use_fb")
    parser.add_argument('--nomotors', help="Forcibly disables motors", action='store_false', dest="motors")
    parser.add_argument('-r','--raw', help="If graphics is enabled, forces rendering of the raw image - for debugging use", action='store_true', dest="show_raw")
    parser.add_argument('-p','--preprocessed', help="If graphics is enabled, forces rendering of the preprocessed image - for debugging use", action='store_true', dest="show_preprocessed")
    parser.add_argument('-i','--image', help="Loads an image from the filesystem as a \"camera\"", type=str)
    parser.add_argument('--stop', help="When enabled the robot will try to stop on the finish line", action='store_true', dest="stop_on_line")
    parser.add_argument('-c','--colors', help="When enabled the robot will try to react to colored markings", action='store_true', dest="detect_colors")
    parser.add_argument('-s','--speed', help="Sets a new start speed (default = 0x6666)", default=0x6666, type=int)
    parser.add_argument('--accel','--acceleration', help="Sets a new start speed (default = 0x3333)", default=0x3333, type=int)
    parser.add_argument('--servo', help="Enables the serveo", action='store_true')
    parser.add_argument('-v','--verbose', help="Prints aditional info", action='store_true')

    args = parser.parse_args() # headless, use_fb, motors, show_raw, show_preprocessed, image, stop_on_line, detect_colors, speed, acceleration, servo
    
    try:
        with open('/sys/firmware/devicetree/base/model') as f:  # Check if running on an Raspberry Pi
            model = f.read()
            if "Raspberry" in model:
                args.running_on_rpi = True
                try:
                    with open('/sys/class/graphics/fb0/virtual_size') as f:  # Get framebuffer size
                        size = f.read()
                        fb_size = size[:-1].split(",")
                        fb_size = [int(side) for side in fb_size]
                        args.fb_size = tuple(fb_size)
                except FileNotFoundError:
                    args.use_fb = False
            else:
                args.running_on_rpi = False
    except FileNotFoundError:  # If the file doesn't exist automatically assume a PC
        args.running_on_rpi = False

    print(f"Running on a Raspberry Pi: {args.running_on_rpi}")

    if args.image is not None:
        from src.virtcam import Camera
    else:
        if args.running_on_rpi:
            from src.picam import Camera
        else:
            from src.webcam import Camera

    try:
        camera = Camera(args.image)
    except:
        print("There as been an error with the initialization of the camera")
        sys.exit()
    
    print(args)
    robot = Robot(camera=camera,args=args)
    robot.main_loop()