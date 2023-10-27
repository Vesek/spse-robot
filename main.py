import cv2
import numpy as np
import math
import time

# Flags (maybe will add as arguments)
running_on_rpi = True
headless = False
in_fb = True
perf_metrics = False

if running_on_rpi:
    from src.picam import Camera
    from src.motors import Motors
    motors = Motors()
    motors.enable()
else:
    from src.webcam import Camera

if perf_metrics: framecounter = 0

camera = Camera()

while True:
    if perf_metrics: frametime = time.time()

    frame = camera.capture()

    if perf_metrics:  gettime = time.time()
    
    framegray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)

    # ret,th = cv2.threshold(framegray,0,255,cv2.THRESH_OTSU)

    th = cv2.adaptiveThreshold(framegray,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,cv2.THRESH_BINARY,33,4)

    inverted = cv2.bitwise_not(th)
    if perf_metrics:  graythreshold = time.time()
    kernel = np.ones((3,3),np.uint8)
    opening = cv2.morphologyEx(inverted, cv2.MORPH_OPEN, kernel)

    canny = cv2.Canny(opening, 1, 3)

    rho = 1
    theta = np.pi/180
    threshold = 60
    min_line_length = 50
    max_line_gap = 10
    line_image = np.copy(frame)
    if perf_metrics: morphedge = time.time()
    lines = cv2.HoughLines(canny, rho, theta, threshold, None,
                            min_line_length, max_line_gap, -1, 1)
    if perf_metrics: houghtime = time.time()
    render_lines = []
    theta_avg = None
    if lines is not None:
        # render_lines = lines
        render_lines = [line for line in lines if line[0][1] > 0 or line[0][1] < 0]
        # print(render_lines)
        theta_avg = 0 if render_lines != [] else None
        for line in render_lines:
            rho = line[0][0]
            theta = line[0][1]
            a = math.cos(theta)
            b = math.sin(theta)
            x0 = a * rho
            y0 = b * rho
            pt1 = (int(x0 + 1000*(-b)), int(y0 + 1000*(a)))
            pt2 = (int(x0 - 1000*(-b)), int(y0 - 1000*(a)))
            theta_avg += theta
            # print(theta-2)
            cv2.line(line_image, pt1, pt2, (0,0,255), 1, cv2.LINE_AA)
    if theta_avg is not None:
        theta_avg = theta_avg / len(render_lines)
        print(theta_avg)
    if perf_metrics: framecounter += 1
    if not headless:
        if in_fb:
            frame32 = cv2.cvtColor(line_image, cv2.COLOR_BGR2BGRA)
            fbframe = cv2.resize(frame32, (1920,1080))
            with open('/dev/fb0', 'rb+') as buf:
                buf.write(fbframe)
        else:
            cv2.imshow('video', line_image)
            if cv2.waitKey(1) == 27:
                break
    if perf_metrics: print(f"Frame: {framecounter} Total Frametime: {time.time()-frametime} Gettime: {gettime-frametime} Graythreshold: {graythreshold-gettime} Morphedge: {morphedge-graythreshold} Houghtime: {houghtime-morphedge}")
    # cv2.imwrite("sus.png",line_image)
    # break

if not running_on_rpi: cam.release()
if not headless: cv2.destroyAllWindows()
