import cv2
import numpy as np
import math
from picamera2 import Picamera2
import time

headless = True
running_on_rpi = True

framecounter = 0
lastsecond = 0

if running_on_rpi: picam2 = Picamera2()
else: cam = cv2.VideoCapture(0)

if running_on_rpi:
    config = picam2.create_preview_configuration({"format": 'RGB888', "size": (640, 480)}, raw=picam2.sensor_modes[4])
    picam2.configure(config)
    picam2.start()

while True:
    frametime = time.time()
    if running_on_rpi: frame = picam2.capture_array()
    else: check, frame = cam.read()
    # print(frame.shape)
    gettime = time.time()
    framegray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)

    # ret,th = cv2.threshold(framegray,0,255,cv2.THRESH_OTSU)

    th = cv2.adaptiveThreshold(framegray,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,cv2.THRESH_BINARY,33,2)

    inverted = cv2.bitwise_not(th)
    graythreshold = time.time()
    kernel = np.ones((5,5),np.uint8)
    opening = cv2.morphologyEx(inverted, cv2.MORPH_OPEN, kernel)

    canny = cv2.Canny(opening, 1, 3)

    rho = 1
    theta = np.pi/180
    threshold = 60
    min_line_length = 50
    max_line_gap = 10
    line_image = np.copy(frame)
    morphedge = time.time()
    lines = cv2.HoughLines(canny, rho, theta, threshold, None,
                            min_line_length, max_line_gap)
    houghtime = time.time()
    if lines is not None:
        render_lines = [line for line in lines if line[0][1] > 2 or line[0][1] < 1]
        for line in render_lines:
            rho = line[0][0]
            theta = line[0][1]
            a = math.cos(theta)
            b = math.sin(theta)
            x0 = a * rho
            y0 = b * rho
            pt1 = (int(x0 + 1000*(-b)), int(y0 + 1000*(a)))
            pt2 = (int(x0 - 1000*(-b)), int(y0 - 1000*(a)))
            # print(theta-2)
            cv2.line(line_image, pt1, pt2, (0,0,255), 1, cv2.LINE_AA)
    framecounter += 1
    if not headless:
        cv2.imshow('video', line_image)
        if cv2.waitKey(1) == 27:
            break
    print(f"Frame: {framecounter} Total Frametime: {time.time()-frametime} Gettime: {gettime-frametime} Graythreshold: {graythreshold-gettime} Morphedge: {morphedge-graythreshold} Houghtime: {houghtime-morphedge}")
    # cv2.imwrite("sus.png",line_image)
    # break

if not running_on_rpi: cam.release()
if not headless: cv2.destroyAllWindows()
