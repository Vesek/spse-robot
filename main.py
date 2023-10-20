import cv2
import numpy as np
import math
from picamera2 import Picamera2

running_on_rpi = True

if running_on_rpi: picam2 = Picamera2()
else: cam = cv2.VideoCapture(0)

if running_on_rpi:
    config = picam2.create_preview_configuration({"format": 'RGB888', "size": (640, 480)})
    picam2.configure(config)
    picam2.start()

while True:
    if running_on_rpi: frame = picam2.capture_array()
    else: check, frame = cam.read()
    print(frame.shape)

    framegray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)

    # ret2,th2 = cv2.threshold(framegray,0,255,cv2.THRESH_OTSU)

    th3 = cv2.adaptiveThreshold(framegray,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,cv2.THRESH_BINARY,33,2)

    inverted = cv2.bitwise_not(th3)

    kernel = np.ones((5,5),np.uint8)
    opening = cv2.morphologyEx(inverted, cv2.MORPH_OPEN, kernel)

    image_canny = cv2.Canny(opening, 1, 3)

    rho = 1
    theta = np.pi/180
    threshold = 60
    min_line_length = 50
    max_line_gap = 10
    line_image = np.copy(frame)

    lines = cv2.HoughLines(image_canny, rho, theta, threshold, None,
                            min_line_length, max_line_gap)

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
            print(theta-2)
            cv2.line(line_image, pt1, pt2, (0,0,255), 1, cv2.LINE_AA)
    cv2.imshow('video', line_image)
    # cv2.imwrite("sus.png",line_image)
    # break
    key = cv2.waitKey(1)
    if key == 27:
        break

if not running_on_rpi: cam.release()
cv2.destroyAllWindows()
