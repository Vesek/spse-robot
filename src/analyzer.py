import cv2
import numpy as np
import math
import time

class Analyzer:
    def __init__(self, save_times=False):
        self.save_times = save_times
        if self.save_times:
            self.framecounter = 0
            self.times = {}
    
    def preprocessing(self,frame,otsu=False,kernel_size=(3,3)):
        if self.save_times:
            begin_time = time.time() # Save the time at which the preprocessing started
            self.framecounter += 1 # Add one to the framecounter, as we can safely assume that preprocessing is the first point where this class interacts with a frame
        framegray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY) # Convert frame to grayscale

        if otsu: # Pick which thresholding to use based on arguments
            ret,th = cv2.threshold(framegray,0,255,cv2.THRESH_OTSU)
        else:
            th = cv2.adaptiveThreshold(framegray,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,cv2.THRESH_BINARY,33,4)

        inverted = cv2.bitwise_not(th) # Invert frame

        kernel = np.ones(kernel_size,np.uint8)
        opening = cv2.morphologyEx(inverted, cv2.MORPH_OPEN, kernel) # Carry out a morphological opening on the thresholded image

        if self.save_times: self.times['preprocessing_time'] = round(time.time()-begin_time,5) # Save how much time did preprocessing take

        return opening
    
    def detect_lines(self,frame):
        if self.save_times: begin_time = time.time() # Save the time at which edge detection started
        canny = cv2.Canny(frame, 1, 3)
        if self.save_times: self.times['edge_detection_time'] = round(time.time()-begin_time,5) # Save how much time did edge detection take

        if self.save_times: begin_time = time.time() # Save the time at which line detection started
        rho = 1
        theta = np.pi/180
        threshold = 60
        min_line_length = 30
        max_line_gap = 20
        lines = cv2.HoughLines(canny, rho, theta, threshold, None,
                                min_line_length, max_line_gap, -1, 1)
        if self.save_times: self.times['line_time'] = round(time.time()-begin_time,5) # Save how much time did line detection take

        return lines

    def process_lines(self,lines,frame=None): # No time checks here because it takes a very small amount of time
        if lines is not None:
            theta_avg = 0
            for line in lines:
                theta = line[0][1]
                theta_avg += theta
                if frame is not None:
                    rho = line[0][0]
                    a = math.cos(theta)
                    b = math.sin(theta)
                    x0 = a * rho
                    y0 = b * rho
                    pt1 = (int(x0 + 1000*(-b)), int(y0 + 1000*(a)))
                    pt2 = (int(x0 - 1000*(-b)), int(y0 - 1000*(a)))
                    cv2.line(frame, pt1, pt2, (0,0,255), 1, cv2.LINE_AA)
            theta_avg = theta_avg / len(lines)
            return theta_avg, frame
        return None, frame
    