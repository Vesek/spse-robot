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

    def find_colors(self,frame,otsu=False):
        channels = [frame[:,:,0],frame[:,:,1],frame[:,:,2]]
        for i in range(len(channels)):
            if otsu: # Pick which thresholding to use based on arguments
                blur = cv2.GaussianBlur(channels[i],(5,5),0)
                ret,th = cv2.threshold(blur,0,255,cv2.THRESH_OTSU)
                print(f"Channel: {i}, Ret: {ret}")
            else:
                th = cv2.adaptiveThreshold(channels[i],255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,cv2.THRESH_BINARY,33,4)
            channels[i] = th
        # red = np.bitwise_and(channels[2],np.bitwise_not(channels[1]))
        # green = np.bitwise_and(channels[1],np.bitwise_not(channels[2]))
        red = np.bitwise_and(np.bitwise_and(channels[2],np.bitwise_not(channels[1])),np.bitwise_not(channels[0]))
        green = np.bitwise_and(np.bitwise_and(channels[1],np.bitwise_not(channels[2])),np.bitwise_not(channels[0]))
        red_contours, red_hierarchy = cv2.findContours(red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        green_contours, green_hierarchy = cv2.findContours(green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        output = np.zeros(frame.shape,np.uint8)
        red[:,:] = 0
        green[:,:] = 0
        verdict = 0

        if len(red_contours) != 0:
            red_contour = max(red_contours, key = cv2.contourArea)
            cv2.drawContours(red, [red_contour], -1, 255, 3)
            if cv2.contourArea(red_contour) == 0:
                pass # Verdict
            
        if len(green_contours) != 0:
            green_contour = max(green_contours, key = cv2.contourArea)
            cv2.drawContours(green, [green_contour], -1, 255, 3)
            if cv2.contourArea(green_contour) == 0: 
                pass # Verdict
        
        # for i in range(len(channels)):
        #     output[:,:,i] = channels[i]
        output[:,:,2] = red
        output[:,:,1] = green
        return verdict,output
        
    
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

    def find_centroid(self,frame,render=False,stop_on_line=False):
        contours, hierarchies = cv2.findContours(frame, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        deviation = 0
        out_image = None
        sf_detect = False
        if contours is not None and contours != ():
            contour = max(contours, key = cv2.contourArea)
            moments = cv2.moments(contour)
            cx = int(moments['m10']/moments['m00'])
            deviation = (cx-frame.shape[1]/2)/(frame.shape[1]/2)
            # Calculate the orientation of each contour segment
            if stop_on_line:
                (x, y), (MA, ma), angle = cv2.fitEllipse(contour)
                # Filter contours based on orientation (close to 0 or close to 90 degrees)
                if ma < 800:
                    sf_detect = True

            if render:
                out_image = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
                cv2.drawContours(out_image, [contour], -1, (255, 0, 0), 3)
                cy = int(moments['m01']/moments['m00'])
                cv2.circle(out_image, (cx, cy), 7, (0, 0, 255), -1)
                cv2.putText(img=out_image, text=str(deviation), org=(20, 30), fontFace=cv2.FONT_HERSHEY_TRIPLEX, fontScale=1, color=(0, 0, 255),thickness=2)
                if stop_on_line:
                    cv2.putText(img=out_image, text=str(angle), org=(20, 60), fontFace=cv2.FONT_HERSHEY_TRIPLEX, fontScale=1, color=(0, 0, 255),thickness=2)
                    cv2.putText(img=out_image, text=str(MA), org=(20,90), fontFace=cv2.FONT_HERSHEY_TRIPLEX, fontScale=1, color=(0, 0, 255),thickness=2)
                    cv2.putText(img=out_image, text=str(ma), org=(20, 120), fontFace=cv2.FONT_HERSHEY_TRIPLEX, fontScale=1, color=(0, 0, 255),thickness=2)
        return deviation, out_image, sf_detect

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
    