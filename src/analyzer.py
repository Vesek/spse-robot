import cv2
import numpy as np


class Analyzer:
    def __init__(self):
        pass

    def find_colors(self, frame, render=False, otsu=True, centroid=None, thresh=None, mask=None):  # If it's stupid and it works, it is not stupid.
        if centroid is not None:  # Cuts off the left part of the line because there will never be a color dot there (at least in our case)
            cut = int((centroid + 1) * (frame.shape[1] / 2))
            channels = [frame[:, cut:, 0], frame[:, cut:, 1], frame[:, cut:, 2]]
            mask = mask[:,cut:]
        else:
            channels = [frame[:, :, 0], frame[:, :, 1], frame[:, :, 2]]
        for i in range(len(channels)):
            if otsu:  # Pick which thresholding to use based on arguments
                blur = channels[i]
                # blur = cv2.GaussianBlur(channels[i], (3, 3), 0)  # Decided on with  C R E A T I V E  measures
                if thresh is None:  # If combined thresholding is enabled use that, otherwise use otsu
                    ret, th = cv2.threshold(blur, 0, 255, cv2.THRESH_OTSU)
                else:
                    ret, th = cv2.threshold(blur, thresh, 255, cv2.THRESH_BINARY)
            else:  # THIS IS NOT TESTED, ADAPTIVE THRESHOLDING SUCKS ANYWAY
                th = cv2.adaptiveThreshold(channels[i], 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 33, 4)
            kernel = np.ones((13, 13), np.uint8)
            opening = cv2.morphologyEx(th, cv2.MORPH_ERODE, kernel)  # For good measure :)
            channels[i] = opening #[int(opening.shape[0]*0.5):,int(opening.shape[1]*0.5):]
        # red = np.bitwise_and(channels[2],np.bitwise_not(channels[1]))
        # green = np.bitwise_and(channels[1],np.bitwise_not(channels[2]))
        red = np.bitwise_and(np.bitwise_and(channels[0], np.bitwise_not(channels[1])), np.bitwise_not(channels[2]))  # Remove white and off color things from our filtered images
        green = np.bitwise_and(np.bitwise_and(channels[1], np.bitwise_not(channels[0])), np.bitwise_not(channels[2]))
        if mask is not None:
            kernel = np.ones((5, 5), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_ERODE, kernel)
            np.bitwise_and(red, mask)
            np.bitwise_and(green, mask)
        red_contours, red_hierarchy = cv2.findContours(red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)  # This is just straight up stupid but i want to go to sleep earlier than yesterday
        green_contours, green_hierarchy = cv2.findContours(green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        #if render:
        #    red[:, :] = 0
        #    green[:, :] = 0
        verdict = [0, 0]

        if len(red_contours) != 0:
            red_contour = max(red_contours, key=cv2.contourArea)
            if cv2.contourArea(red_contour) >= 1500:  # Save the final verdict only if it's pretty confident
                if render:
                    cv2.drawContours(red, [red_contour], -1, 255, 3)
                verdict = [1, cv2.contourArea(red_contour)]
        if len(green_contours) != 0:
            green_contour = max(green_contours, key=cv2.contourArea)
            if cv2.contourArea(green_contour) >= 1500:  # Save the final verdict only if it's pretty confident
                if render:
                    cv2.drawContours(green, [green_contour], -1, 255, 3)
                verdict = [2, cv2.contourArea(green_contour)]

        # for i in range(len(channels)):
        #     output[:,:,i] = channels[i]
        output = None
        if render:
            output = np.zeros(frame.shape, np.uint8)
            if centroid is not None:  # Put the image back together for added ✨ flare ✨
                output[:, cut:, 2] = red
                output[:, cut:, 1] = green
            else:
                output[:, :, 2] = red
                output[:, :, 1] = green
        # print(verdict)
        return verdict, output

    def preprocessing(self, frame, otsu=False, kernel_size=(3, 3)):
        framegray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)  # Convert frame to grayscale

        ret = None

        if otsu:  # Pick which thresholding to use based on arguments
            ret, th = cv2.threshold(framegray, 0, 255, cv2.THRESH_OTSU)
        else:
            th = cv2.adaptiveThreshold(framegray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 33, 4)

        inverted = cv2.bitwise_not(th)  # Invert frame

        kernel = np.ones(kernel_size, np.uint8)
        opening = cv2.morphologyEx(inverted, cv2.MORPH_OPEN, kernel)  # Carry out a morphological opening on the thresholded image

        return opening, ret

    def find_centroid(self, frame, render=False):
        contours, hierarchies = cv2.findContours(frame, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)  # Find all the contours
        deviation = 0
        out_image = None
        contour = None
        if contours is not None and contours != ():  # Yes, i could make this smaller, no, I will not do that, spent too much time on this 'if'
            contour = max(contours, key=cv2.contourArea)  # Get the biggest contour (by area)
            moments = cv2.moments(contour)  # Honestly don't have a clue what the fuck this is but it works sooooooo
            cx = int(moments['m10'] / moments['m00'])
            deviation = (cx - frame.shape[1] / 2) / (frame.shape[1] / 2)
            # Calculate the orientation of each contour segment

            if render:
                out_image = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
                cv2.drawContours(out_image, [contour], -1, (255, 0, 0), 3)  # Render the line
                cy = int(moments['m01'] / moments['m00'])
                cv2.circle(out_image, (cx, cy), 7, (0, 0, 255), -1)  # Render the centroid
                # cv2.putText(img=out_image, text=str(cv2.contourArea(contour)), org=(20, 30), fontFace=cv2.FONT_HERSHEY_TRIPLEX, fontScale=1, color=(0, 0, 255), thickness=2)
                cv2.putText(img=out_image, text=str(deviation), org=(20, 70), fontFace=cv2.FONT_HERSHEY_TRIPLEX, fontScale=1, color=(255, 0, 0), thickness=2)  # Some debug text (also it looks cool)
            #print(cv2.contourArea(contour))
            #if cv2.contourArea(contour) > 70000:
            #    deviation = None
        return deviation, out_image, contour

    def stop_line_detect(self, contour, point1, point2):  # Same as the colors, if it's stupid and it works, it is not stupid
        if contour is not None:  # It just checks if those two pixels are in the line, simple but effective
            dst1 = cv2.pointPolygonTest(contour, point1, True)
            dst2 = cv2.pointPolygonTest(contour, point2, True)
            return (dst1 > -20) and (dst2 > -20)
        return False
