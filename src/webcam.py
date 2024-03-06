import cv2
import numpy as np


class Camera:
    def __init__(self, *args):
        self.cam = cv2.VideoCapture(0)
        if not self.cam.read()[0]:
            raise Exception("Camera error")

    def __del__(self):
        self.cam.release()

    def capture(self):
        check, frame = self.cam.read()
        if frame is None:
            frame = np.zeros((640, 480, 3), dtype=np.uint8)
        frame = cv2.resize(frame, (640, 480))
        return frame
