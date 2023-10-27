import cv2

class Camera:
    def __init__(self):
        self.cam = cv2.VideoCapture(0)

    def capture(self):
        check, frame = cam.read()
        return frame

    def deinit(self):
        del self.picam2