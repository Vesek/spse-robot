import cv2

class Camera:
    def __init__(self,*args):
        self.cam = cv2.VideoCapture(0)
        if not self.cam.read()[0]:
            raise Exception("Camera error")

    def capture(self):
        check, frame = self.cam.read()
        frame = cv2.resize(frame, (640,480))
        return frame

    def deinit(self):
        self.cam.release()