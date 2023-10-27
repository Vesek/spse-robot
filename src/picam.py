from picamera2 import Picamera2
import cv2

class Camera:
    def __init__(self):
        self.picam2 = Picamera2()
        config = self.picam2.create_preview_configuration({"format": 'RGB888', "size": (640, 480)})
        self.picam2.configure(config)
        self.picam2.start()

    def capture(self):
        return self.picam2.capture_array()

    def deinit(self):
        del self.picam2