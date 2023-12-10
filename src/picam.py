from picamera2 import Picamera2
import cv2

class Camera:
    def __init__(self,*args):
        self.picam2 = Picamera2()
        config = self.picam2.create_preview_configuration(raw=self.picam2.sensor_modes[5])
        self.picam2.configure(config)
        self.picam2.start()

    def __del__(self):
        del self.picam2

    def capture(self):
        array = self.picam2.capture_array()
        array = array[:,80:480,:] # Hopefully won't need this
        # array = array[:,:480,:]
        array = cv2.rotate(array, cv2.ROTATE_90_COUNTERCLOCKWISE)
        return array