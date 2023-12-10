import cv2

class Camera:
    def __init__(self,path):
        self.image = cv2.resize(cv2.imread(path), (640,480))
        print(path)

    def __del__(self):
        pass

    def capture(self):
        return self.image