from src.picam import Camera
import cv2

cam = Camera()

cv2.imwrite("test.png", cam.capture())

cam.deinit()
