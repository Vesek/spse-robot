from src.picam import Camera
import cv2

cam = Camera()

cv2.imwrite("9.png", cam.capture())

cam.deinit()
