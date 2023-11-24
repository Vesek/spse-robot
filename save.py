from src.picam import Camera
import cv2

cam = Camera()

cv2.imwrite("13.png", cam.capture())

cam.deinit()
