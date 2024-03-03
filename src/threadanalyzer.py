from multiprocessing import shared_memory
from .analyzer import Analyzer
import numpy as np
import cv2


class ThreadAnalyzer:  # WHY DOES THIS HAVE TO EXIST (i want to unalive myself)
    def __init__(self, cam_mem_name, shape, dtype, out_mem_name):
        self.shm = shared_memory.SharedMemory(name=cam_mem_name)
        self.buffer = np.ndarray(shape, dtype=dtype, buffer=self.shm.buf)
        self.output_shm = shared_memory.SharedMemory(name=out_mem_name)
        self.analyzer = Analyzer()

    def __del__(self):
        self.shm.close()
        self.output_shm.close()
        self.output_shm.unlink()

    def main_loop(self):
        #with open('/sys/class/graphics/fb0/virtual_size') as f:  # Get framebuffer size
        #    size = f.read()
        #    fb_size = size[:-1].split(",")
        #    fb_size = [int(side) for side in fb_size]
        #    fb_size = tuple(fb_size)

        while True:
            out, _ = self.analyzer.find_colors(self.buffer, otsu=True, render=False)
           # frame32 = cv2.cvtColor(image, cv2.COLOR_BGR2BGRA)
           # fbframe = cv2.resize(frame32, fb_size)
           # with open('/dev/fb0', 'rb+') as buf:
           #     buf.write(fbframe)

            self.output_shm.buf[0] = out[0]
