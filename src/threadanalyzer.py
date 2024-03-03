from multiprocessing import shared_memory
from .analyzer import Analyzer
import numpy as np


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
        y_cut = int(len(self.buffer)*0.7)
        x_cut = int(len(self.buffer)*0.7)
        while True:
            out, _ = self.analyzer.find_colors(self.buffer[y_cut:,x_cut:,:])
            self.output_shm.buf[0] = out[0]
