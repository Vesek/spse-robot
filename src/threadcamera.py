from multiprocessing import shared_memory
import numpy as np


class ThreadCamera:  # WHY DOES THIS HAVE TO EXIST (i want to unalive myself)
    def __init__(self, mem_name, shape, dtype):
        self.shm = shared_memory.SharedMemory(name=mem_name)
        self.buffer = np.ndarray(shape, dtype=dtype, buffer=self.shm.buf)

    def __del__(self):
        self.shm.close()

    def capture(self):
        return self.buffer
