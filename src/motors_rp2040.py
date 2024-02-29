import time
from spidev import SpiDev

class Motors:

    def __init__(self):
        self._speed = [0,0]
        self._angle = 0
        # Init spi
        self.spi = SpiDev()
        self.spi.open(0,0)

        self.spi.max_speed_hz = 4000000

        self.servo_range = 1000
        self.servo_step = self.servo_range / float(180)
        self.period = (1000000 / 100) # 100 is the frequency of the PWM

    def __del__(self):
        self.spi.xfer([0x00])

        self.spi.close()
        

    def enable(self,state=True):
        self.spi.xfer([state])

    @property
    def speed(self):
        return self._speed

    @property
    def angle(self):
        return self._angle

    @speed.setter
    def speed(self, newSpeed):
        self._speed = newSpeed
        msg = [0x10]

        h1,h2 = self._speed[0].to_bytes(2, "big")
        msg.append(h1)
        msg.append(h2)

        h1,h2 = self._speed[1].to_bytes(2, "big")
        msg.append(h1)
        msg.append(h2)

        self.spi.xfer(msg)

    @angle.setter
    def angle(self, newAngle):
        self._angle = newAngle
        if (self._angle < 0):
            self._angle = 0
        elif (self._angle > 180):
            self._angle = 180

        pulseWidth = self._angle * self.servo_step + 1000 # Angle * Degree represented in microseconds + Pulse for 0 degrees
        duty = int((pulseWidth * 100) / float(self.period) / 100 * 0xFFFF)
        h1,h2 = duty.to_bytes(2, "big")
        self.spi.xfer([0x20, h1, h2])
