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

        self.servo_base = 501
        self.servo_step = 1995 / float(180) # 1995 is the maximum range of the servo with some safeguards added
        self.period = (1000000 / 100) # Perion of PWM in us

    def __del__(self):
        self.spi.xfer([0x00])

        self.spi.close()
        

    def enable(self,state=True):
        self.spi.xfer([state, 0x05])

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

        pulseWidth = self._angle * self.servo_step + self.servo_base # Angle * Degree represented in microseconds + Pulse for 0 degrees
        duty = int((pulseWidth / float(self.period)) * 0xFFFF)
        h1,h2 = duty.to_bytes(2, "big")
        self.spi.xfer([0x20, h1, h2])
