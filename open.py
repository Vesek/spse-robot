import spidev
import time

spi = spidev.SpiDev()

spi.open(0, 0)

spi.max_speed_hz = 4000000

spi.xfer([0x02, 0x03]) # Init servos

servo_base = 501
servo_step = 1995 / float(180) # 1995 is the maximum range of the servo with some safeguards added
period = (1000000 / 100) # Perion of PWM in us

def move_servo(servo, duration, step, base_angle, end_angle):
    '''
    Servo - either A or B
    Duration - in seconds how much time it will take to complete the transition
    Step - time for a step in ms
    Base, End angle - the starting a stopping angle of the transition
    '''
    if servo == "A":
        command = 0x20
    elif servo == "B":
        command = 0x21
    else:
        raise Exception("Invalid servo")

    steps = int(duration/step) # Calculate the number of steps

    step_angle = int((end_angle-base_angle)/steps) # Calculate the of that step

    for i in range(steps):
        pulseWidth = base_angle + (step_angle * i) * servo_step + servo_base # Angle * Degree represented in microseconds + Pulse for 0 degrees
        duty = int((pulseWidth / float(period)) * 0xFFFF)
        h1,h2 = duty.to_bytes(2, "big")
        spi.xfer([command, h1, h2])
        time.sleep(step)
    
    
    

move_servo("B", 2, 0.1, 135, 90) # Open the lid

spi.xfer([0x00]) 

spi.close()