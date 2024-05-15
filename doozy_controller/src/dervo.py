from __future__ import division
import time
import Adafruit_PCA9685

class ServoController:
    def __init__(self):
        # Initialize the PCA9685 board
        self.pwm = Adafruit_PCA9685.PCA9685(0x41, busnum=4)
        self.pwm.set_pwm_freq(60)
       # self.current_angles = [90, 135, 135]  # Initial angles of each servo
        # Define the minimum and maximum pulse lengths (in microseconds)
        # These values may need to be adjusted depending on your servo
        self.SERVO_MIN_PULSE = 150  # Min pulse length out of 4096
        self.SERVO_MAX_PULSE = 600  # Max pulse length out of 4096

    def angle_to_pulse(self, angle):
        # Convert angle (in degrees) to pulse length (in microseconds)
        pulse = (angle / 180.0) * (self.SERVO_MAX_PULSE - self.SERVO_MIN_PULSE) + self.SERVO_MIN_PULSE
        return int(pulse)

    def set_servo_angle(self, channel, angle):
        pulse = self.angle_to_pulse(angle)
        self.pwm.set_pwm(channel, 0, pulse)


a=ServoController()
while(1):
	b=int(input())
	a.set_servo_angle(11,b)
