#!/usr/bin/env python3
from __future__ import division
import rospy
from std_msgs.msg import Float32MultiArray
from Adafruit_PCA9685 import PCA9685

pwm = PCA9685(0x41, busnum=4)
pwm.set_pwm_freq(60)

SERVO_MIN_PULSE = 150  # Min pulse length out of 4096
SERVO_MAX_PULSE = 600  # Max pulse length out of 4096

def angle_to_pulse(angle):
    # Convert angle (in degrees) to pulse length (in microseconds)
    pulse = (angle / 180.0) * (SERVO_MAX_PULSE - SERVO_MIN_PULSE) + SERVO_MIN_PULSE
    return int(pulse)

def set_servo_angle(channel, angle):
    pulse = angle_to_pulse(angle)
    pwm.set_pwm(channel, 0, pulse)
    
def motor(i,pos):
    if(i==4):
        pos=90+(pos)
        set_servo_angle(11,pos)
    if(i==5):
        pos=90-(pos)
        set_servo_angle(14,pos)
    if(i==10):
        pos=30-(pos)
        set_servo_angle(13,pos)
    if(i==11):
        pos=90-(pos)
        set_servo_angle(15,pos)
    if(i==12):
        pos=70-(pos)
        set_servo_angle(12,pos)
    if(i==13):
        pos=160+(pos)
        set_servo_angle(10,pos)
    if(i==14):
        pos=110-(pos)
        set_servo_angle(9,pos)

def callback(msg):
    for i, pos in enumerate(msg.data):
        motor(i, pos)  

def servo_control():
    rospy.init_node('servo_control')
    rospy.Subscriber("joint_angle_servo", Float32MultiArray, callback)
    rospy.spin()

if __name__ == "__main__":
    servo_control()

