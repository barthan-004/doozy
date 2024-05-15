#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray
import math

pub = None
rate =None
def callback(joint_state):
    global pub
    global rate
    msg = Float32MultiArray()
    # Assuming position[2] is the angle in radians
    msg.data.append(math.degrees(joint_state.position[6]))
    msg.data.append(math.degrees(joint_state.position[7]))
    msg.data.append(math.degrees(joint_state.position[8]))
    msg.data.append(math.degrees(joint_state.position[9]))
    msg.data.append(math.degrees(joint_state.position[10]))
    msg.data.append(math.degrees(joint_state.position[11]))
    msg.data.append(math.degrees(joint_state.position[12]))
    msg.data.append(math.degrees(joint_state.position[13]))
    msg.data.append(math.degrees(joint_state.position[14]))
    msg.data.append(math.degrees(joint_state.position[15]))
    msg.data.append(math.degrees(joint_state.position[16]))
    msg.data.append(math.degrees(joint_state.position[17]))  
    msg.data.append(math.degrees(joint_state.position[3]))
    msg.data.append(math.degrees(joint_state.position[4]))
    msg.data.append(math.degrees(joint_state.position[5])) 
    msg.data.append(math.degrees(joint_state.position[2])) 
    pub.publish(msg)

def main():
    global pub
    global rate
    rospy.init_node("joint_servo")

    sub = rospy.Subscriber("/joint_states", JointState, callback)
    pub = rospy.Publisher("joint_angle_servo", Float32MultiArray, queue_size=1)
    rospy.spin()

if __name__ == '__main__':
    main()
