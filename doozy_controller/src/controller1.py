#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32MultiArray
from ctypes import *
import sys
import time
sys.path.append('/home/doozy/catkin_ws/src/doozy_controller/src/unitree_actuator_sdk/lib/')
from unitree_actuator_sdk import *
import time
import ctypes
from ctypes import *
# Define the minimum and maximum pulse lengths (in microseconds)
# These values may need to be adjusted depending on your servo
SERVO_MIN_PULSE = 150  # Min pulse length out of 4096
SERVO_MAX_PULSE = 600  # Max pulse length out of 4096

serial =  SerialPort('/dev/unitree')
cmd = MotorCmd()  # Creating command objects for three motors
data = MotorData() # Creating data objects for three motors

motor_type = MotorType.GO_M8010_6
mode = queryMotorMode(motor_type, MotorMode.FOC)
current_angles = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0] 
b=[]
class VCI_INIT_CONFIG(Structure):
    _fields_ = [("AccCode", c_uint),
                ("AccMask", c_uint),
                ("Reserved", c_uint),
                ("Filter", c_ubyte),
                ("Timing0", c_ubyte),
                ("Timing1", c_ubyte),
                ("Mode", c_ubyte)
                ]

class VCI_CAN_OBJ(Structure):
    _fields_ = [("ID", c_uint),
                ("TimeStamp", c_uint),
                ("TimeFlag", c_ubyte),
                ("SendType", c_ubyte),
                ("RemoteFlag", c_ubyte),
                ("ExternFlag", c_ubyte),
                ("DataLen", c_ubyte),
                ("Data", c_ubyte*8),
                ("Reserved", c_ubyte*3)
                ]

canDLL = None
VCI_USBCAN2 = 4
STATUS_OK = 1 
def motor_initialization():
    global b
    a=[0,2,4,5,6]
    for i in range(5):
        data.motorType = MotorType.GO_M8010_6
        cmd.motorType = MotorType.GO_M8010_6
        cmd.id = a[i]
        cmd.mode = 0
        cmd.dq   = 0.0
        cmd.kp   = 0.01
        cmd.kd   = 0.01
        serial.sendRecv(cmd, data)
        b.append(data.q)
        print('\n')
        print(a[i])
        print(data.mode)
        print("q: " + str(data.q))
        print("dq: " + str(data.dq))
        print("temp: " + str(data.temp))
        print("merror: " + str(data.merror))
        print('\n')	
        #time.sleep(2)
        print(b)
    return b
def init_can():
    global canDLL
    global VCI_USBCAN2
    global STATUS_OK
    can_dll_name = '/home/doozy/zx/controlcan/libcontrolcan.so'
    canDLL = cdll.LoadLibrary('/home/doozy/zx/controlcan/libcontrolcan.so')

    print(can_dll_name)

    ret = canDLL.VCI_OpenDevice(VCI_USBCAN2, 0, 0)
    if ret == STATUS_OK:
        print("VCI_OpenDevice okk")
    else:
        print("VCI_OpenDevice fail")

    # Initialize CAN
    vci_initconfig = VCI_INIT_CONFIG(0x80000008, 0xFFFFFFFF, 0, 2, 0x00, 0x14, 0)  # Assuming your configuration
    ret = canDLL.VCI_InitCAN(VCI_USBCAN2, 0, 0, byref(vci_initconfig))

    if ret == STATUS_OK:
        print("VCI_InitCAN okk")
    else:
        print("VCI_InitCAN fail")

    ret = canDLL.VCI_StartCAN(VCI_USBCAN2, 0, 0)
    if ret == STATUS_OK:
        print("VCI_startCAN okk")
    else:
        print("VCI_StartCAN fail")
        canDLL.VCI_CloseDevice(VCI_USBCAN2, 0)

def convert_angle_to_position(desired_angle):
    target_position = int(desired_angle * 101 * 65536 / 360)  # Assuming conversion formula
    return target_position

def send_can_command(can_id, command, parameter):
    global canDLL
    global VCI_USBCAN2
    global STATUS_OK
    send = VCI_CAN_OBJ()

    send.ID = can_id
    send.SendType = 0
    send.RemoteFlag = 0
    send.ExternFlag = 0
    send.DataLen = 5

    parameter_bytes = parameter.to_bytes(4, byteorder='little', signed=True)
    for i in range(4):
        send.Data[i+1] = parameter_bytes[i]

    send.Data[0] = command

    if canDLL.VCI_Transmit(VCI_USBCAN2, 0, 0, byref(send), 1) == 1:
        print("CAN TX successful")
    else:
        print("CAN TX failed")
def hip_rot_clock_wise(target_angle):
        global current_angles
        global mode
        global motor_type
        global b
        for i in range(int(current_angles[4]), int(target_angle), 1):
            q_value = (i / 360.0 * 43)+b[0]  # Assuming 43 corresponds to 360 degrees            
            cmd.motorType = motor_type
            cmd.id=0
            cmd.mode = mode
            cmd.q = q_value
            cmd.dq = 0
            cmd.kp = 0.1
            cmd.kd = 0.01
            cmd.tau = 0.0
            serial.sendRecv(cmd, data)
            print('\n')
            print("q: " + str(data.q))
            print("dq: " + str(data.dq))
            print("temp: " + str(data.temp))
            print("merror: " + str(data.merror))
            print('\n')
            current_angles[4]=target_angle
def hip_rot_anti_wise(target_angle):
        global current_angles
        global mode
        global motor_type
        global b
        for i in range(int(current_angles[4]), int(target_angle), -1):
            q_value = (i / 360.0 * 43)+b[0]  # Assuming 43 corresponds to 360 degrees
            cmd.motorType = motor_type
            cmd.id=0
            cmd.mode = mode
            cmd.q = q_value
            cmd.dq = 0
            cmd.kp = 0.1
            cmd.kd = 0.01
            cmd.tau = 0.0
            serial.sendRecv(cmd, data)
            print('\n')
            print("q: " + str(data.q))
            print("dq: " + str(data.dq))
            print("temp: " + str(data.temp))
            print("merror: " + str(data.merror))
            print('\n')
            current_angles[4]=target_angle
def left_side_up_down_clock(target_angle):
        global current_angles
        global mode
        global motor_type
        global b
        for i in range(int(current_angles[0]), int(target_angle), 1):
            q_value = (i / 360.0 * 43)+b[2]  # Assuming 43 corresponds to 360 degrees            
            cmd.motorType = motor_type
            cmd.id=4
            cmd.mode = mode
            cmd.q = q_value
            cmd.dq = 0
            cmd.kp = 0.1
            cmd.kd = 0.01
            cmd.tau = 0.0
            serial.sendRecv(cmd, data)
            print('\n')
            print("q: " + str(data.q))
            print("dq: " + str(data.dq))
            print("temp: " + str(data.temp))
            print("merror: " + str(data.merror))
            print('\n')
            current_angles[0]=target_angle
def left_side_up_down_clock_anti(target_angle):
        global current_angles
        global mode
        global motor_type
        global b
        for i in range(int(current_angles[0]), int(target_angle), -1):
            q_value = (i / 360.0 * 43)+b[2]  # Assuming 43 corresponds to 360 degrees
            cmd.motorType = motor_type
            cmd.id=4
            cmd.mode = mode
            cmd.q = q_value
            cmd.dq = 0
            cmd.kp = 0.1
            cmd.kd = 0.01
            cmd.tau = 0.0
            serial.sendRecv(cmd, data)
            print('\n')
            print("q: " + str(data.q))
            print("dq: " + str(data.dq))
            print("temp: " + str(data.temp))
            print("merror: " + str(data.merror))
            print('\n') 
            current_angles[0]=target_angle
def left_side_clock(target_angle):
        global current_angles
        global mode
        global motor_type
        global b
        for i in range(int(current_angles[1]), int(target_angle), 1):
            q_value = (i / 360.0 * 43)+b[1]  # Assuming 43 corresponds to 360 degrees            
            cmd.motorType = motor_type
            cmd.id=2
            cmd.mode = mode
            cmd.q = q_value+1
            cmd.dq = 0
            cmd.kp = 0.1
            cmd.kd = 0.01
            cmd.tau = 0.0
            serial.sendRecv(cmd, data)
            print('\n')
            print("q: " + str(data.q))
            print("dq: " + str(data.dq))
            print("temp: " + str(data.temp))
            print("merror: " + str(data.merror))
            print('\n')
            current_angles[1]=target_angle
def left_side_anti(target_angle):
        global current_angles
        global mode
        global motor_type
        global b
        for i in range(int(current_angles[1]), int(target_angle), -1):
            q_value = (i / 360.0 * 43)+b[1]  # Assuming 43 corresponds to 360 degrees
            cmd.motorType = motor_type
            cmd.id=2
            cmd.mode = mode
            cmd.q = q_value
            cmd.dq = 0
            cmd.kp = 0.1
            cmd.kd = 0.01
            cmd.tau = 0.0
            serial.sendRecv(cmd, data)
            print('\n')
            print("q: " + str(data.q))
            print("dq: " + str(data.dq))
            print("temp: " + str(data.temp))
            print("merror: " + str(data.merror))
            print('\n')
            current_angles[1]=target_angle
def right_up_clock_wise(target_angle):
        global current_angles
        global mode
        global motor_type
        global b
        for i in range(int(current_angles[2]), int(target_angle), 1):
            q_value = (i / 360.0 * 43)+b[4]  # Assuming 43 corresponds to 360 degrees            
            cmd.motorType = motor_type
            cmd.id=6
            cmd.mode = mode
            cmd.q = q_value+1
            cmd.dq = 0
            cmd.kp = 0.1
            cmd.kd = 0.01
            cmd.tau = 0.0
            serial.sendRecv(cmd, data)
            print('\n')
            print("q: " + str(data.q))
            print("dq: " + str(data.dq))
            print("temp: " + str(data.temp))
            print("merror: " + str(data.merror))
            print('\n')
            current_angles[2]=target_angle
def right_down_anti_wise(target_angle):
        global current_angles
        global mode
        global motor_type
        global b
        for i in range(int(current_angles[2]), int(target_angle), -1):
            q_value = (i / 360.0 * 43)+b[4]  # Assuming 43 corresponds to 360 degrees
            cmd.motorType = motor_type
            cmd.id=6
            cmd.mode = mode
            cmd.q = q_value
            cmd.dq = 0
            cmd.kp = 0.1
            cmd.kd = 0.01
            cmd.tau = 0.0
            serial.sendRecv(cmd, data)
            print('\n')
            print("q: " + str(data.q))
            print("dq: " + str(data.dq))
            print("temp: " + str(data.temp))
            print("merror: " + str(data.merror))
            print('\n')
            current_angles[2]=target_angle
def right_side_clock_wise(target_angle):
        global current_angles
        global mode
        global motor_type
        global b
        for i in range(int(current_angles[3]), int(target_angle), 1):
            q_value = (i / 360.0 * 43)+b[3]  # Assuming 43 corresponds to 360 degrees            
            cmd.motorType = motor_type
            cmd.id=5
            cmd.mode = mode
            cmd.q = q_value
            cmd.dq = 0
            cmd.kp = 0.1
            cmd.kd = 0.01
            cmd.tau = 0.0
            serial.sendRecv(cmd, data)
            print('\n')
            print("q: " + str(data.q))
            print("dq: " + str(data.dq))
            print("temp: " + str(data.temp))
            print("merror: " + str(data.merror))
            print('\n')
            current_angles[3]=target_angle
def right_side_anti_wise(target_angle):
        global current_angles
        global mode
        global motor_type
        global b
        for i in range(int(current_angles[3]), int(target_angle), -1):
            q_value = (i / 360.0 * 43)+b[3]  # Assuming 43 corresponds to 360 degrees
            cmd.motorType = motor_type
            cmd.id=5
            cmd.mode = mode
            cmd.q = q_value
            cmd.dq = 0
            cmd.kp = 0.1
            cmd.kd = 0.01
            cmd.tau = 0.0
            serial.sendRecv(cmd, data)
            print('\n')
            print("q: " + str(data.q))
            print("dq: " + str(data.dq))
            print("temp: " + str(data.temp))
            print("merror: " + str(data.merror))
            print('\n')
            current_angles[3]=target_angle
def motor(i,pos):
    global current_angles
    if (i==0):                #left shoulder up down motor->unitree
        pos=-(pos)
        if pos >= current_angles[0]:
            left_side_up_down_clock(pos)
        elif pos <= current_angles[0]:
            left_side_up_down_clock_anti(pos)
    if (i==1):                   #left shoulder right left motor->unitree
        pos=-(pos)
        if pos >= current_angles[1]:
            left_side_clock(pos)
        elif pos <= current_angles[1]:
            left_side_anti(pos)
    if (i==2):                       #left elbow rot  motor->deffy
        pos=(pos)
        print(pos)
        target=convert_angle_to_position(pos)
        send_can_command(1,30,target)
    if (i==3):  
        pos=(pos)  
        print(pos)                    #left elbow up  motor->deffy
        target=convert_angle_to_position(pos)
        send_can_command(4,30,target)
    if (i==15):                        #left finger motor->servo
        pos=-(pos)
        if pos >= current_angles[4]:
            hip_rot_clock_wise(pos)
        elif pos <= current_angles[4]:
            hip_rot_anti_wise(pos)                      
    if (i==6):                        #right shoulder up down motor->unitree
        if pos >= current_angles[2]:
            right_up_clock_wise(pos)
        elif pos <= current_angles[2]:
            right_down_anti_wise(pos)
    if (i==7):                           #right shoulder right left motor->unitree
        pos=-(pos)
        if pos >= current_angles[3]:
            right_side_clock_wise(pos)
        elif pos <= current_angles[3]:
            right_side_anti_wise(pos)
    if (i==8):                           #right elbow rot  motor->deffy
        pos=-(pos)
        print(pos)
        target=convert_angle_to_position(pos)
        send_can_command(2,30,target)
    if (i==9):       
        pos=(pos)   
        print(pos)                  #right elbow up  motor->deffy
        target=convert_angle_to_position(pos)
        send_can_command(3,30,target)
                
def callback(msg):
        a=len(msg.data)
        for i in range(a):
            motor(i,msg.data[i])  
    
def main():
    motor_initialization()
    init_can()
    rospy.init_node("control")
    sub = rospy.Subscriber("joint_angle", Float32MultiArray, callback)
    rospy.spin()
if __name__ == "__main__":
    main()
