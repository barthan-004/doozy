import rospy 
from std_msgs.msg import Float32MultiArray
import ctypes
from ctypes import*
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
init_can()
i=0
while (1):
    angle=int(input())
    for i in range(angle):
        angle=-(angle)
        tar=convert_angle_to_position(angle)
        send_can_command(4,25,tar)
