#!/home/hxl228server20/anaconda3/envs/yolov10/bin/python3.9

import serial
import struct
import rospy
from network_port_camera.msg import ControlCommand, RotationPose


class DoubleLight():
    def __init__(self):
        # Server address and port number
        self.usart = serial.Serial(port = '/dev/ttyUSB0', baudrate = 9600, timeout = 0.001)

        self.data_head = 0xff


    def CalculateParityBit(self, information):
        return sum(information[1:]) % 0x100


    def CommandConversion(self, command_msg):
        # Stop action
        if command_msg.type == 0:
            information = [self.data_head, 0x01, 0x00, 0x00, 0x00, 0x00]
            information.append(self.CalculateParityBit(information))
        
        # 90 pitch
        elif command_msg.type == 1:
            information = [self.data_head, 0x01, 0x00, 0x4d, 0x69, 0x78]
            information.append(self.CalculateParityBit(information))
        
        # 0 pitch
        elif command_msg.type == 2:
            information = [self.data_head, 0x01, 0x00, 0x4d, 0x00, 0x00]
            information.append(self.CalculateParityBit(information))

        # 38 yaw
        elif command_msg.type == 3:
            information = [self.data_head, 0x01, 0x00, 0x4b, 0x0e, 0xd8]
            information.append(self.CalculateParityBit(information))

        # 0 yaw
        elif command_msg.type == 4:
            information = [self.data_head, 0x01, 0x00, 0x4b, 0x00, 0x00]
            information.append(self.CalculateParityBit(information))

        # Set default point position
        elif command_msg.type == 5:
            information = [self.data_head, 0x01, 0x00, 0x03, 0x00]
            information.append(command_msg.default_point)
            information.append(self.CalculateParityBit(information))

        # Turn to default point position
        elif command_msg.type == 6:
            information = [self.data_head, 0x01, 0x00, 0x07, 0x00]
            information.append(command_msg.default_point)
            information.append(self.CalculateParityBit(information))
        
        # Set Zoom Position
        elif command_msg.type == 7:
            information = [self.data_head, 0x01, 0x00, 0x4f, 0xff, 0xff]
            information.append(self.CalculateParityBit(information))

        return information


    def Run(self, command_msg):
        if self.usart.isOpen():
            print("open success")

            message = self.CommandConversion(command_msg)

            send_data = struct.pack("%dB"%(len(message)), *message)
            print(send_data)

            self.usart.write(send_data) 
        else:
            print("open failed")
        
        
if __name__ == "__main__":
    rospy.init_node('double_light_socket_send')

    double_light = DoubleLight()

    topic_sub_name = rospy.get_param("~topic_sub_name")
    rospy.Subscriber(topic_sub_name, ControlCommand, double_light.Run)
    
    rospy.spin()
  
