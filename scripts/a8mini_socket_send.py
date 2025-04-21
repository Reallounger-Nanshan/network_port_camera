#!/home/hxl228server20/anaconda3/envs/yolov10/bin/python3.9
import socket
import rospy
from network_port_camera.msg import ControlCommand, RotationPose


class A8Mini():
    def __init__(self):
        # Server address and port number
        self.host = rospy.get_param("~host")
        self.port = int(rospy.get_param("~port"))
        
        # The 16-bit CRC check code of a single byte
        self.mCRC16_Tables = [0, 4129, 8258, 12387, 16516, 20645, 24774, 28903, 33032, 37161, 41290, 45419, 49548,
            53677, 57806, 61935, 4657, 528, 12915, 8786, 21173, 17044, 29431, 25302, 37689, 33560, 45947, 41818,
            54205, 50076, 62463, 58334, 9314, 13379, 1056, 5121, 25830, 29895, 17572, 21637, 42346, 46411, 34088,
            38153, 58862, 62927, 50604, 54669, 13907, 9842, 5649, 1584, 30423, 26358, 22165, 18100, 46939, 42874,
            38681, 34616, 63455, 59390, 55197, 51132, 18628, 22757, 26758, 30887, 2112, 6241, 10242, 14371, 51660,
            55789, 59790, 63919, 35144, 39273, 43274, 47403, 23285, 19156, 31415, 27286, 6769, 2640, 14899, 10770,
            56317, 52188, 64447, 60318, 39801, 35672, 47931, 43802, 27814, 31879, 19684, 23749, 11298, 15363, 3168,
            7233, 60846, 64911, 52716, 56781, 44330, 48395, 36200, 40265, 32407, 28342, 24277, 20212, 15891, 11826,
            7761, 3696, 65439, 61374, 57309, 53244, 48923, 44858, 40793, 36728, 37256, 33193, 45514, 41451, 53516,
            49453, 61774, 57711, 4224, 161, 12482, 8419, 20484, 16421, 28742, 24679, 33721, 37784, 41979, 46042,
            49981, 54044, 58239, 62302, 689, 4752, 8947, 13010, 16949, 21012, 25207, 29270, 46570, 42443, 38312,
            34185, 62830, 58703, 54572, 50445, 13538, 9411, 5280, 1153, 29798, 25671, 21540, 17413, 42971, 47098,
            34713, 38840, 59231, 63358, 50973, 55100, 9939, 14066, 1681, 5808, 26199, 30326, 17941, 22068, 55628,
            51565, 63758, 59695, 39368, 35305, 47498, 43435, 22596, 18533, 30726, 26663, 6336, 2273, 14466, 10403,
            52093, 56156, 60223, 64286, 35833, 39896, 43963, 48026, 19061, 23124, 27191, 31254, 2801, 6864, 10931,
            14994, 64814, 60687, 56684, 52557, 48554, 44427, 40424, 36297, 31782, 27655, 23652, 19525, 15522, 11395,
            7392, 3265, 61215, 65342, 53085, 57212, 44955, 49082, 36825, 40952, 28183, 32310, 20053, 24180, 11923,
            16050, 3793, 7920]
        
        self.topic_pub_name = rospy.get_param("~topic_pub_name")
        self.camera_pose_pub = rospy.Publisher(self.topic_pub_name, RotationPose, queue_size = 1)


    def ComputeCheckBit(self, byte_str):
        # Initial
        crc = 0x0000
        data = bytearray(byte_str)
        # print("bytestr: ", byte_str)
        len1 = len(byte_str)
    
        # Calculate CRC check code
        # print("[ * ] crc: ", crc.to_bytes(2, "little"))
        for i in range(len1):
            cp = (crc >> 8) ^ data[i]
            crc = ((crc << 8) & 0xFFFF) ^ self.mCRC16_Tables[cp]
            # print("[", i, "] crc: ", crc.to_bytes(2, "little"))
    
        return crc.to_bytes(2, byteorder = "little")


    # Convert angle data from decimal to hexadecimal
    def AngleConversion(self, angles):
        angle_hex = b""
        for i in range(len(angles)):
           # Convert int to bytes
           angle_hex += angles[i].to_bytes(2, byteorder = "little", signed = True)
            
        return angle_hex


    def CommandConversion(self, command_msg):
        # Request the current working mode of the PTZ camera
        if command_msg.type == 0:
            information = b"\x55\x66\x01\x00\x00\x00\x00\x19\x5d\x57"
        
        # One click back
        elif command_msg.type == 1:
            information = b"\x55\x66\x01\x01\x00\x00\x00\x08\x01\xd1\x12"
        
        # Request PTZ camera attitude data
        elif command_msg.type == 2:
            information = b"\x55\x66\x01\x00\x00\x00\x00\x0d\xe8\x05"
        
        # Send control angle to PTZ camera
        elif command_msg.type == 3:
            yaw, pitch = command_msg.camera_rotation_pose.yaw, command_msg.camera_rotation_pose.pitch
            angle_hex = self.AngleConversion([yaw, pitch])
            data_bag = b"\x55\x66\x01\x04\x00\x00\x00\x0e" + angle_hex
            check_bit = self.ComputeCheckBit(data_bag)
            information = data_bag + check_bit
        
        return information


    # Publish PTZ camera attitude
    def SendCameraPose(self, pose_bytes):
        camera_pose = RotationPose()
        
        camera_pose.yaw = int.from_bytes(pose_bytes[:2], byteorder = "little", signed = True)
        camera_pose.pitch = int.from_bytes(pose_bytes[2:4], byteorder = "little", signed = True)
        camera_pose.roll = int.from_bytes(pose_bytes[4:], byteorder = "little", signed = True)
        
        print("yaw: {:.1f}  pitch: {:.1f}  roll: {:.1f}".format(camera_pose.yaw / 10, camera_pose.pitch / 10, camera_pose.roll / 10))
        
        self.camera_pose_pub.publish(camera_pose)


    def Run(self, command_msg):
        # Create a socket object
        client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        
        # Connect PTZ camera
        client_socket.connect((self.host, self.port))
        
        # Convert command to hexadecimal data
        message = self.CommandConversion(command_msg)
        print("Send message：{", message.hex(" "), "}")
        
        # Send message
        client_socket.sendall(message)

        # Received response
        response = client_socket.recv(1024)

        if command_msg.type == 2:
            self.SendCameraPose(response[-14:-8])

        # Print response message
        print("Response message：{", response.hex(" "), "}")
        
        # Close connection
        client_socket.close()
        
        
if __name__ == "__main__":
    rospy.init_node('a8mini_socket_send')

    a8mini = A8Mini()

    topic_pub_name = rospy.get_param("~topic_pub_name")
    rospy.Subscriber(topic_pub_name, ControlCommand, a8mini.Run)
    
    rospy.spin()
  
