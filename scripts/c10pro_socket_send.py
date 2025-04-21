#!/home/uav/anaconda3/envs/SceneMatch/bin/python3.8
import socket
import rospy
from std_msgs.msg import Int8


class C10Pro():
    def __init__(self):
        # Server address and port number
        self.host = rospy.get_param("~host")
        self.port = int(rospy.get_param("~port"))
        

    def CommandConversion(self, command_msg):
        # Head down
        if command_msg.data == 0:
            information = b"#TPUG2wPTZ026C"

        return information


    def Run(self, command_msg):
        # Create a socket object
        # client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        
        # Connect PTZ camera
        client_socket.connect((self.host, self.port))
        
        # Convert command to hexadecimal data
        message = self.CommandConversion(command_msg)
        print("Send message：{", message.hex(" "), "}")
        
        # Send message
        client_socket.sendall(message)

        # Received response
        response = client_socket.recv(1024)

        # Print response message
        print("Response message：{", response.hex(" "), "}")
        
        # Close connection
        client_socket.close()
        
        
if __name__ == "__main__":
    rospy.init_node('a8mini_socket_send')

    c10pro = C10Pro()

    topic_sub_name = rospy.get_param("~topic_sub_name")
    rospy.Subscriber(topic_sub_name, Int8, c10pro.Run)
    
    rospy.spin()
  
