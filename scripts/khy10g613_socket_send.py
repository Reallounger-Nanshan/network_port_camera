#!/home/hxl228server20/anaconda3/envs/yolov10/bin/python3.9
import socket
import rospy
from std_msgs.msg import Int8


class KHY10G613():
    def __init__(self):
        # Server address and port number
        self.host = rospy.get_param("~host")
        self.port = int(rospy.get_param("~port"))
    

    def CommandConversion(self, command_msg):
        # Switch picture-in-picture mode
        if command_msg.data == 0:
            information = b"#TPUD2wPIP0A63"
        
        # Thermal image: gray-scale
        elif command_msg.data == 1:
            information = b"#TPUD2wIMG0046"
        
        # Thermal image: lava
        elif command_msg.data == 2:
            information = b"#TPUD2wIMG0147"

        # Thermal image: iron oxide red
        elif command_msg.data == 3:
            information = b"#TPUD2wIMG0248"

        # Thermal image: aurora borealis
        elif command_msg.data == 4:
            information = b"#TPUD2wIMG0349"

        # One-click return to middle
        elif command_msg.data == 5:
            information = b"#TPUG2wPTZ056F"

        # 
        elif command_msg.data == 6:
            information = b"#tpUG6wGIY34BC1E9B"

        # 
        elif command_msg.data == 7:
            information = b"#tpUG6wGIYDCD81EB2"

        # The positive direction is 90 degrees
        # Hoisting is downward, conversely upward
        elif command_msg.data == 8:
            information = b"#tpUG6wGIP23281E75"

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

    khy10g613 = KHY10G613()

    topic_sub_name = rospy.get_param("~topic_sub_name")
    rospy.Subscriber(topic_sub_name, Int8, khy10g613.Run)
    
    rospy.spin()
  
