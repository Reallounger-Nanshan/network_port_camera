#!/home/uav/anaconda3/envs/SceneMatch/bin/python3.8
import socket

class C10Pro():
    def __init__(self):
        # Server address and port number
        self.host = "192.168.144.108"
        self.port = 5000


    def Run(self):
        # Create a socket object
        client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        
        # Connect PTZ camera
        client_socket.connect((self.host, self.port))
        
        # Convert command to hexadecimal data
        message = b"#TPUG2wPTZ026C"
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
    c10pro = C10Pro()
    c10pro.Run()
