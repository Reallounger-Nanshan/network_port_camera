#include <string.h>
#include <errno.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <ros/ros.h>
#include <a8mini_msgs/control_command.h>

#define RECV_BUUF_SIZE 64
#define SERVER_PORT 37260  // PTZ camera (server) port number
#define SERVER_IP "192.168.144.25"  // PTZ camera (server) IP


int main(int argc, char *argv[]){
    int sockfd, ret, i, recv_len;
    struct sockaddr_in send_addr, recv_addr;
    unsigned char send_buf[] = {0x55, 0x66, 0x01, 0x01, 0x00, 0x00, 0x00, 0x08, 0x01, 0xD1, 0x12};  // The corresponding function of the frame protocol (hexadecimal data)
    /*
    Request the current working mode of the PTZ camera:
    0x55, 0x66, 0x01, 0x00, 0x00, 0x00, 0x00, 0x19, 0x5D, 0x57
    
    One click back:
    0x55, 0x66, 0x01, 0x01, 0x00, 0x00, 0x00, 0x08, 0x01, 0xD1, 0x12
    
    Request PTZ camera attitude data:
    0x55, 0x66, 0x01, 0x00, 0x00, 0x00, 0x00, 0x0D, 0xE8, 0x05
    
    Send control angle -90 degrees (face down) to PTZ camera:
    0x55, 0x66, 0x01, 0x04, 0x00, 0x00, 0x00, 0x0E, 0x00, 0x00, 0xFF, 0xA6, 0x3B, 0x11
    */
    
    unsigned char recv_buf[RECV_BUUF_SIZE] = {0};
    
    /* Create a UDP socket
        AF_INET: IPV4 address
        SOCK_DGRAM: UDP protocol
        0: SelectedThe automatically default protocol for the type
    */
    if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
        perror("socket");
        exit(1);
    }
    
    /*
    Set the IP and port number of the PTZ camera
        sin_family: IPV4 address
        sin_addr.s_addr: PTZ camera IP
        sin_port: PTZ camera port number
    */
    memset(&send_addr, 0, sizeof(send_addr));
    send_addr.sin_family = AF_INET;
    send_addr.sin_addr.s_addr = inet_addr(SERVER_IP);
    send_addr.sin_port = htons(SERVER_PORT);
    
    /* Send frame data
        sockfd: Socket file descriptor
        send_buf: The first address in memory of the data to be sent
        sizeof(send_buf): Length of data to be sent
        0: Send flag, usually 0
        (struct sockaddr*)&send_addr: Structure pointer to the address (including IP address and port number) of the data receiver
        addr_len: The size of the receiver address structure
    */
    printf("Send HEX data\n");
    socklen_t addr_len = sizeof(struct sockaddr_in);
    if(sendto(sockfd, send_buf, sizeof(send_buf), 0, (struct sockaddr *)&send_addr, addr_len) < 0){
        perror("sendto");
        exit(1);
    }
    
    /* Receive the return data from the PTZ camera
        sockfd: Socket file descriptor
        recv_buf: The location in memory where the received data is stored
        RECV_BUUF_SIZE: Refers to the size of the buffer, i.e. the length of the maximum data expected to be received
        0: Receive flag, usually 0
        (struct sockaddr *)&recv_addr: The pointed structure will be filled with the address (including IP address and port number) of the data sender
        &addr_len: To the storage location, the structure size of src_addr and addrlen should be filled in before the call, and the actual size of the sender address will be filled in after the call
    */
    printf("Waiting for datas\n");
    recv_len = recvfrom(sockfd, recv_buf, RECV_BUUF_SIZE, 0, (struct sockaddr*)&recv_addr, &addr_len);
    printf("%d\n", recv_len);
    if (recv_len < 0) {
        perror("recvfrom");
        exit(1);
    }
    
    // Print the received data in hexadecimal form
    printf("Received HEX data: \n");
    for (int i = 0; i < recv_len; i++){
        printf("%02x ", recv_buf[i]);
    }
    printf("\n");
    
    // Close the socket
    close(sockfd);
    
    return 0;
}



void ControlCommandCallback(const a8mini_msgs::control_command::ConstPtr& command_msg){
    // Read control command
    
}

int main(int argc, char **argv){
    // Initialize ROS node
    ros::init(argc, argv, "a8mini_controller");
    
    // Create a node handle
    ros::NodeHandle nh;
    
    // Create a Subscriber
    ros::Subscriber control_command_sub = nh.subscribe("", 10, ControlCommandCallback);
    
    ros::spin();
    
    return 0;
}
