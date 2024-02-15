#include "ip_to_usb_bridge.h"
#include <string.h>
#include <termios.h>
#include <fcntl.h>
#include <stdio.h>
#include <sys/time.h>
#include <unistd.h>
#include <poll.h>
#include <stdlib.h>

using namespace dynamixel;

#define USB_MIN_LENGTH 11

IPtoUsbBridge::IPtoUsbBridge(int ip_port, const char *usb_port)
: bridge_ip_port(ip_port),
  client_ip_known(false)
{
    strncpy(port_name_, usb_port, 100);
}

bool IPtoUsbBridge::setupIPSocket()
{
    struct sockaddr_in server_addr;

    bzero(&server_addr,  sizeof(server_addr));

    ip_socket_fd_ = socket(AF_INET, SOCK_DGRAM | SOCK_NONBLOCK, 0);
    if (ip_socket_fd_ < 0) return false;

    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(bridge_ip_port);
    server_addr.sin_addr.s_addr = htonl(INADDR_ANY);

    if (bind(ip_socket_fd_, (struct sockaddr*)&server_addr, sizeof(server_addr))){
        // TODO indicate error?
        printf("[Bridge::setupIPSocket] Error binding\n");
        return false;
    }

    return true;
}

bool IPtoUsbBridge::setupUSBPort(int cflag_baud)
{
    /**
     * TODO
     * to start we set up the USB port with default (1M) bandwidth.
     * Later we can look at obtaining the bandwidth over IP.
     */
    int baudrate = cflag_baud;
    struct termios newtio;
    closeUSBPort();
    printf("[Bridge::setupUSB] Baudrate set to %x\n", baudrate);

    usb_socket_fd_ = open(port_name_, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (usb_socket_fd_ < 0)
    {
        printf("[Bridge::setupUSB] Error opening serial port %s!\n",
                port_name_);
        perror("[Bridge::setupUSB]");
        return false;
    }

    bzero(&newtio, sizeof(newtio)); // clear struct for new port settings

    newtio.c_cflag = baudrate | CS8 | CLOCAL | CREAD;
    newtio.c_iflag = IGNPAR;
    newtio.c_oflag = 0;
    newtio.c_lflag = 0;
    newtio.c_cc[VTIME] = 0;
    newtio.c_cc[VMIN] = 0;

    // clean the buffer and activate the settings for the port
    tcflush(usb_socket_fd_, TCIFLUSH);
    tcsetattr(usb_socket_fd_, TCSANOW, &newtio);

    tx_time_per_byte = (1000.0 / (double)1000000) * 10.0; // TODO baudrate hardcoded

    return true;
}

int IPtoUsbBridge::read_ip_and_send_usb(int client_socket)
{
    // Ingress IP data is always framed
    uint8_t byte;
    uint16_t length, temp_length;
    uint8_t packet[100];
    int result;
    printf("[Bridge::read_ip]\n");
    // Read up to 100B -- TODO double check that this sufficient
    if (client_ip_known){
        result = recv(client_socket, packet, 100, 0);
    }else{
        socklen_t addr_len = sizeof(client_ip);
        result = recvfrom(client_socket, packet, 100, 0, 
                        (struct sockaddr*)&client_ip,
                        &addr_len);
        char host[NI_MAXHOST], service[NI_MAXSERV];
        int s = getnameinfo((struct sockaddr*)&client_ip, addr_len,
                            host, NI_MAXHOST, service, NI_MAXSERV,
                            NI_NUMERICSERV);
        if (s == 0){
            printf("[Bridge] Connection from %s:%s\n", host, service);
        }
        client_ip_known = true;
    }
    if (result == -1){
        // TODO analyse error? We know there is data available.
        printf("[Bridge::read_ip] error reading buffer\n");
        perror("[Bridge::read_ip]");
        return -1;
    }
    printf("[Bridge::read_ip] Received %dB: ", result);
    for (int i=0;i<result;i++) printf("%02x ", packet[i]);printf("\n");
    // Check the structure
    if (packet[0] != PORT_HANDLER_IP_PKTSTART){
        // TODO this is a problem
        printf("Error: packet start of frame missing\n");
        return -1;
    }
    // Get length of the USB payload
    length = htons(((uint16_t*)(packet+1))[0]);

    // Check if the packet is a "closing socket" packet
    if (length == 8){
        int closing = 1;
        for (int i=0;i<length;i++){
            if (packet[3+i] != 0xAA){
                closing = 0;
                break;
            }
        }
        if (closing){
            printf("Client closing\n");
            client_ip_known = 0;
            return 0;
        }
    }
    printf("[Bridge::read_ip] USB payload length is %d\n", length);

    /* Now forward the packet over USB */
    printf("[Bridge::read_ip] Sending to USB\n");
    
    uint8_t *cursor = packet+3;
    temp_length = length;
    while (temp_length > 0){
        result = write(usb_socket_fd_, cursor, temp_length);
        if (result == -1){
            perror("[Bridge::readIP] write to USB error\n");
            continue; // TODO check for error
        }
        temp_length -= result;
        cursor += result;
    }
    printf("[Bridge::read_ip] Successfully sent %d bytes to USB\n", temp_length);

    // Done!
    return 0;
}

void IPtoUsbBridge::read_usb_and_send_ip(int client_socket)
{
    // Read as much as we can from the USB and send over IP.
    // The protocol layer above will take care of framing, etc.
    int result;
    uint8_t packet[USB_MIN_LENGTH];
    result = 0;
    do{
        result = read(usb_socket_fd_, packet, USB_MIN_LENGTH);
        // Forward over IP
        printf("USB->IP: read %d bytes from USB\n", result);
        for (int i = 0; i < result; i++)
            printf("%02x ", packet[i]);
        printf("\n");
        if (result > 0){
            int ip_res = 0;
            ip_res = sendto(client_socket, packet, result, 0,
                            (struct sockaddr*)&client_ip,
                            sizeof(client_ip));
            if (ip_res == -1) perror("USB->IP ");
            else
            printf("USB->IP: successfully sent %d (%d) bytes to IP\n",
                    result, ip_res);
        }
    }while (result > 0);
}

void IPtoUsbBridge::bridge()
{
    struct pollfd watched_fds[2];

    // Start listening for data on both IP and USB ports
    watched_fds[0].fd = ip_socket_fd_;
    watched_fds[0].events = POLLIN;
    watched_fds[1].fd = usb_socket_fd_;
    watched_fds[1].events = POLLIN;
    while (true){
        if (poll(watched_fds, 2, 10) > 0){ // Wait for 10ms
            // process events
            printf("[Bridge::bridge] data available\n");
            if (watched_fds[0].revents){
                printf("[Bridge::bridge] on ip data\n");
                if (read_ip_and_send_usb(ip_socket_fd_) < 0) break;
                printf("All good\n");
            }

            if (watched_fds[1].revents){
                printf("[Bridge::bridge] on usb data\n");
                read_usb_and_send_ip(ip_socket_fd_);
                printf("All good\n");
            }
        }
    }
}

void IPtoUsbBridge::closeIPSocket(){
    close(ip_socket_fd_);
}

void IPtoUsbBridge::closeUSBPort(){
    if (usb_socket_fd_ != -1)
        close(usb_socket_fd_);
    usb_socket_fd_ = -1;
}

void clearPort();

bool setBaudRate(const int baudrate);

int getBaudRate();

int getBytesAvailable();

void setPacketTimeout(uint16_t packet_length);

void setPacketTimeout(double msec);

bool isPacketTimeout();

int main(){
    // TODO - maybe pass the ip and usb ports as cmd line arguments?
    auto bridge = new IPtoUsbBridge(6666, "/dev/ttyDXL");
    if (!bridge->setupIPSocket()){
        printf("[Bridge::SetupIPSocket] error setting up IP socket\n");
        return -1;
    }
    if (!bridge->setupUSBPort(B57600)){
        printf("[Bridge::SetupUSBPort] error setting up USB comms\n");
        //delete bridge;
        //return -1;
    }
    if (!bridge->setupUSBPort(B1000000)) {
        printf("[Bridge::SetupUSBPort] error setting up USB comms\n");
        //delete bridge;
        //return -1;
    }
    bridge->bridge();

    return 0;
}