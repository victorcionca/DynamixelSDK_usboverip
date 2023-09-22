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
: bridge_ip_port(ip_port)
{
    strncpy(port_name_, usb_port, 100);
}

bool IPtoUsbBridge::setupIPSocket()
{
    struct sockaddr_in server_addr;

    bzero(&server_addr,  sizeof(server_addr));

    ip_socket_fd_ = socket(AF_INET, SOCK_STREAM, 0);
    if (ip_socket_fd_ < 0) return false;

    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(bridge_ip_port);
    server_addr.sin_addr.s_addr = htonl(INADDR_ANY);

    if (bind(ip_socket_fd_, (struct sockaddr*)&server_addr, sizeof(server_addr))){
        // TODO indicate error?
        return false;
    }

}

bool IPtoUsbBridge::setupUSBPort()
{
    /**
     * TODO
     * to start we set up the USB port with default (1M) bandwidth.
     * Later we can look at obtaining the bandwidth over IP.
     */
    int baudrate = B1000000;
    struct termios newtio;

    usb_socket_fd_ = open(port_name_, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (usb_socket_fd_ < 0)
    {
        printf("[PortHandlerLinux::SetupPort] Error opening serial port!\n");
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
}

void IPtoUsbBridge::read_ip_and_send_usb()
{
    // Ingress IP data is always framed
    uint8_t byte;
    uint16_t length, temp_length;
    uint8_t *packet;
    int result;
    result = read(ip_socket_fd_, &byte, 1);
    if (result == -1){
        // TODO analyse error? We know there is data available.
        return;
    }
    if (byte != PORT_HANDLER_IP_PKTSTART){
        // TODO this is a problem
        printf("Error: packet start of frame missing\n");
        return;
    }
    result = read(ip_socket_fd_, &length, 2);
    if (result != 2){
        printf("Error: reading packet length\n");
        return;
    }
    // Convert length back to host notation
    length = htons(length);
    temp_length = length;

    packet = (uint8_t*)malloc(length);
    uint8_t *cursor = packet;
    while (length > 0){
        result = read(ip_socket_fd_, cursor, length);
        if (result == -1) continue;
        length -= result;
        cursor += result;
    }
    // Read the packet termination
    result = read(ip_socket_fd_, &byte, 1);
    if (result == -1){
        // TODO
        return;
    }
    if (byte != PORT_HANDLER_IP_PKTEND){
        printf("Error: reading packet termination\n");
        return;
    }

    /* Now forward the packet over USB */
    printf("IP->USB: read %d bytes, sending to USB\n", temp_length);
    cursor = packet;
    while (temp_length > 0){
        result = write(usb_socket_fd_, cursor, temp_length);
        if (result == -1) continue; // TODO check for error
        temp_length -= result;
        cursor += result;
    }
    printf("IP->USB: successfully sent %d bytes to USB\n", temp_length);

    // Done!
}

void IPtoUsbBridge::read_usb_and_send_ip()
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
        if (result > 0){
            write(ip_socket_fd_, packet, result);
            printf("USB->IP: successfully sent %d bytes to IP\n", result);
        }
    }while (result > 0);
}

void IPtoUsbBridge::bridge()
{
    struct pollfd watched_fds[2];

    // Accept connections on the IP port
    struct sockaddr client_addr;
    socklen_t *client_addr_len;
    *client_addr_len = sizeof(client_addr);
    int client_sock;
    client_sock = accept4(ip_socket_fd_, &client_addr, client_addr_len,
                            SOCK_NONBLOCK); // Open non-blocking

    // Start listening for data on both IP and USB ports
    watched_fds[0].fd = ip_socket_fd_;
    watched_fds[0].events = POLLIN;
    watched_fds[1].fd = usb_socket_fd_;
    watched_fds[1].events = POLLIN;
    while (true){
        if (poll(watched_fds, 2, 10) > 0){
            // process events
            if (watched_fds[0].revents){
                read_ip_and_send_usb();
            }

            if (watched_fds[1].revents){
                read_usb_and_send_ip();
            }
        }
    }
}

void closeIPSocket();

void closeUSBPort();

void clearPort();

bool setBaudRate(const int baudrate);

int getBaudRate();

int getBytesAvailable();

void setPacketTimeout(uint16_t packet_length);

void setPacketTimeout(double msec);

bool isPacketTimeout();
