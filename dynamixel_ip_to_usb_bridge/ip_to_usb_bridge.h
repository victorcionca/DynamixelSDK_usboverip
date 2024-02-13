////////////////////////////////////////////////////////////////////////////////
/// @file IP to USB bridge
/// @author Victor Cionca
////////////////////////////////////////////////////////////////////////////////

#ifndef DYNAMIXEL_SDK_INCLUDE_DYNAMIXEL_SDK_IP_USB_BRIDGE_H_
#define DYNAMIXEL_SDK_INCLUDE_DYNAMIXEL_SDK_IP_USB_BRIDGE_H_

#include <sys/types.h>
#include <sys/socket.h>
#include <netdb.h>

namespace dynamixel
{

#define PORT_HANDLER_IP_PKTSTART 0xAA
#define PORT_HANDLER_IP_PKTEND   0xBB

class IPtoUsbBridge
{
private:
    int ip_socket_fd_;
    int bridge_ip_port;
    struct addrinfo bridge_ip_;
    struct sockaddr_in client_ip;
    bool client_ip_known;
    int usb_socket_fd_;
    char port_name_[100]; // For compatibility
    double packet_start_time_;
    double packet_timeout_;
    double tx_time_per_byte;

    int read_ip_and_send_usb(int client_socket);
    void read_usb_and_send_ip(int client_socket);

    bool setupPort(const int cflag_baud);
    bool setCustomBaudrate(int speed);
    int getCFlagBaud(const int baudrate);

    double getCurrentTime();
    double getTimeSinceStart();

public:
    ////////////////////////////////////////////////////////////////////////////////
    /// @brief Initialise the bridge.
    /// @description Initialise the bridge for given ip port and usb port.
    ////////////////////////////////////////////////////////////////////////////////
    IPtoUsbBridge(int ip_port, const char *usb_port);

    ////////////////////////////////////////////////////////////////////////////////
    /// @brief The function that closes the port
    /// @description The function calls PortHandlerIP::closePort() to close the port.
    ////////////////////////////////////////////////////////////////////////////////
    virtual ~IPtoUsbBridge() { closeIPSocket();  closeUSBPort();}

    ////////////////////////////////////////////////////////////////////////////////
    /// @brief Setup IP socket
    /// @return status of the socket (true/false)
    ////////////////////////////////////////////////////////////////////////////////
    bool setupIPSocket();

    ////////////////////////////////////////////////////////////////////////////////
    /// @brief Setup USB port
    /// @return status of the port (true/false)
    ////////////////////////////////////////////////////////////////////////////////
    bool setupUSBPort(int cflag_baud);

    ////////////////////////////////////////////////////////////////////////////////
    /// @brief The function that closes the IP socket
    /// @description The function closes the IP socket
    ////////////////////////////////////////////////////////////////////////////////
    void closeIPSocket();

    ////////////////////////////////////////////////////////////////////////////////
    /// @brief The function that closes the USB port
    /// @description The function closes the USB port.
    ////////////////////////////////////////////////////////////////////////////////
    void closeUSBPort();

    ////////////////////////////////////////////////////////////////////////////////
    /// @brief The function that clears the port
    /// @description The function clears the port.
    ////////////////////////////////////////////////////////////////////////////////
    void clearPort();

    ////////////////////////////////////////////////////////////////////////////////
    /// @brief The function that sets baudrate into the port handler
    /// @description The function sets baudrate into the port handler.
    /// @param baudrate Baudrate
    /// @return false
    /// @return   when error was occurred during port opening
    /// @return or true
    ////////////////////////////////////////////////////////////////////////////////
    bool setBaudRate(const int baudrate);

    ////////////////////////////////////////////////////////////////////////////////
    /// @brief The function that returns current baudrate set into the port handler
    /// @description The function returns current baudrate set into the port handler.
    /// @return Baudrate
    ////////////////////////////////////////////////////////////////////////////////
    int getBaudRate();

    ////////////////////////////////////////////////////////////////////////////////
    /// @brief The function that checks how much bytes are able to be read from the port buffer
    /// @description The function checks how much bytes are able to be read from the port buffer
    /// @description and returns the number.
    /// @return Length of read-able bytes in the port buffer
    ////////////////////////////////////////////////////////////////////////////////
    int getBytesAvailable();

    ////////////////////////////////////////////////////////////////////////////////
    /// @brief Start bridging between IP and USB, forever
    /// @description This function doesn't end
    ////////////////////////////////////////////////////////////////////////////////
    void bridge();

    ////////////////////////////////////////////////////////////////////////////////
    /// @brief The function that sets and starts stopwatch for watching packet timeout
    /// @description The function sets the stopwatch by getting current time and the time of packet timeout with packet_length.
    /// @param packet_length Length of the packet expected to be received
    ////////////////////////////////////////////////////////////////////////////////
    void setPacketTimeout(uint16_t packet_length);

    ////////////////////////////////////////////////////////////////////////////////
    /// @brief The function that sets and starts stopwatch for watching packet timeout
    /// @description The function sets the stopwatch by getting current time and the time of packet timeout with msec.
    /// @param packet_length Length of the packet expected to be received
    ////////////////////////////////////////////////////////////////////////////////
    void setPacketTimeout(double msec);

    ////////////////////////////////////////////////////////////////////////////////
    /// @brief The function that checks whether packet timeout is occurred
    /// @description The function checks whether current time is passed by the time of packet timeout from the time set by PortHandlerIP::setPacketTimeout().
    ////////////////////////////////////////////////////////////////////////////////
    bool isPacketTimeout();
};
}

#endif

