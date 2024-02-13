/*******************************************************************************
* Copyright 2017 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Port handler that uses IP sockets instead of USB communication */
/* Author: Victor Cionca */

#if defined(__linux__)

#include <stdlib.h>
#include <stdio.h>
#include <fcntl.h>
#include <string.h>
#include <unistd.h>
#include <termios.h>
#include <time.h>
#include <sys/time.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <linux/serial.h>
#include <errno.h>

#include "port_handler_ip.h"

#define LATENCY_TIMER  16  // msec (USB latency timer)
                           // You should adjust the latency timer value. From the version Ubuntu 16.04.2, the default latency timer of the usb serial is '16 msec'.
                           // When you are going to use sync / bulk read, the latency timer should be loosen.
                           // the lower latency timer value, the faster communication speed.

                           // Note:
                           // You can check its value by:
                           // $ cat /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
                           //
                           // If you think that the communication is too slow, type following after plugging the usb in to change the latency timer
                           //
                           // Method 1. Type following (you should do this everytime when the usb once was plugged out or the connection was dropped)
                           // $ echo 1 | sudo tee /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
                           // $ cat /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
                           //
                           // Method 2. If you want to set it as be done automatically, and don't want to do above everytime, make rules file in /etc/udev/rules.d/. For example,
                           // $ echo ACTION==\"add\", SUBSYSTEM==\"usb-serial\", DRIVER==\"ftdi_sio\", ATTR{latency_timer}=\"1\" > 99-dynamixelsdk-usb.rules
                           // $ sudo cp ./99-dynamixelsdk-usb.rules /etc/udev/rules.d/
                           // $ sudo udevadm control --reload-rules
                           // $ sudo udevadm trigger --action=add
                           // $ cat /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
                           //
                           // or if you have another good idea that can be an alternatives,
                           // please give us advice via github issue https://github.com/ROBOTIS-GIT/DynamixelSDK/issues

using namespace dynamixel;

PortHandlerIP::PortHandlerIP(const char *port_name)
  : socket_fd_(-1),
  socket_open(false),
  baudrate_(DEFAULT_BAUDRATE_), // For compatibility
    packet_start_time_(0.0),
    packet_timeout_(0.0),
    tx_time_per_byte(0.0)
{
  buffer_length = 0;
  is_using_ = false;
  setPortName(port_name);
}

bool PortHandlerIP::openPort()
{
  return setBaudRate(baudrate_);
}

void PortHandlerIP::closePort()
{
  if(socket_fd_ != -1)
    close(socket_fd_);
  socket_fd_ = -1;
  socket_open = false;
}

void PortHandlerIP::clearPort()
{
  // No equivalent for IP
  //tcflush(socket_fd_, TCIFLUSH);
}

void PortHandlerIP::setPortName(const char *port_name)
{
  struct addrinfo hints;
  struct addrinfo *addrlist; // To store the output of getaddrinfo
  strcpy(port_name_, port_name);
  // We consider the port_name to be the address of the bridge
  // First split the port_name into IP and port number at ':'
  char *port_number = strchr(port_name_, ':');
  // TODO: validate port_name in the constructor because this doesn't return
  if (!port_number){
    return;
  }
  *port_number = 0; // Terminate the IP address
  port_number ++;
  memset(&hints, 0, sizeof(struct addrinfo));
  hints.ai_family = AF_INET;
  hints.ai_socktype = SOCK_STREAM;
  hints.ai_flags = 0;
  hints.ai_protocol = 0;

  if (getaddrinfo(port_name_, port_number, &hints, &addrlist)){
    printf("[PortHandlerIP::setPortName] Error parsing bridge address\n");
    return;
  }
  bridge_ = *addrlist;

  // Put back the ':' in the port name
  port_number --;
  *port_number = ':';
}

char *PortHandlerIP::getPortName()
{
  return port_name_;
}

// TODO: baud number ??
bool PortHandlerIP::setBaudRate(const int baudrate)
{
  // If the socket is already open just ignore
  if (socket_open) return true;

  // Otherwise open the socket by calling setupPort
  return setupPort(baudrate);
}

int PortHandlerIP::getBaudRate()
{
  return baudrate_;
}

int PortHandlerIP::getBytesAvailable()
{
  // TODO: this probably stays the same
  //printf("[PortHandlerIP::getBytesAvailable]\n");
  int bytes_available;
  ioctl(socket_fd_, FIONREAD, &bytes_available);
  return bytes_available;
}

int PortHandlerIP::readPort(uint8_t *packet, int length)
{
  // TODO - this all needs to be moved to the bridge.
  int result;
  int to_read = length;
  if (buffer_length < to_read){
    to_read = buffer_length;
  }

  // copy from buffer into packet to_read bytes
  // advace packet by to_read bytes
  for (int i=0;i<to_read;i++){
    *packet = read_buffer[i];
    packet++;
  }
  // move buffer contents back by to_read bytes
  for (int i=0;i<buffer_length-to_read;i++){
    read_buffer[i] = read_buffer[to_read+i];
  }
  buffer_length -= to_read;

  int remaining = length - to_read;
  if (remaining > 0) {
    // read from socket into buffer 11B
    result = read(socket_fd_, read_buffer, 11);
    if (result == -1) {
      if (errno == EAGAIN || errno == EWOULDBLOCK) {
        result = 0;
        remaining = 0;
      } else {
        perror("reading: ");
        return -1;
      }
    } else {
      buffer_length = result;
      //printf("Read %dB: ", result);
      //for (int i = 0; i < result; i++)
      //  printf("%02x ", read_buffer[i]);
      //printf("\n");
      // copy from buffer into packet min(remaining, result)
      // move buffer contents back by min(remaining, result)
      if (result < remaining) {
        remaining = result;
      }
      for (int i = 0; i < remaining; i++) {
        *packet = read_buffer[i];
        packet++;
      }
      // move buffer contents back by to_read bytes
      for (int i = 0; i < buffer_length - remaining; i++) {
        read_buffer[i] = read_buffer[remaining + i];
      }
      buffer_length -= remaining;
    }
  }
  return to_read + remaining;
}

int PortHandlerIP::writePort(uint8_t *packet, int length)
{
  // This function changes to include he framing of the USB packet
  int result = 0;
  //printf("Writing\n");
  uint16_t new_length = htons(length);
  uint8_t *buf = (uint8_t*)malloc(length+2+2);
  buf[0] = PORT_HANDLER_IP_PKTSTART;
  ((uint16_t*)(buf+1))[0] = new_length;
  memcpy(buf+3, packet, length);
  buf[3+length] = PORT_HANDLER_IP_PKTEND;
  result = send(socket_fd_, buf, length+2+2, 0);
  free(buf);
  return result-4; // account for 4 framing bytes
}

void PortHandlerIP::setPacketTimeout(uint16_t packet_length)
{
  // maintained for compatibility
  packet_start_time_  = getCurrentTime();
  packet_timeout_     = (tx_time_per_byte * (double)packet_length) + (LATENCY_TIMER * 2.0) + 2.0;
}

void PortHandlerIP::setPacketTimeout(double msec)
{
  // maintained for compatibility
  packet_start_time_  = getCurrentTime();
  packet_timeout_     = msec;
}

bool PortHandlerIP::isPacketTimeout(int flag)
{
  // maintained for compatibility
  //printf("Checking timeout %d\n", flag);
  if(getTimeSinceStart() > packet_timeout_)
  {
    packet_timeout_ = 0;
    return true;
  }
  return false;
}

double PortHandlerIP::getCurrentTime()
{
	struct timespec tv;
	clock_gettime(CLOCK_REALTIME, &tv);
	return ((double)tv.tv_sec * 1000.0 + (double)tv.tv_nsec * 0.001 * 0.001);
}

double PortHandlerIP::getTimeSinceStart()
{
  double time;

  time = getCurrentTime() - packet_start_time_;
  if(time < 0.0)
    packet_start_time_ = getCurrentTime();

  return time;
}

/**
 * Create the socket and connect to server
 * Note: This is a private function called from setBaudrate
 */
bool PortHandlerIP::setupPort(int cflag_baud)
{
  socket_fd_ = socket(AF_INET, SOCK_DGRAM | SOCK_NONBLOCK, 0);
  if(socket_fd_ < 0)
  {
    printf("[PortHandlerIP::SetupPort] Error opening serial port!\n");
    return false;
  }

  // Connect to the server
  if (connect(socket_fd_, bridge_.ai_addr, bridge_.ai_addrlen) == -1)
  {
    printf("[PortHandlerIP::SetupPort] Error connecting to server\n");
    return false;
  }

  socket_open = true;
  tx_time_per_byte = (1000.0 / (double)baudrate_) * 10.0;
  printf("[PortHandlerIP::SetupPort] Successfully connected\n");
  return true;
}

bool PortHandlerIP::setCustomBaudrate(int speed)
{
  // Not used and private so can be removed
  return true;
  // try to set a custom divisor
  struct serial_struct ss;
  if(ioctl(socket_fd_, TIOCGSERIAL, &ss) != 0)
  {
    printf("[PortHandlerIP::SetCustomBaudrate] TIOCGSERIAL failed!\n");
    return false;
  }

  ss.flags = (ss.flags & ~ASYNC_SPD_MASK) | ASYNC_SPD_CUST;
  ss.custom_divisor = (ss.baud_base + (speed / 2)) / speed;
  int closest_speed = ss.baud_base / ss.custom_divisor;

  if(closest_speed < speed * 98 / 100 || closest_speed > speed * 102 / 100)
  {
    printf("[PortHandlerIP::SetCustomBaudrate] Cannot set speed to %d, closest is %d \n", speed, closest_speed);
    return false;
  }

  if(ioctl(socket_fd_, TIOCSSERIAL, &ss) < 0)
  {
    printf("[PortHandlerIP::SetCustomBaudrate] TIOCSSERIAL failed!\n");
    return false;
  }

  tx_time_per_byte = (1000.0 / (double)speed) * 10.0;
  return true;
}

int PortHandlerIP::getCFlagBaud(int baudrate)
{
  // TODO: private function, not used so will probably go
  switch(baudrate)
  {
    case 9600:
      return B9600;
    case 19200:
      return B19200;
    case 38400:
      return B38400;
    case 57600:
      return B57600;
    case 115200:
      return B115200;
    case 230400:
      return B230400;
    case 460800:
      return B460800;
    case 500000:
      return B500000;
    case 576000:
      return B576000;
    case 921600:
      return B921600;
    case 1000000:
      return B1000000;
    case 1152000:
      return B1152000;
    case 1500000:
      return B1500000;
    case 2000000:
      return B2000000;
    case 2500000:
      return B2500000;
    case 3000000:
      return B3000000;
    case 3500000:
      return B3500000;
    case 4000000:
      return B4000000;
    default:
      return -1;
  }
}

#endif
