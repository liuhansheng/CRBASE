#pragma once
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <stdint.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <sys/uio.h>
#include <sys/time.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <fcntl.h>
#include <sys/epoll.h>
#include <sys/poll.h>
#include <vector>

#include <signal.h>


namespace COWA
{
namespace drivers
{
struct CanStamped : public can_frame
{
  uint64_t timestamp;
};
struct Canet_2e_u 
{
  struct{
    uint8_t can_length :4;
    uint8_t reserved :2;
    uint8_t rtr :1; //must be zero
    uint8_t frame_format :1;
  }can_info;
  uint8_t  id[4];
  uint8_t data[8];
};

class CANStream
{
public:
  virtual int open(const char *interface) = 0;
  virtual int recv(std::vector<CanStamped>& buffer, int timeout_msec = -1) = 0;
  virtual int send(std::vector<CanStamped>& buffer, int timeout_msec = -1) = 0;
};

class SocketCAN : public CANStream
{
private:
  int skt;
  struct ifreq ifr;

  struct msghdr msg;
  struct iovec iov;
  char ctrlmsg[CMSG_SPACE(sizeof(struct timeval) + 3 * sizeof(struct timespec) + sizeof(__u32))];
  struct can_frame frame;
  struct sockaddr_can addr;
public:
  int open(const char *interface);
  int recv(std::vector<CanStamped>& buffer, int timeout_msec = -1);
  int send(std::vector<CanStamped>& buffer, int timeout_msec = -1);
};

class CANOverUDP : public CANStream
{
private:
  int sock;
  int type;
  struct sockaddr addr_client;
  

public:
  int open(const char *interface);
  int recv(std::vector<CanStamped>& buffer, int timeout_msec = -1);
  int send(std::vector<CanStamped>& buffer, int timeout_msec = -1);
};


}; // namespace drivers

} // namespace COWA