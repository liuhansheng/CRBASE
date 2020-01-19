#include "can.hpp"
#include <signal.h>
#include <crmw/crmw.h>
#include <linux/net_tstamp.h>

using namespace COWA::drivers;

#define PF_CAN 29

int SocketCAN::open(const char *interface)
{
  CRINFO << "open can interface " << interface;

  if ((skt = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0)
  {
    CRFATAL << "socket init error";
    return 1;
  }

  /* receive all CAN frame */
  struct can_filter rfilter;
  rfilter.can_id = 0;
  rfilter.can_mask = 0;
  setsockopt(skt, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter));

  strcpy(ifr.ifr_name, interface);
  ioctl(skt, SIOCGIFINDEX, &ifr);

  const int timestamping_flags = SOF_TIMESTAMPING_SOFTWARE;
  if (setsockopt(skt, SOL_SOCKET, SO_TIMESTAMPING,
                  &timestamping_flags, sizeof(timestamping_flags)) < 0)
  {
    CRFATAL << "setsockopt SO_TIMESTAMPING is not supported by your Linux kernel";
    return 1;
  }
  addr.can_family = PF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;

  if (bind(skt, (struct sockaddr *)&addr, sizeof(addr)) < 0)
  {
    AFATAL << "socket bind error";
    return 1;
  }

  return 0;
}
int SocketCAN::recv(std::vector<CanStamped>& buffer, int timeout_msec)
{
  buffer.resize(1024);
  for(int i = 0; i < buffer.size(); ++i)
  {
    struct pollfd fds;
    memset(&fds, 0, sizeof(fds));
    fds.fd = skt;
    fds.events |= POLLIN;
    int poll_result = poll(&fds, 1, timeout_msec);
    if (poll_result <= 0 || ((uint32_t)fds.revents & (uint32_t)POLLIN) == 0)
    {
      buffer.resize(i);
      return buffer.size();;
    } 

    auto to_nano = [](struct timespec *ts) -> uint64_t { 
      return ts->tv_sec * (uint64_t)1000000000L + ts->tv_nsec; 
    };
    iov.iov_base = &frame;
    iov.iov_len = sizeof(frame);
    msg.msg_name = &addr;
    msg.msg_iov = &iov;
    msg.msg_iovlen = 1;
    msg.msg_control = &ctrlmsg;
    msg.msg_namelen = sizeof(addr);
    msg.msg_controllen = sizeof(ctrlmsg);
    msg.msg_flags = 0;
    auto nbytes = recvmsg(skt, &msg, 0);

    if (nbytes > 0)
    {
      auto& can = buffer[i];
      memcpy(&can, &frame, sizeof(frame));

      struct cmsghdr *cmsg = CMSG_FIRSTHDR(&msg);
      if (cmsg && cmsg->cmsg_type == SO_TIMESTAMPING &&  (cmsg->cmsg_level == SOL_SOCKET))
      {
        struct timespec *stamp = (struct timespec *)CMSG_DATA(cmsg);
        can.timestamp = to_nano(&stamp[0]);
      }
      else
      {
        CREVERY(100000, CRWARN << "CAN not suport raw socket time");
        can.timestamp = COWA::Time::Now().ToNanosecond(); 
      }
    }
  }
  return buffer.size();
}
int SocketCAN::send(std::vector<CanStamped>& buffer, int timeout_msec)
{
  for(int i = 0; i < buffer.size(); ++i)
  {
    struct pollfd fds;
    memset(&fds, 0, sizeof(fds));
    fds.fd = skt;
    fds.events |= POLLOUT;

    const int poll_result = poll(&fds, 1, -1);
    if ((poll_result == -1) || ((uint32_t)fds.revents & (uint32_t)POLLOUT) == 0)
      return -1;

    if (poll_result == 0)
      return -2;

    const ssize_t nbytes = write(skt, &buffer[i], sizeof(can_frame));
    if (nbytes < 0)
      return -1;
  }

  return 0;
}


int CANOverUDP::open(const char *interface)
{
  sock = socket(PF_INET, SOCK_DGRAM, 0);
  if (sock < 0)
    AFATAL << "socket() error";

  std::string str = interface;

  int port = 0;
  type = 0;
  int index = str.find(":");
  if(index != -1)
  {
    std::string p = str.substr(0, index + 1);
    std::string t = str.substr(index + 1, 1);
    port = atoi(p.c_str());
    type = atoi(t.c_str());
    CRTRACE<< "port: " << p << " type: " << t;
  }
  else
    CRERROR << "UDP CAN CONFIG ERROR!";
  
  struct sockaddr_in serv_addr;
  memset(&serv_addr, 0, sizeof(serv_addr));
  serv_addr.sin_family = AF_INET;
  serv_addr.sin_addr.s_addr = htonl(INADDR_ANY);
  serv_addr.sin_port = htons(port);

  if (bind(sock, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0)
    AFATAL << "bind() error";
  std::vector<CanStamped> buffer;
  
  for(int tries = 0; tries < 50; ++tries)
  {
    if(recv(buffer, 100) >= 1)
      return 0;
  }
  CRFATAL << "CANOverUDP should get frame first";
}
int CANOverUDP::recv(std::vector<CanStamped>& buffer, int ms_timeout)
{
  struct timeval time;  
  time.tv_sec = 0; 
  if(ms_timeout >= 0)
      time.tv_usec = ms_timeout * 1000;
  else
      time.tv_sec = 10; 
  fd_set fs;    
  FD_ZERO(&fs);    
  FD_SET(sock, &fs);   
  int ret = select(sock + 1, &fs, NULL, NULL, &time);
  if(ret <= 0)
    return ret;

  buffer.resize(2048);
  std::vector<Canet_2e_u> canet;
  if(type == 1)
    canet.resize(2048);
  time.tv_usec = 0;
  for(int i = 0; i < buffer.size(); )
  {
    int size = 0;

    FD_ZERO(&fs);    
    FD_SET(sock, &fs);   
    if(select(sock + 1, &fs, NULL, NULL, &time) > 0)
    {
      int len = sizeof(addr_client); 
      if(type == 0)
        size = recvfrom(sock, &buffer[i], sizeof(CanStamped) * (buffer.size() - i), 
                            0, (struct sockaddr *)&addr_client, (socklen_t *)&len);
      else if(type == 1)
        size = recvfrom(sock, &canet[i], sizeof(Canet_2e_u) * (canet.size() - i), 
                            0, (struct sockaddr *)&addr_client, (socklen_t *)&len);
    }
    if(size > 0)
    {
      if(type == 0)
        i += size / sizeof(CanStamped);
      else if(type == 1)
        i += size / sizeof(Canet_2e_u);
    }
    else
    {
      if(type == 1)
      {
        for(int j = 0; j < i; j++)
        {
          buffer[j].can_dlc = 8;
          uint32_t id = int(canet[j].id[0]) << 24 | int(canet[j].id[1]) << 16 | int(canet[j].id[2]) *256 | int(canet[j].id[3]);
          buffer[j].can_id = id;
          buffer[j].timestamp = COWA::Time::Now().ToNanosecond();
          memcpy(buffer[j].data, canet[j].data, 8);
        }
      }
      buffer.resize(i);
      return buffer.size();
    }
    
  }  
  return 0;
}
int CANOverUDP::send(std::vector<CanStamped>& buffer, int timeout_msec)
{
  int len = sizeof(addr_client);
  if(type == 0)
    return sendto(sock, &buffer[0], sizeof(CanStamped) * buffer.size(), 0, (struct sockaddr *)&addr_client, len);
  else if(type == 1)
  {
    std::vector<Canet_2e_u> canet;
    canet.resize(buffer.size());
    for(int i = 0; i < buffer.size(); i++)
    {
      canet[i].can_info.can_length = buffer[i].can_dlc;
      canet[i].can_info.frame_format = 0;
      canet[i].can_info.rtr = 0;
      canet[i].id[0] = buffer[i].can_id >> 24 & 0xFF;
      canet[i].id[1] = buffer[i].can_id >> 16 & 0xFF;
      canet[i].id[2] = buffer[i].can_id >> 8 & 0xFF;
      canet[i].id[3] = buffer[i].can_id & 0xFF;
      memcpy(canet[i].data, buffer[i].data, 8);
    }
    return sendto(sock, &canet[0], sizeof(Canet_2e_u) * canet.size(), 0, (struct sockaddr *)&addr_client, len);
  }
}

