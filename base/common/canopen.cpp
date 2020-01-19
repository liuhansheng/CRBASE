#include "canopen.hpp"
#include <crmw/crmw.h>

using namespace COWA::drivers;

int CANOPEN::do_write(uint8_t slave, uint16_t index, uint8_t subindex)
{
  CRINFO << "Write: " << std::hex << index << ":" << (uint32_t)subindex;
  auto send_frame = frame;
  for (int i = 0; i < 3; ++i)
  {
    can->send(send_frame);

    usleep(20 * 1000);
    
    int size;
    do {
      can->recv(frame, 1);
      for(int i = 0; i < frame.size(); ++i)
      {
        uint16_t _cmd = frame[i].data[0];
        uint16_t _index = frame[i].data[1] + frame[i].data[2] * 0x0100;
        uint8_t _subidnex = frame[i].data[3];
        uint32_t _error;
        memcpy(&_error, frame[i].data + 4, 4);
        if (frame[i].can_id == 0x580 + slave && _index == index && _subidnex == subindex)
        {
          if (_cmd == 0x60)
          {
            CRINFO << std::hex << index << ":" << subindex << "write OK";
            return 0;
          }
          else if (_cmd == 0x80)
          {
            CRERROR << std::hex << index << ":" << subindex << "write fail: " << _error;
            return _error;
          }
          else
          {
            CRERROR << std::hex << index << ":" << subindex << "datagram error, " << _cmd;
            return -1;
          }
        }
      }
    } while(size > 0);
  }
}