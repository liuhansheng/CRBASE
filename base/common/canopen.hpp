#pragma once
#include "can.hpp"
#include <memory>

namespace COWA
{
namespace drivers
{

class CANOPEN
{
private:
    std::shared_ptr<CANStream> can;
    std::vector<CanStamped> frame;
    int do_write(uint8_t slave, uint16_t index, uint8_t subindex);

public:
    CANOPEN(std::shared_ptr<CANStream> _can) : can(_can) {}
    template <class T>
    int ContrustWrite(CanStamped* frame, uint8_t slave, uint16_t index, uint8_t subindex, T t, uint8_t cmd = 0)
    {
        uint8_t *data = frame[0].data;
        frame[0].can_id = slave + 0x600;
        frame[0].can_dlc = 8;
        memset(data, 0, 8);
        if (cmd == 0)
        {
            uint8_t cmd[] = {0x2f, 0x2b, 0x27, 0x23};
            data[0] = cmd[sizeof(T) - 1];
        }
        else
            data[0] = cmd;
        data[1] = index % 0x0100;
        data[2] = index / 0x0100;
        data[3] = subindex;

        memcpy(data + 4, &t, sizeof(T));
        return 0;
    }
    int ContrustRead(CanStamped* frame, uint8_t slave, uint16_t index, uint8_t subindex)
    {
        uint8_t *data = frame[0].data;
        frame[0].can_id = slave + 0x600;
        frame[0].can_dlc = 8;
        memset(data, 0, 8);

        data[0] = 0x40;
        data[1] = index % 0x0100;
        data[2] = index / 0x0100;
        data[3] = subindex;
        return 0;
    } 
    template <class T>
    int write(uint8_t slave, uint16_t index, uint8_t subindex, T t, uint8_t cmd = 0)
    {
        frame.resize(1);
        ContrustWrite<T>(&frame[0], slave, index, subindex, t, cmd);
        return do_write(slave, index, subindex);
    }
};

}


}