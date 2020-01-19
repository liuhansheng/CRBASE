#ifndef CAN_ELMO
#define CAN_ELMO

#include "../common/dispatch.h"
#include "vehicle.h"
#include "common/can.hpp"


namespace COWA
{
namespace drivers
{
class Elmo
{
private:
    uint8_t slave = 2;
    bool posmode;
    bool pos_shift = false;

    bool _enable;
    bool _enabled;
    bool _reset;
    bool _falut;
    uint16_t _error_code;
    uint16_t   _error_cnt;

    bool _operational;
    int _operational_dealy;
    float v_min = 0;
    float v_max = 0;

    int _error_request;
    struct __attribute__((__packed__))
    {
        uint16_t ctrl;
        int32_t target;
    } cmd;
    struct __attribute__((__packed__))
    {
        uint16_t status;
        int32_t pos;
    } fbk;
    std::shared_ptr<CANDispatch> can;
    int UpdateStatus(const CanStamped &frame);
    int UpdateFault(const CanStamped &frame);

public:
    Elmo(std::shared_ptr<CANDispatch> _can, YAML::Node node, bool _posmode = true);

    uint16_t GetFault();
    void Enable();
    void Disable();
    void Setup();
private:
    int SetTarget(int32_t t);
    void SetProfiledVelocity(int32_t t);
    int32_t GetPos(void);
    int32_t offset;
    float scale;
    inline void MapRange(int32_t min, int32_t max, float vmin = 0, float vmax = 1)
    {
        scale = (max - min) / (vmax - vmin);
        offset = min - vmin * scale;
        v_min = vmin;
        v_max = vmax;
    }
public:
    inline int CmdTarget(float v)
    {
        // if(v < v_min)
        //     v = v_min;
        // else if(v > v_max)
        //     v = v_max;
        return SetTarget(v * scale + offset);

    }
    inline void CmdProfiledVelocity(float v)
    {
        return SetProfiledVelocity(fabs(v * scale));
    }
    inline float FbkPos()
    {
        return (GetPos() - offset) / scale;
    }
};
} // namespace drivers
} // namespace COWA
#endif