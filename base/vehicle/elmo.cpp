#include "elmo.hpp"
using namespace COWA;
using namespace COWA::drivers;
template <class T>
T ElmoParam(YAML::Node &node, const std::string &name, const T t = T())
{
  auto x = node[name];
  if (!x)
    return t;
  return x.as<T>();
}
Elmo::Elmo(std::shared_ptr<CANDispatch> _can, YAML::Node node, bool _posmode)
    : posmode(_posmode), can(_can)
{
  slave = ElmoParam<int>(node, "addr", 2);
  int min = ElmoParam<int>(node, "min", 0);
  int max = ElmoParam<int>(node, "max", 100000);
  float vmin = ElmoParam<float>(node, "vmin", 0);
  float vmax = ElmoParam<float>(node, "vmax", 1);
  MapRange(min, max, vmin, vmax);

  _operational_dealy = 0;
  _enable = false;
  _reset = false;
  _falut = false;
  _error_code = 0;
  _error_request = 0;
  _error_cnt = 0;
  memset(&fbk, 0, sizeof(fbk));

  auto timeout = [this]() {
    CRERROR << "elmo timeout, addr = " << int(slave);
    _falut = true;
    _error_code = 0xffff;
  };
  can->Register(0x180 + slave, [this](const struct CanStamped &frame) -> void {
      this->UpdateStatus(frame);
    }, 200, timeout);
  can->Register(0x580 + slave, [this](const struct CanStamped &frame) -> void {
    this->UpdateFault(frame);
  });
}

int Elmo::UpdateStatus(const CanStamped &rframe)
{
  memcpy(&fbk, rframe.data, 6);
  CRTRACE << "status: " << fbk.status << "pos: " << fbk.pos;

  if ((fbk.status & 0x8) && _reset)
  {
    cmd.ctrl = 0x80; //fault reset
    cmd.target = 0;
  }
  else
    _reset = false;

  if (_enable && _reset == false)
  {
    if (_operational_dealy > 0)
      _operational_dealy -= 1;
    uint16_t state = fbk.status & 0x0f;
    if (state == 0)
      cmd.ctrl = 0x6; //shutdown 2
    else if (state == 1)
      cmd.ctrl = 0x7; //swithc on 3
    else if (state == 3)
      cmd.ctrl = 0xf; //op 4
    else if (state != 7)
      _enable = false;
  }
  if ((fbk.status & 0x0f) == 7)
    _operational = 1;
  else
    _operational = 0;
  if (posmode && _operational && pos_shift == false)
  {
    cmd.ctrl = 0x3f;
    pos_shift = true;
  }
  else if (posmode && _operational && pos_shift == true)
  {
    cmd.ctrl = 0x0f;
    pos_shift = false;
  }

  if (_enable == false && _reset == false)
    cmd.ctrl = 0;

  CanStamped frame;
  if (fbk.status & 0x8)
  {
    if(_error_request % 50 == 1)
    {
      can->COE().ContrustRead(&frame, slave, 0x603f, 0);
      can->Write(frame);
    }
    _error_request += 1;
    _error_cnt += 1;
    CanStamped frame;
    can->COE().ContrustWrite<uint32_t>(&frame, slave, 0x2f21, 1, 0x00000001); //reset battery error
    can->Write(frame);
  }
  else
  {
    _error_request = 0;
    _error_cnt = 0;
    _falut = false;
  }
  if(_error_cnt > 5)
    _falut = true;

  frame.can_id = 0x200 + slave;
  frame.can_dlc = 6;
  memcpy(frame.data, &cmd, 6);
  can->Write(frame);
  return 0;
}
struct DS402Error
{
  uint16_t code;
  const char *text;
};
DS402Error code_impl[] = {
    {0x2340, "error:elmo short circuit motor or its wiring may be defective,or drive is faulty."},
    {0x3120, "error:elmo Under-voltage."},
    {0x3333, "error:elmo Over-voltage."},
    {0x4310, "error:elmo temperature drive is overheating."},
    {0x5280, "error:ECAM table problem."},
    {0x5281, "error:elmo Timing error."},
    {0x5400, "error:elmo cannot start motor."},
    {0x5441, "error:elmo disabled by limit switch."},
    {0x6180, "error:elmo stack overflow."},
    {0x6181, "error:elmo CPU exception FATAL."},
    {0x6320, "error:elmo inconsistent database."},
    {0x7121, "error:elmo motor stuck. Moter powered but not moving."},
    {0x7300, "error:elmo resolver or analog encoder feedback loss"},
    {0x7380, "error:elmo Feedback loss.No match between encoder and hall location."},
    {0x8130, "error:elmo Heartbeat failure."},
    {0x8311, "error:elmo Peak current has been exceeded due to badly-tuned current controller or drive malfunction."},
    {0x8380, "error:elmo cannot find electrial zero of motor."},
    {0x8381, "error:elmo cannot tune current offsets."},
};
int Elmo::UpdateFault(const CanStamped &x)
{
  auto a = x.data[5];
  auto b = x.data[4];
  uint16_t code = a << 8 + b;
  for (auto &i : code_impl)
  {
    if (code == i.code)
    {
      CRERROR << i.text;
      _error_code = code;
    }
  }
}
uint16_t Elmo::GetFault()
{
  if (_falut)
    return _error_code;
  else
    return 0;
}
int Elmo::SetTarget(int32_t t)
{
  cmd.target = t;
  if (_falut)
  {
    CRINFO << "slave "<< int(slave) << "elmo falut: "<< std::hex << _error_code;
    return _error_code;
  }
  else if (!_operational && _operational_dealy == 0)
    return -1;
  else
    return 0;
}
void Elmo::SetProfiledVelocity(int32_t t)
{
  CanStamped frame;
  can->COE().ContrustWrite<uint32_t>(&frame, slave, 0x6081, 0, t);
  can->Write(frame);
}
int32_t Elmo::GetPos(void)
{
  return fbk.pos;
}
void Elmo::Enable()
{
  _operational_dealy = 50 * 3;
  _reset = true;
  _enable = true;
}
void Elmo::Disable()
{
  _enable = false;
}

void Elmo::Setup()
{
  if (posmode == false)
    can->COE().write<uint8_t>(slave, 0x6060, 0, 3); //Object 0x6060: Modes of operatio, int8 3 velocity, 1 position
  else
    can->COE().write<uint8_t>(slave, 0x6060, 0, 1);

  uint32_t t = 0x80000200 + slave;
  can->COE().write<uint32_t>(slave, 0x1400, 1, t); //disable pdo

  can->COE().write<uint32_t>(slave, 0x1600, 0, 0);
  can->COE().write<uint32_t>(slave, 0x1600, 1, 0x60400010); // control word
  if (posmode == false)
    can->COE().write<uint32_t>(slave, 0x1600, 2, 0x60FF0020); // speed
  else
    can->COE().write<uint32_t>(slave, 0x1600, 2, 0x607A0020); //position

  can->COE().write<uint32_t>(slave, 0x1600, 0, 2);    // 2 items
  can->COE().write<uint32_t>(slave, 0x1400, 2, 0xFE); //sync type
  can->COE().write<uint16_t>(slave, 0x1017, 0, 1000); //# 1000ms

  t = 0x200 + slave;
  can->COE().write<uint32_t>(slave, 0x1400, 1, t); // enable

  t = 0xC0000180 + slave;
  can->COE().write<uint32_t>(slave, 0x1800, 1, t);

  can->COE().write<uint32_t>(slave, 0x1a00, 0, 0);
  can->COE().write<uint32_t>(slave, 0x1a00, 1, 0x60410010); //status
  can->COE().write<uint32_t>(slave, 0x1a00, 2, 0x60640020); //postion

  can->COE().write<uint32_t>(slave, 0x1a00, 0, 2);    //2items
  can->COE().write<uint32_t>(slave, 0x1800, 2, 0xfe); // # sync type
  can->COE().write<uint32_t>(slave, 0x1800, 5, 50);   // # time

  t = 0x40000180 + slave;
  can->COE().write<uint32_t>(slave, 0x1800, 1, t); // enable

  can->COE().write<uint32_t>(slave, 0x6083, 0, (1 << 17) * 100); //acc
  can->COE().write<uint32_t>(slave, 0x6084, 0, (1 << 17) * 100); //dec
  can->COE().write<uint32_t>(slave, 0x6081, 0, (1 << 17) * 50);  //profile velocity
}