#pragma once
#include "can.hpp"
#include "canopen.hpp"
#include <crmw/crmw.h>
namespace COWA
{
namespace drivers
{

class CANDispatch
{
private:
  struct CanCallback
  {
    int tick;
    int interval;
    std::function<void(void)> timeout;
    std::function<void(const CanStamped &)> callback;
  };
  std::shared_ptr<CANStream> can;
  std::shared_ptr<CANOPEN>  coe;
  std::map<uint32_t, CanCallback> callbacks;
  std::vector<CanStamped> buffer_send;
public:
  CANDispatch(const std::string& dev);
  inline void Register(uint32_t id, std::function<void(const CanStamped &)>cbk, 
      int cnts = -1, std::function<void(void)> timeout = NULL)
  {
    if(callbacks.find(id) != callbacks.end())
      CRFATAL << "can id overlap";
    

    CanCallback t;
    t.interval = cnts;
    t.tick = 0;
    t.timeout = timeout;
    t.callback = cbk;
    callbacks[id] = t;
  }

  inline void Write(const CanStamped& frame)
  {
    buffer_send.emplace_back(frame);
  }
  inline CANOPEN& COE()
  {
    return *(coe.get());
  }

  void Update();
};

class CANProcess
{
public:
  static std::map<std::string, std::function<std::shared_ptr<CANProcess>()> > * impls;
  static std::vector< std::shared_ptr<CANDispatch> > cans;
  static std::shared_ptr<COWA::Node> node;

  virtual int Update() = 0;
};
#define REGISTER_CAN(name, x) \
class REG_##x{\
public:\
    REG_##x() {\
        auto& impls = COWA::drivers::CANProcess::impls;\
        if(impls == NULL) impls = new std::map<std::string, std::function<std::shared_ptr<CANProcess>()>  >();\
        (*impls)[#name] = []() -> std::shared_ptr<COWA::drivers::CANProcess> { \
            return std::make_shared<x>(); \
        }; \
    }\
};\
static REG_##x reg_##x;\
extern "C" __attribute__((constructor)) void* __register_can_##name(){ return& reg_##x ; }

} // namespace drivers
}