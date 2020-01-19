#include "common/can.hpp"
#include <crmw/crmw.h>
#include "common/dispatch.h"
#include <msgs/chassis.pb.h>
namespace COWA
{
namespace drivers
{

class Vehicle: public COWA::TimerComponent
{
private:
  std::vector< std::shared_ptr<CANProcess> > process;
public:
  bool Init() override
  {
    auto param = std::make_shared<COWA::Parameter>(node_);
    
    std::vector< std::shared_ptr<CANDispatch> > cans;
    std::vector<std::string> canbus = param->value< std::vector<std::string> >("canbus");
    for(auto& i: canbus)
    {
      auto x = std::make_shared<CANDispatch>(i);
      cans.emplace_back(x);
    }
    CANProcess::cans = cans;
    CANProcess::node = node_;

    auto& dict = *CANProcess::impls;
    std::vector<std::string> canprocess = param->value< std::vector<std::string> > ("process");
    for(auto& i: canprocess)
    {
      if(dict.find(i) == dict.end())
      {
        CRFATAL << "module " << i << "not found";
        exit(-1);
      }
      process.emplace_back(
       dict.find(i)->second()
      );
    }

    CanStamped frame;
    frame.can_id = 0;
    frame.can_dlc = 2;
    frame.data[0] = 0x80;
    frame.data[1] = 0x00;
    for(int i = 0; i < 5; ++i)
    {
      for(auto &c: CANProcess::cans)
      {
        c->Write(frame);
        c->Update();
      } 
      usleep(10*1000);
    }
    frame.can_id = 0;
    frame.can_dlc = 2;
    frame.data[0] = 0x01;
    frame.data[1] = 0x00;
    for(int i = 0; i < 5; ++i)
    {
      for(auto &c: CANProcess::cans)
      {
        c->Write(frame);
        c->Update();
      } 
      usleep(10*1000);
    }
    return true;
  }
  bool Proc() override
  {
    for(auto& i: CANProcess::cans)
    {
      i->Update();
    }
    for(auto& i: process){
      i->Update();
    }
  }
};


NODE_REGISTER_COMPONENT(Vehicle);
} // namespace drivers
}