#include "dispatch.h"
using namespace COWA::drivers;


std::map<std::string, std::function<std::shared_ptr<CANProcess>()> > * CANProcess::impls;
std::vector< std::shared_ptr<CANDispatch> > CANProcess::cans;
std::shared_ptr<COWA::Node> CANProcess::node;
CANDispatch::CANDispatch(const std::string& dev)
{
  if(dev.find("can") == 0)
    can = std::make_shared<SocketCAN>();
  else
    can = std::make_shared<CANOverUDP>();
  can->open(dev.c_str());
  buffer_send.reserve(128);

  coe = std::make_shared<CANOPEN>(can);
}
void CANDispatch::Update()
{
  std::vector<CanStamped> frames;
  can->recv(frames, 0);
  for(int i = 0; i < frames.size(); ++i)
  { 
    uint32_t id = frames[i].can_id & CAN_EFF_MASK;
    auto x = callbacks.find(id);
    if(x != callbacks.end())
    {
      x->second.callback(frames[i]);
      x->second.tick = 0;
    }
  } 

  can->send(buffer_send, 5);
  buffer_send.clear();

  for(auto& i: callbacks)
  {
    if(i.second.interval <= 1)
      continue;
    i.second.tick += 1;
    if(i.second.tick > i.second.interval && i.second.timeout)
    {
      i.second.tick = 0;
      i.second.timeout();
    }
  }
}