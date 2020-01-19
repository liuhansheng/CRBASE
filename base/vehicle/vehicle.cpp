#include <common/dispatch.h>
#include <msgs/chassis.pb.h>
#include "vehicle.h"

namespace COWA
{
namespace drivers
{

BaseVehicle::BaseVehicle()
{
  param = std::make_shared<COWA::Parameter>(node);

  std::vector<std::string> canbus = param->value< std::vector<std::string> >("chassis/canbus");
  bool joystickEnable = param->value<bool>("chassis/joystick", false);
  CRTRACE << "joystick enable "<<joystickEnable;
  for(auto& i: canbus)
  {
    can_drive.push_back(cans[atoi(i.c_str())]);
  }
  
  fbk = node->CreateWriter<COWA::NavMsg::VehicleInfo>("/base_info");
  cmd = node->CreateReader<COWA::NavMsg::VehicleCommond>("/ctrl_cmd");
  joystickCtrl = false;
  if(joystickEnable)
  {
    joystick = node->CreateReader<COWA::NavMsg::VehicleCommond>("/joystick", 
        [this](const std::shared_ptr<COWA::NavMsg::VehicleCommond>& msg){
          lock.lock();
          if(!msg->has_drive() || !msg->drive().has_driving_mode())
            joystickCtrl = false;
          else
          {
            joystickCtrl = true;
            this->DirectCtrl(msg);
          }
          lock.unlock();
        });
  }

  vehicle_info = std::make_shared<COWA::NavMsg::VehicleInfo>();
  object_pool_ = std::make_shared<COWA::util::ObjectPool<COWA::NavMsg::VehicleInfo> > (256);

  vehicle_cmd = std::make_shared<COWA::NavMsg::VehicleCommond>();
  vehicle_mask = std::make_shared<COWA::NavMsg::VehicleCommond>();
  service = node->CreateService<NavMsg::VehicleCommond, NavMsg::VehicleInfo>("vehicle", 
      [this](const std::shared_ptr<NavMsg::VehicleCommond>& cmd_t, std::shared_ptr<NavMsg::VehicleInfo>& fbk){
        lock.lock();
        vehicle_mask->CopyFrom(*cmd_t);
        PrepareCmd();
        UpdateCMD(vehicle_cmd);
        cmd->ClearData();
        *fbk = *vehicle_info;
        lock.unlock();
      });
}
void BaseVehicle::PrepareCmd()
{
  #define CHECKX2Y(name) if(x.has_##name())y->set_##name(x.name())
  if(vehicle_mask->has_drive())
  {
    auto& x = vehicle_mask->drive();
    auto y = vehicle_cmd->mutable_drive();
    CHECKX2Y(driving_mode);
    CHECKX2Y(gear);
    CHECKX2Y(steer);
    CHECKX2Y(steer_speed);
    CHECKX2Y(throttle);
    CHECKX2Y(speed);
    CHECKX2Y(speed_mode);
  }
  if(vehicle_mask->has_bcm())
  {
    auto& x = vehicle_mask->bcm();
    auto y = vehicle_cmd->mutable_bcm();
    CHECKX2Y(turn_signal);
    CHECKX2Y(high_beam);
    CHECKX2Y(low_beam);
    CHECKX2Y(horn);
    CHECKX2Y(emergency_light);
    CHECKX2Y(wiper);
    CHECKX2Y(sweep);
    CHECKX2Y(vacuum);
    CHECKX2Y(spray);
  }
}
int BaseVehicle::Update()
{
  cmd->Observe();
  vehicle_cmd->Clear();
  
  if(!joystickCtrl)
  {
    if (cmd->HasReceived())
      vehicle_cmd->CopyFrom(*(cmd->GetLatestObserved()));
    lock.lock();
    PrepareCmd();
    UpdateCMD(vehicle_cmd);
    lock.unlock();
  }
  
  if(UpdateFBK(vehicle_info) == 0)
  {
    auto x = object_pool_->GetObject();
    if(x == NULL)
      CRWARN << "object get failed";
    else
    {
      x->CopyFrom(*vehicle_info);
      x->set_timestamp(COWA::Time().Now().ToNanosecond());
      x->set_sequence(sequence);
      fbk->Write(x);
      sequence++;
    }
  }
}


}
}