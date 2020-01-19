#pragma once
#include <common/dispatch.h>
#include <msgs/chassis.pb.h>
#include <crmw/service.h>

namespace COWA
{
namespace drivers
{
class BaseVehicle : public CANProcess
{
private:
  std::mutex lock;
  std::shared_ptr<COWA::Writer<COWA::NavMsg::VehicleInfo>> fbk;
  std::shared_ptr<COWA::Reader<COWA::NavMsg::VehicleCommond>> cmd;
  std::shared_ptr<COWA::Reader<COWA::NavMsg::VehicleCommond>> joystick;
  bool joystickCtrl;
  
  std::shared_ptr<COWA::Service<NavMsg::VehicleCommond, 
                                NavMsg::VehicleInfo>> service;
  std::shared_ptr<COWA::NavMsg::VehicleCommond> vehicle_cmd;
  std::shared_ptr<COWA::NavMsg::VehicleCommond> vehicle_mask;
  void PrepareCmd();

  std::shared_ptr<COWA::util::ObjectPool<COWA::NavMsg::VehicleInfo> > object_pool_;
protected:
  std::vector<std::shared_ptr<CANDispatch>> can_drive;
  std::shared_ptr<COWA::Parameter> param;
  std::shared_ptr<COWA::NavMsg::VehicleInfo> vehicle_info;
  uint32_t sequence = 0;
public:
  BaseVehicle();
  int Update() override;
  virtual int UpdateCMD(std::shared_ptr<COWA::NavMsg::VehicleCommond> cmd) = 0;
  virtual int UpdateFBK(std::shared_ptr<COWA::NavMsg::VehicleInfo> fbk) = 0;
  virtual int DirectCtrl(std::shared_ptr<COWA::NavMsg::VehicleCommond> cmd) = 0;
};
}
}