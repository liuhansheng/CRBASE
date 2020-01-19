#include <common/dispatch.h>
#include <msgs/chassis.pb.h>
#include "eq1.h"
#include "vehicle.h"
#include "common/can.hpp"
using namespace COWA::drivers;

class Vehicle : public BaseVehicle
{
private:
  CanStamped can_idm1;
  CanStamped can_idm2;
  uint8_t driving_mode = 0;
  double steerCmd;
  double speedCmd;

public:
  Vehicle()
  {
    vehicle_info->add_wheel_speed(0);
    vehicle_info->add_wheel_speed(0);
    vehicle_info->add_wheel_speed(0);
    vehicle_info->add_wheel_speed(0);

    can_idm1.can_id = 0x2c5;
    can_idm1.can_dlc = 8;
    memset(can_idm1.data, 0, 8);

    can_idm2.can_id = 0x1c5;
    can_idm2.can_dlc = 8;
    memset(can_idm2.data, 0, 8);

    can_drive[0]->Register(0x1d5, [this](const CanStamped &frame) -> void {
      this->eps_status_1d5(frame);
    });
    can_drive[0]->Register(0x151, [this](const CanStamped &frame) -> void {
      this->vcu_status_151(frame);
    });
    can_drive[0]->Register(0x300, [this](const CanStamped &frame) -> void {
      this->abs_status_300(frame);
    });
    can_drive[0]->Register(0x318, [this](const CanStamped &frame) -> void {
      this->abs_car_speed_318(frame);
    });
  }

  int UpdateFBK(std::shared_ptr<COWA::NavMsg::VehicleInfo> fbk)
  {
    return 0;
  }
  int UpdateCMD(std::shared_ptr<COWA::NavMsg::VehicleCommond> cmd) override
  {
    can_idm1.timestamp = (COWA::Time::Now().ToNanosecond());
    can_drive[0]->Write(can_idm1);
    can_idm2.timestamp = (COWA::Time::Now().ToNanosecond());
    can_drive[0]->Write(can_idm2);
    drive(cmd);
  
    return 0;
  }
  int DirectCtrl(std::shared_ptr<COWA::NavMsg::VehicleCommond> cmd) override
  {

  }
private:
  void reset()
  {
    steerCmd = 0;
    speedCmd = 0;
  }

  //feedback
  void eps_status_1d5(const CanStamped &frame)
  {
    eps_status *status = (eps_status *)(frame.data);

    int EPS_AutoCtrlModeSts = status->eps_status_byte0.EPS_AutoCtrlModeSts;
    uint16_t steer_angle = status->eps_status_byte1.EPS_SteeringAngleH * 256 + status->eps_status_byte2.EPS_SteeringAngleL;
    double angle = -(steer_angle * 0.1 - 1080) / 57.3 / 15.27;
    vehicle_info->mutable_drive()->set_steer(angle);
    vehicle_info->set_timestamp_steer(frame.timestamp);
    CRTRACE << "angle: " << angle;
  }

  void vcu_status_151(const CanStamped &frame)
  {
    const uint8_t null = 0x0a;
    const uint8_t drive = 0x01;
    const uint8_t reverse = 0x09;
    vcu_status *status = (vcu_status *)(frame.data);

    uint8_t mVCU_ActGear = status->vcu_status_byte0.VCU_ActGear;
    mVCU_ActGear = mVCU_ActGear & 0x0f;

    uint8_t VCU_ActGear_VD = status->vcu_status_byte2.VCU_ActGear_VD;
    VCU_ActGear_VD = VCU_ActGear_VD & 0x01;

    uint8_t VCU_ReadySts = status->vcu_status_byte2.VCU_ReadySts;
    VCU_ReadySts = VCU_ReadySts & 0x01;

    uint8_t mVCU_AutoCtrlModeSts = status->vcu_status_byte2.VCU_AutoCtrlModeSts;
    mVCU_AutoCtrlModeSts = mVCU_AutoCtrlModeSts & 0x01;

    if (mVCU_ActGear == drive)
      vehicle_info->mutable_drive()->set_gear(COWA::NavMsg::VehicleDrive_GearPosition_GEAR_DRIVE);
    else if (mVCU_ActGear == reverse)
      vehicle_info->mutable_drive()->set_gear(COWA::NavMsg::VehicleDrive_GearPosition_GEAR_REVERSE);
    else if (mVCU_ActGear == null)
      vehicle_info->mutable_drive()->set_gear(COWA::NavMsg::VehicleDrive_GearPosition_GEAR_NEUTRAL);

    if (mVCU_AutoCtrlModeSts == 1)
      driving_mode = 1;
    else
      driving_mode = 0;
  }
  void abs_status_300(const CanStamped &frame)
  {
    std::vector<double> speed;
    abs_wheel_status *status = (abs_wheel_status *)(frame.data);

   // if (status->abs_status_byte0.ABS_WheelSpd_FL_VD)
   //  {
    uint16_t FL_speed = uint16_t(status->abs_status_byte0.ABS_WheelSpd_FL_H << 8 | status->abs_status_byte1.ABS_WheelSpd_FL_L);
    speed.emplace_back(FL_speed * 0.0625 / 3.6);
   //  }
   // else
   //   speed.emplace_back(NAN);

  //  if (status->abs_status_byte2.ABS_WheelSpd_FR_VD)
  //  {
    uint16_t FR_speed = uint16_t(status->abs_status_byte2.ABS_WheelSpd_FR_H << 8 | status->abs_status_byte3.ABS_WheelSpd_FR_L);
    speed.emplace_back(FR_speed * 0.0625 / 3.6);
   // }
   // else
   //   speed.emplace_back(NAN);

   // if (status->abs_status_byte4.ABS_WheelSpd_RL_VD)
   // {
    uint16_t RL_speed = uint16_t(status->abs_status_byte4.ABS_WheelSpd_RL_H << 8 | status->abs_status_byte5.ABS_WheelSpd_RL_L);
    speed.emplace_back(RL_speed * 0.0625 / 3.6);
   // }
   // else
   //   speed.emplace_back(NAN);

   // if (status->abs_status_byte4.ABS_WheelSpd_RL_VD)
   // {
    uint16_t RR_speed = uint16_t(status->abs_status_byte6.ABS_WheelSpd_RR_H << 8 | status->abs_status_byte7.ABS_WheelSpd_RR_L);
    speed.emplace_back(RR_speed * 0.0625 /3.6);
   //  }
   // else
   //   speed.emplace_back(NAN);

    uint8_t index = 0;
    if (vehicle_info->mutable_drive()->gear() == COWA::NavMsg::VehicleDrive_GearPosition_GEAR_REVERSE)
    {
      for (auto &i : speed)
      {
        vehicle_info->set_wheel_speed(index, -i);
        ++index;
      }
     // vehicle_info->mutable_drive()->set_speed((speed[2] + speed[3]) / 2);
    }
    else
    {
      for (auto &i : speed)
      {
        vehicle_info->set_wheel_speed(index, i);
        ++index;
      }
     // vehicle_info->mutable_drive()->set_speed((speed[2] + speed[3]) / 2);
    }
    vehicle_info->set_timestamp_speed(frame.timestamp);
  }

  void abs_car_speed_318(const CanStamped &frame)
  {
    abs_car_status *status = (abs_car_status *)(frame.data);
    double speed = 0;
    //if (status->abs_status_byte2.ABS_VehSpd_VD)
    //{
      uint8_t h = status->abs_status_byte3.ABS_Car_Speed_H;
      uint8_t l = status->abs_status_byte4.ABS_Car_Speed_L;

      speed = (h * 256 + l) * 0.01 / 3.6;
      if (vehicle_info->mutable_drive()->gear() == COWA::NavMsg::VehicleDrive_GearPosition_GEAR_REVERSE)
        speed = -speed;
   // }
   // else
   //  {
   //    speed = NAN;
   // }
    vehicle_info->mutable_drive()->set_speed(speed);
    CRTRACE << "speed " << speed;
  }

  // control BCM
  int turn_signal_request(COWA::NavMsg::VehicleBCM::TurnSignal target)
  {
    idm1 *ctrl = (idm1 *)(can_idm1.data);

    if (target == COWA::NavMsg::VehicleBCM_TurnSignal_TURN_RIGHT)
    {
      
      ctrl->idm_byte1.IDM_TurnlightReq_R = 1;
      ctrl->idm_byte1.IDM_TurnlightReq_L = 0;
    }
    else if (target == COWA::NavMsg::VehicleBCM_TurnSignal_TURN_LEFT)
    {
      ctrl->idm_byte1.IDM_TurnlightReq_R = 0;
      ctrl->idm_byte1.IDM_TurnlightReq_L = 1;
    }
    else
    {
      ctrl->idm_byte1.IDM_TurnlightReq_R = 0;
      ctrl->idm_byte1.IDM_TurnlightReq_L = 0;
    }
    return 0;
  }

  int warning_lamp(bool flag) { return 0; }
  int head_lamp(bool flag) { return 0; }
  int stop_lamp(bool flag) { return 0; }
  int far_near(bool far) { return 0; }
  int loudspeaker(bool flag) { return 0; }

  //control car
  int mode_request(bool mode)
  {
    idm1 *ctrl = (idm1 *)(can_idm1.data);
    if (mode == 1)
      ctrl->idm_byte0.IDM_AutomaticDriveModeReq = 1;
    else
      ctrl->idm_byte0.IDM_AutomaticDriveModeReq = 0;
    return 0;
  }

  int gear_request(COWA::NavMsg::VehicleDrive::GearPosition target)
  {
    if (vehicle_info->mutable_drive()->speed() < 0.01)
    {
      idm2 *ctrl = (idm2 *)(can_idm2.data);

      switch (target)
      {
        case COWA::NavMsg::VehicleDrive_GearPosition_GEAR_NEUTRAL:
          ctrl->idm_byte0.IDM_GearReq = 0xA;
          break;
        case COWA::NavMsg::VehicleDrive_GearPosition_GEAR_DRIVE:
          ctrl->idm_byte0.IDM_GearReq = 0x1;
          break;
        case COWA::NavMsg::VehicleDrive_GearPosition_GEAR_REVERSE:
          ctrl->idm_byte0.IDM_GearReq = 0x9;
          break;
        case COWA::NavMsg::VehicleDrive_GearPosition_GEAR_PARKING:
          ctrl->idm_byte0.IDM_GearReq = 0xB;
          break;

        default:
          ctrl->idm_byte0.IDM_GearReq = 0x0;
      }
    }
    return 0;
  }

  int throttle_request(double value)
  {
    return 0;
  }
  int speed_request(double value)
  {
    if (value > 30)
      value = 30;
    else if (value < 0)
      value = 0;

    idm2 *ctrl = (idm2 *)(can_idm2.data);

    if (vehicle_info->mutable_drive()->gear() == COWA::NavMsg::VehicleDrive_GearPosition_GEAR_DRIVE ||
        vehicle_info->mutable_drive()->gear() == COWA::NavMsg::VehicleDrive_GearPosition_GEAR_REVERSE)
    {
      value = value * 10 * 3.6;

      uint8_t speed = (uint8_t)(value);
      if(speed > 0.1)
      ctrl->idm_byte1.IDM_VehSpd = speed;
    }
    else
      ctrl->idm_byte1.IDM_VehSpd = 0;

    return 0;
  }

  int brake_request(double value) /* 0 - 1, for 1 is 100% */
  {
    if (value < 0)
      value = 0;
    else if (value > 1)
      value = 1;

    idm2 *ctrl = (idm2 *)(can_idm2.data);

    uint8_t pressure = (value * 100.0) / 0.4;
    ctrl->idm_byte2.IDM_BrakePedalPositionReq = pressure;

    return 0;
  }

  int steer_request(double angle)
  {
    idm2 *ctrl = (idm2 *)(can_idm2.data);
    angle = -angle * 15.27;

    double target = angle * 180.0 / 3.1415;

    if (target > 540)
      target = 540;
    if (target < -540)
      target = -540;

    uint16_t cmd_angle = (uint16_t)((target + 1080) / 0.1);
    uint16_t motorola_cmd_angle = htons(cmd_angle);
    ctrl->idm_byte4_5.IDM_SteeringAngleReq = motorola_cmd_angle;
    return 0;
  }

  //drive
  int drive(std::shared_ptr<COWA::NavMsg::VehicleCommond> vehicle_cmd)
  {
    if (vehicle_cmd->mutable_drive()->driving_mode() == COWA::NavMsg::VehicleDrive_DrivingMode_MANUAL)
    {
      if (vehicle_info->mutable_drive()->gear() != COWA::NavMsg::VehicleDrive_GearPosition_GEAR_NEUTRAL && driving_mode == 1)
      {
        auto_drive(vehicle_cmd, COWA::NavMsg::VehicleDrive_GearPosition_GEAR_NEUTRAL);
      }
      else
        mode_request(0);
    }
    else
    {
      auto_drive(vehicle_cmd, vehicle_cmd->mutable_drive()->gear());
    }
    return 0;
  }

  //autonomous driving car
  int auto_drive(std::shared_ptr<COWA::NavMsg::VehicleCommond> vehicle_cmd, COWA::NavMsg::VehicleDrive::GearPosition target_gear)
  {
    //step 1: mode request
    if (driving_mode == 0)
    {
      mode_request(1);
      speed_request(0);
      steer_request(0);
    }
    //step 2: gear switch
    else if (target_gear != vehicle_info->mutable_drive()->gear())
    {
      //stage 1: control car speed to 0, using brake
      if (vehicle_info->mutable_drive()->speed() > 0.1 / 3.6)
        brake_request(0.8);
      // stage 2: gear switch to neutral gear
      else if (vehicle_info->mutable_drive()->gear() != COWA::NavMsg::VehicleDrive_GearPosition_GEAR_NEUTRAL)
      {
        gear_request(COWA::NavMsg::VehicleDrive_GearPosition_GEAR_NEUTRAL);
      }
      //stage 3: gear switch to target gear
      else
      {
        gear_request(target_gear);
      }
    }
    //step 3: car control
    else if (vehicle_info->mutable_drive()->gear() != COWA::NavMsg::VehicleDrive_GearPosition_GEAR_NEUTRAL)
    {
      lateral_longitudinal_control(vehicle_cmd);
      bcm_control(vehicle_cmd);
    }
    return 0;
  }

  int lateral_longitudinal_control(std::shared_ptr<COWA::NavMsg::VehicleCommond> vehicle_cmd)
  {
    switch (vehicle_cmd->mutable_drive()->driving_mode())
    {
    case COWA::NavMsg::VehicleDrive_DrivingMode_AUTO_DRIVE:
      speed_request(speed_control(vehicle_cmd));
      steer_request(steer_control(vehicle_cmd));
      break;
    case COWA::NavMsg::VehicleDrive_DrivingMode_AUTO_STEER_ONLY:
      steer_request(steer_control(vehicle_cmd));
      break;
    case COWA::NavMsg::VehicleDrive_DrivingMode_AUTO_SPEED_ONLY:
      speed_request(steer_control(vehicle_cmd));
      break;
    //case COWA::NavMsg::VehicleDrive_DrivingMode_MANUAL:
    //  speed_request(0);
    //  steer_request(0);
    default:
      speed_request(0);
      steer_request(0);
    }
    return 0;
  }

  int bcm_control(std::shared_ptr<COWA::NavMsg::VehicleCommond> vehicle_cmd)
  {
    turn_signal_request(vehicle_cmd->mutable_bcm()->turn_signal());
  }

  double speed_control(std::shared_ptr<COWA::NavMsg::VehicleCommond> vehicle_cmd)
  {
    auto clamp = [](double v, double maxV) {
      if (v > maxV)
        return maxV;
      if (v < -maxV)
        return -maxV;
      return v;
    };
    double maxSpeed = 4;
    double target = clamp(vehicle_cmd->mutable_drive()->speed(), maxSpeed);

    if (fabs(target) < 0.1)
    {
      target = 0;
    }
    else
      brake_request(0);

    return target;
  }

  double steer_control(std::shared_ptr<COWA::NavMsg::VehicleCommond> vehicle_cmd)
  {
    auto clamp = [](double angle, double maxAngle) {
      if (angle > maxAngle)
        return maxAngle;
      if (angle < -maxAngle)
        return -maxAngle;
      return angle;
    };

    steerCmd = vehicle_cmd->mutable_drive()->steer();
    return steerCmd;
  }
};

REGISTER_CAN(EQ1, Vehicle);