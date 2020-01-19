#include <common/dispatch.h>
#include <msgs/chassis.pb.h>
#include <msgs/pose.pb.h>
#include "vehicle.h"
#include "elmo.hpp"
using namespace COWA::drivers;

class Hulk : public BaseVehicle
{
private:
  std::shared_ptr<COWA::drivers::Elmo> turn;
  std::shared_ptr<COWA::drivers::Elmo> brake;
  std::shared_ptr<COWA::drivers::Elmo> clutch;

  struct CanStamped frame_throttle;

  double steer_ratio;
  double max_speed;
  double speedGain, speedToThrottle, throttleAcc, throttleDec;
  double throttleOut = 0;
  double throttleOffset;
  std::vector<double> brakeStage;
  int ctrlHz = 50;
public:
  Hulk()
  {
    CRINFO<< "Hulk init";
    auto turn_slave = param->query("chassis/turn");
    auto brake_slave = param->query("chassis/brake");
    auto clutch_slave = param->query("chassis/clutch");
    auto throttleFbk = param->value<bool>("chassis/throttleFbk", true);
    auto OBDFbk = param->value<bool>("chassis/OBDFbk", true);
    steer_ratio = param->value<double>("chassis/turn_ratio", 1);
    max_speed = param->value<double>("chassis/max_speed", 0.5);
    speedGain = param->value<double>("chassis/speedGain", 0.5);
    speedToThrottle = param->value<double>("chassis/speedToThrottle", 0.5);
    throttleAcc = param->value<double>("chassis/throttleAcc", 2);
    throttleDec = param->value<double>("chassis/ThrottleDec", 5);
    throttleOffset = param->value<double>("chassis/ThrottleOffset", 0.4);
    brakeStage = param->value<std::vector<double>>("chassis/brakeStage");

    turn.reset(new COWA::drivers::Elmo(can_drive[0], turn_slave));
    brake.reset(new COWA::drivers::Elmo(can_drive[0], brake_slave));
    clutch.reset(new COWA::drivers::Elmo(can_drive[0], clutch_slave));

    turn->Setup();
    brake->Setup();
    clutch->Setup();

    turn->Disable();
    brake->Disable();
    clutch->Disable();

    frame_throttle.can_id = 0x221;
    frame_throttle.can_dlc = 8;
    memset(frame_throttle.data, 0, 8);

    vehicle_info->add_wheel_speed(0);
    vehicle_info->add_wheel_speed(0);
    vehicle_info->add_wheel_speed(0);
    vehicle_info->add_wheel_speed(0);
    vehicle_info->set_wheel_speed(0, NAN);
    vehicle_info->set_wheel_speed(1, NAN);
    vehicle_info->set_wheel_speed(2, NAN);
    vehicle_info->set_wheel_speed(3, NAN);
    vehicle_info->mutable_drive()->set_driving_mode(COWA::NavMsg::VehicleDrive::MANUAL);
    if(throttleFbk)
    {   
      //油门模块，ID及超时处理
      auto throttle_timeout = [this] {
        CRERROR << "throttle error";
        vehicle_info->mutable_drive()->set_driving_mode(COWA::NavMsg::VehicleDrive::MANUAL);
        vehicle_info->set_error(COWA::NavMsg::DrivingError::CAN_ERROR);
      };
      can_drive[0]->Register(0x225, [this](const struct CanStamped &frame) -> void {
          // this->throttle_status_225(frame);
          //vehicle_info->clear_error();
        }, 500, throttle_timeout);
    }

    if(OBDFbk)
    {
      //ODB模块，ID及超时处理
      auto odb_timeout = [this] {
        CRERROR << "ODB error";
        vehicle_info->mutable_drive()->set_driving_mode(COWA::NavMsg::VehicleDrive::MANUAL);
        vehicle_info->set_error(COWA::NavMsg::DrivingError::CAN_ERROR);
        vehicle_info->clear_timestamp_speed();
        vehicle_info->mutable_drive()->clear_speed();
      };
      //最后一路can是给obd的
      can_drive.back()->Register(0x18FEBF0B, [this](const struct CanStamped &frame) -> void {
          float kpm = frame.data[0] / 256.0 + frame.data[1]; //1/256 km/h per bit

          if(kpm > 90)
          {
            vehicle_info->mutable_drive()->set_speed(NAN);// if speed feedback error, set a large number 
            CRWARN << "speed feedback error";
          }
          else
            vehicle_info->mutable_drive()->set_speed(kpm / 3.6);

          vehicle_info->set_wheel_speed(0, NAN);
          vehicle_info->set_wheel_speed(1, NAN);
          vehicle_info->set_wheel_speed(2, NAN);
          vehicle_info->set_wheel_speed(3, NAN);
          vehicle_info->set_timestamp_speed(frame.timestamp);
          //vehicle_info->clear_error();
        }, 200, odb_timeout);
    }
    else
    {
        vehicle_info->clear_timestamp_speed();
        vehicle_info->mutable_drive()->clear_speed();
        vehicle_info->set_wheel_speed(0, NAN);
        vehicle_info->set_wheel_speed(1, NAN);
        vehicle_info->set_wheel_speed(2, NAN);
        vehicle_info->set_wheel_speed(3, NAN);
    }
  }
  int UpdateFBK(std::shared_ptr<COWA::NavMsg::VehicleInfo> fbk) override
  {
    fbk->mutable_drive()->set_steer(steer_ratio * turn->FbkPos());
    fbk->set_timestamp_steer(COWA::Time::Now().ToNanosecond());
        
    fbk->clear_debug();
    auto x = fbk->add_debug();
    x->set_field("turn");
    x->set_value(turn->FbkPos());
    x = fbk->add_debug();
    x->set_field("clutch");
    x->set_value(clutch->FbkPos());
    x = fbk->add_debug();
    x->set_field("brake");
    x->set_value(brake->FbkPos());
    return 0;
  }


  int UpdateCMD(std::shared_ptr<COWA::NavMsg::VehicleCommond> cmd) override
  {
    if(!vehicle_info->has_drive() ||  !vehicle_info->drive().has_driving_mode())
      return 0;
    switch(vehicle_info->drive().driving_mode())
    {
      case COWA::NavMsg::VehicleDrive::MANUAL:
        StateManual(cmd);
        break;
      case COWA::NavMsg::VehicleDrive::STANDBY:
        StateStandby(cmd);
        break;
      case COWA::NavMsg::VehicleDrive::AUTO_SPEED_ONLY:
      case COWA::NavMsg::VehicleDrive::AUTO_STEER_ONLY:
      case COWA::NavMsg::VehicleDrive::AUTO_DRIVE:
        StateAutonomous(cmd);
        break;
    }

    vehicle_info->mutable_drive()->set_steer(turn->FbkPos());

    return 0;
  }
  int DirectCtrl(std::shared_ptr<COWA::NavMsg::VehicleCommond> cmd) override
  {
    if(cmd->mutable_drive()->gear() == COWA::NavMsg::VehicleDrive_GearPosition_GEAR_REVERSE)
    {
      //刹车，等待进入自动驾驶，如果有错误则退到手动
      turn->Disable();

      clutch->Enable();
      clutch->CmdProfiledVelocity(1);
      if(clutch->CmdTarget(1))
      {
        vehicle_info->mutable_drive()->set_driving_mode(COWA::NavMsg::VehicleDrive::MANUAL);
        vehicle_info->set_error(COWA::NavMsg::DrivingError::BRAKE_ERROR);
      }

      brake->Enable();
      brake->CmdProfiledVelocity(1);
      if(brake->CmdTarget(1))
      {
        vehicle_info->mutable_drive()->set_driving_mode(COWA::NavMsg::VehicleDrive::MANUAL);
        vehicle_info->set_error(COWA::NavMsg::DrivingError::BRAKE_ERROR);
      }

      //退出准备模式
      if(fabs(turn->FbkPos() - CheckPosStandby) > 0.1)
      {
        vehicle_info->mutable_drive()->set_driving_mode(COWA::NavMsg::VehicleDrive::MANUAL);
        vehicle_info->set_error(COWA::NavMsg::DrivingError::MANUAL_INTERVENTION);
      }
      vehicle_info->mutable_drive()->set_driving_mode(COWA::NavMsg::VehicleDrive::AUTO_DRIVE);
    }
    else if(cmd->mutable_drive()->gear() == COWA::NavMsg::VehicleDrive_GearPosition_GEAR_DRIVE)
    {
      vehicle_info->mutable_drive()->set_driving_mode(COWA::NavMsg::VehicleDrive::AUTO_DRIVE);
      StateAutonomous(cmd);
    }
  }

private:
  float CheckPosStandby;
  void StateManual(std::shared_ptr<COWA::NavMsg::VehicleCommond> cmd)
  {
    ThrottleRequest(0, 0);
    turn->Disable();
    clutch->Enable();
    clutch->CmdProfiledVelocity(0.2);
    if(clutch->CmdTarget(0))
    {
      vehicle_info->mutable_drive()->set_driving_mode(COWA::NavMsg::VehicleDrive::MANUAL);
      vehicle_info->set_error(COWA::NavMsg::DrivingError::BRAKE_ERROR);
    }
    brake->Enable();
    brake->CmdProfiledVelocity(1);
    if(brake->CmdTarget(0))
    {
      vehicle_info->mutable_drive()->set_driving_mode(COWA::NavMsg::VehicleDrive::MANUAL);
      vehicle_info->set_error(COWA::NavMsg::DrivingError::BRAKE_ERROR);
    }
    if(fabs(clutch->FbkPos()) < 0.01 && fabs(brake->FbkPos()) < 0.01)
    {
      clutch->Disable();
      brake->Disable();
      if(cmd->drive().driving_mode() == COWA::NavMsg::VehicleDrive::STANDBY)
      {
        vehicle_info->mutable_drive()->set_driving_mode(cmd->drive().driving_mode());
        CheckPosStandby = turn->FbkPos();
      }
      vehicle_info->mutable_drive()->set_throttle(0);
    }
  }
  void StateStandby(std::shared_ptr<COWA::NavMsg::VehicleCommond> cmd)
  {
    //刹车，等待进入自动驾驶，如果有错误则退到手动
    ThrottleRequest(0, 1);
    turn->Disable();

    clutch->Enable();
    clutch->CmdProfiledVelocity(1);
    if(clutch->CmdTarget(0.9))
    {
      vehicle_info->mutable_drive()->set_driving_mode(COWA::NavMsg::VehicleDrive::MANUAL);
      vehicle_info->set_error(COWA::NavMsg::DrivingError::BRAKE_ERROR);
    }

    brake->Enable();
    brake->CmdProfiledVelocity(1);
    if(brake->CmdTarget(0.95))
    {
      vehicle_info->mutable_drive()->set_driving_mode(COWA::NavMsg::VehicleDrive::MANUAL);
      vehicle_info->set_error(COWA::NavMsg::DrivingError::BRAKE_ERROR);
    }

    //退出准备模式
    if(fabs(turn->FbkPos() - CheckPosStandby) > 0.1)
    {
      vehicle_info->mutable_drive()->set_driving_mode(COWA::NavMsg::VehicleDrive::MANUAL);
      vehicle_info->set_error(COWA::NavMsg::DrivingError::MANUAL_INTERVENTION);
    }

    //请求进入自动驾驶
    if(cmd->drive().has_driving_mode())
    {
      if(cmd->drive().driving_mode() == COWA::NavMsg::VehicleDrive::AUTO_DRIVE ||
              cmd->drive().driving_mode() == COWA::NavMsg::VehicleDrive::AUTO_SPEED_ONLY ||
              cmd->drive().driving_mode() == COWA::NavMsg::VehicleDrive::AUTO_STEER_ONLY ||
              cmd->drive().driving_mode() == COWA::NavMsg::VehicleDrive::MANUAL)
      {
        if(fabs(brake->FbkPos()) > 0.75 && fabs(clutch->FbkPos()) > 0.75)
          vehicle_info->mutable_drive()->set_driving_mode(cmd->drive().driving_mode());
      }
      vehicle_info->mutable_drive()->set_throttle(-1);
    }
  }
  void StateAutonomous(std::shared_ptr<COWA::NavMsg::VehicleCommond> cmd)
  {
    if(1 == SwithGear(cmd) || !cmd->mutable_drive()->has_driving_mode())
      return;
    if(cmd->drive().driving_mode() == COWA::NavMsg::VehicleDrive::MANUAL)
      vehicle_info->mutable_drive()->set_driving_mode(COWA::NavMsg::VehicleDrive::MANUAL);

    else if(cmd->drive().driving_mode() == COWA::NavMsg::VehicleDrive::AUTO_SPEED_ONLY)
    {
      turn->Disable();
      ControlSpeed(cmd);
    }
    else if(cmd->drive().driving_mode() == COWA::NavMsg::VehicleDrive::AUTO_STEER_ONLY)
    {
      clutch->Disable();
      brake->Disable();
      ControlSteer(cmd);
    }
    else if(cmd->drive().driving_mode() == COWA::NavMsg::VehicleDrive::AUTO_DRIVE)
    {
      ControlSteer(cmd);
      ControlSpeed(cmd);
    }
  }

  int SwithGear(std::shared_ptr<COWA::NavMsg::VehicleCommond> cmd)
  {
    auto gear = vehicle_info->drive().gear();
    if (cmd->drive().gear() == COWA::NavMsg::VehicleDrive::GEAR_PARKING)
    {
      ControlSteer(cmd);
      float pos = 0.8;
      clutch->Enable();
      clutch->CmdProfiledVelocity(1);
      if(clutch->CmdTarget(1))
      {
        vehicle_info->mutable_drive()->set_driving_mode(COWA::NavMsg::VehicleDrive::MANUAL);
        vehicle_info->set_error(COWA::NavMsg::DrivingError::BRAKE_ERROR);
      }
      brake->Enable();
      brake->CmdProfiledVelocity(0.01);
      if(brake->CmdTarget(pos))
      {
        vehicle_info->mutable_drive()->set_driving_mode(COWA::NavMsg::VehicleDrive::MANUAL);
        vehicle_info->set_error(COWA::NavMsg::DrivingError::BRAKE_ERROR);
      }
      if(brake->FbkPos() >= pos * 0.9)
      {
        vehicle_info->mutable_drive()->set_gear(cmd->drive().gear());
      }
      return 1;
    }
    if (gear == cmd->drive().gear() )
      return 0;
    else if (cmd->drive().gear() == COWA::NavMsg::VehicleDrive::GEAR_REVERSE)
    {
      vehicle_info->mutable_drive()->set_gear(cmd->drive().gear());
      return 0;
    }
    else if (cmd->drive().gear() == COWA::NavMsg::VehicleDrive::GEAR_NEUTRAL)
    {
      vehicle_info->mutable_drive()->set_gear(cmd->drive().gear());
      return 0;
    }
    else if (cmd->drive().gear() == COWA::NavMsg::VehicleDrive::GEAR_DRIVE)
    {
      vehicle_info->mutable_drive()->set_gear(cmd->drive().gear());
      return 0;
    }
    return -1;
  }
  int Startup(std::shared_ptr<COWA::NavMsg::VehicleCommond> cmd)
  {
    ControlSteer(cmd);
    clutch->Enable();
    if(clutch->FbkPos() >= 0.5)
      clutch->CmdProfiledVelocity(0.2);
    else
      clutch->CmdProfiledVelocity(0.05);
    
    if(clutch->CmdTarget(0))
    {
      vehicle_info->mutable_drive()->set_driving_mode(COWA::NavMsg::VehicleDrive::MANUAL);
      vehicle_info->set_error(COWA::NavMsg::DrivingError::BRAKE_ERROR);
    }

    brake->Enable();
    brake->CmdProfiledVelocity(0.5);
    if(brake->CmdTarget(0))
    {
      vehicle_info->mutable_drive()->set_driving_mode(COWA::NavMsg::VehicleDrive::MANUAL);
      vehicle_info->set_error(COWA::NavMsg::DrivingError::BRAKE_ERROR);
    }
    ThrottleRequest(cmd->mutable_drive()->throttle(), 1);
    if(clutch->FbkPos() < 0.1)
      return 0;

    return 1;
  }
  int ControlSteer(std::shared_ptr<COWA::NavMsg::VehicleCommond> cmd)
  {
    if(!cmd->drive().has_steer())
      return -1;
    turn->Enable();
    turn->CmdProfiledVelocity(steer_ratio / cmd->drive().steer_speed());
    //有错误，则退到手动驾
    if(turn->CmdTarget(cmd->drive().steer() * steer_ratio ))
    {
      vehicle_info->mutable_drive()->set_driving_mode(COWA::NavMsg::VehicleDrive::MANUAL);
      vehicle_info->set_error(COWA::NavMsg::DrivingError::STEER_ERROR);
    }
  }
  int ControlSpeed(std::shared_ptr<COWA::NavMsg::VehicleCommond> cmd)
  {
    if(!cmd->drive().has_speed_mode())
      return -1;
    static int state = 0;
    if(cmd->drive().speed_mode() == COWA::NavMsg::VehicleDrive::SpeedMode::VehicleDrive_SpeedMode_SPEED)
    {
      auto v = cmd->drive().speed();
      auto clamp = [](double v, double maxV, double minV) { if (v > maxV) return maxV; if (v < minV) return minV; return v;};
      double target = clamp(v, this->max_speed, -this->max_speed);

      if(target > 0.3)
      {
        double err = target - fabs(vehicle_info->mutable_drive()->speed());
        double out = (target + err * this->speedGain) * this->speedToThrottle;
        if (this->throttleOut + this->throttleAcc / this->ctrlHz  < out)
          this->throttleOut = this->throttleOut + this->throttleAcc / this->ctrlHz;
        else if (this->throttleOut - this->throttleDec / this->ctrlHz > out)
          this->throttleOut = this->throttleOut - this->throttleDec / this->ctrlHz;
        else
          this->throttleOut = out;
      }
      else
      {
        this->throttleOut = -0.8;
      }
      
        
      this->throttleOut = clamp(this->throttleOut, 1.0, -1.0);

      if (this->throttleOut < -0.1)
      {
        clutch->Enable();
        clutch->CmdProfiledVelocity(1);
        if(clutch->CmdTarget(0.8))
        {
          vehicle_info->mutable_drive()->set_driving_mode(COWA::NavMsg::VehicleDrive::MANUAL);
          vehicle_info->set_error(COWA::NavMsg::DrivingError::BRAKE_ERROR);
        }

        brake->Enable();
        brake->CmdProfiledVelocity(1);
        if(brake->CmdTarget(fabs(this->throttleOut)))
        {
          vehicle_info->mutable_drive()->set_driving_mode(COWA::NavMsg::VehicleDrive::MANUAL);
          vehicle_info->set_error(COWA::NavMsg::DrivingError::BRAKE_ERROR);
        }
        vehicle_info->mutable_drive()->set_throttle(this->throttleOut);
        ThrottleRequest(0, 1);
      }
      else if (this->throttleOut > 0.1)
      {
        if(fabs(brake->FbkPos()) > 0.02 || fabs(clutch->FbkPos()) > 0.02)// && cmd->mutable_drive()->throttle() >= 0)
        {
          clutch->Enable();
          clutch->CmdProfiledVelocity(0.1);
          if(clutch->CmdTarget(0))
          {
            vehicle_info->mutable_drive()->set_driving_mode(COWA::NavMsg::VehicleDrive::MANUAL);
            vehicle_info->set_error(COWA::NavMsg::DrivingError::BRAKE_ERROR);
          }

          brake->Enable();
          brake->CmdProfiledVelocity(1);
          if(brake->CmdTarget(0))
          {
            vehicle_info->mutable_drive()->set_driving_mode(COWA::NavMsg::VehicleDrive::MANUAL);
            vehicle_info->set_error(COWA::NavMsg::DrivingError::BRAKE_ERROR);
          }
        }
        else 
        {
          ThrottleRequest(this->throttleOut, 1);
          vehicle_info->mutable_drive()->set_throttle(this->throttleOut);
        }
      }
      else
      {
        ThrottleRequest(0, 1);
      }
      
    }
    else if(cmd->drive().speed_mode() == COWA::NavMsg::VehicleDrive::SpeedMode::VehicleDrive_SpeedMode_TORQUE)
    {
      auto t = cmd->drive().throttle();
      auto v = cmd->drive().speed();
      if (t < -0.01)
      {
        clutch->Enable();
        clutch->CmdProfiledVelocity(1);
        if(clutch->CmdTarget(fabs(t * 1.5)))
        {
          vehicle_info->mutable_drive()->set_driving_mode(COWA::NavMsg::VehicleDrive::MANUAL);
          vehicle_info->set_error(COWA::NavMsg::DrivingError::BRAKE_ERROR);
        }

        brake->Enable();
        if(brakeStage.size() < 3)
        {
          brakeStage[0] = 0.01;
          brakeStage[1] = 0.1;
          brakeStage[2] = 0.4;
        }
        
        if(v <= 0.01)
          brake->CmdProfiledVelocity(brakeStage[2]);
        else if(v < 0.06)
          brake->CmdProfiledVelocity(brakeStage[1]);
        else
          brake->CmdProfiledVelocity(brakeStage[0]);

        if(brake->CmdTarget(fabs(t)))
        {
          vehicle_info->mutable_drive()->set_driving_mode(COWA::NavMsg::VehicleDrive::MANUAL);
          vehicle_info->set_error(COWA::NavMsg::DrivingError::BRAKE_ERROR);
        }

        ThrottleRequest(0, 1);
        vehicle_info->mutable_drive()->set_throttle(t);
      }
      else if(t > 0.01)
      {
        if(fabs(brake->FbkPos()) > 0.1 || fabs(clutch->FbkPos()) > 0.1)
        {
          clutch->Enable();
          if(vehicle_info->drive().speed() > 0.5)
            clutch->CmdProfiledVelocity(0.3);
          else if(vehicle_info->drive().speed() > 0.3)
            clutch->CmdProfiledVelocity(0.2);
          else 
            clutch->CmdProfiledVelocity(0.1); 
          if(clutch->CmdTarget(0))
          {
            vehicle_info->mutable_drive()->set_driving_mode(COWA::NavMsg::VehicleDrive::MANUAL);
            vehicle_info->set_error(COWA::NavMsg::DrivingError::BRAKE_ERROR);
          }

          brake->Enable();
          brake->CmdProfiledVelocity(1);
          if(brake->CmdTarget(0))
          {
            vehicle_info->mutable_drive()->set_driving_mode(COWA::NavMsg::VehicleDrive::MANUAL);
            vehicle_info->set_error(COWA::NavMsg::DrivingError::BRAKE_ERROR);
          }
          vehicle_info->mutable_drive()->set_throttle(0);
        }
        else 
        {
          ThrottleRequest(t, 1);
          vehicle_info->mutable_drive()->set_throttle(t);
        }
      }
      else
      {
          ThrottleRequest(0, 1);
      }
      
    }
    //有错误，则退到手动驾驶
    if(brake->GetFault() || clutch->GetFault())
    {
      vehicle_info->mutable_drive()->set_driving_mode(COWA::NavMsg::VehicleDrive::MANUAL);
      vehicle_info->set_error(COWA::NavMsg::DrivingError::BRAKE_ERROR);
    }
  }
  int ThrottleRequest(float cmd, int mode)
  {
    cmd += throttleOffset;
    if(mode == 1)
    {
      if(cmd > 0.8)
          cmd = 0.8;
      else if(cmd < 0)
          cmd = 0;

      frame_throttle.data[0] = 0x01;
      frame_throttle.data[1] = (cmd * 100);
    }
    else
    {
      frame_throttle.data[0] = 0x00;
        frame_throttle.data[1] = 0;
    }
    can_drive[0]->Write(frame_throttle);
    return 0;
  }

};
REGISTER_CAN(Hulk, Hulk);