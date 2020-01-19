#include <common/dispatch.h>
#include <msgs/chassis.pb.h>
#include "zl18c.h"
#include "vehicle.h"
#include "elmo.hpp"
#include "common/can.hpp"

using namespace COWA::drivers;


class Zl18c : public BaseVehicle
{
private:
  std::shared_ptr<COWA::drivers::Elmo> turn;
  CanStamped frame_heart, frame_io;

  double throttleOut = 0;
  int ctrlHz = 50;
  double max_speed;
  double  speedGain;
  double speedToThrottle;
  double throttleAcc;
  double throttleDec;

  uint8_t driving_mode = 0;
  int auto_feedback = 0;

  double steerCmd;
  double speedCmd;

  int workthrottle = 0;
  int workThrottle = 0;
  int offset = 0;
  double turnKp = 0;
  double trunKi = 0;
  int air_blower_set = 0;
  double steer_ratio;


public:
  Zl18c()
  {
    CRINFO<< "Zl18c init";
    auto turn_slave = param->query("chassis/turn");

    workthrottle = param->value<int>("workthrottle", 300);
    offset = param->value<int>("offset", 2889);
    turnKp = param->value<double>("turnKp", 15.0);
    trunKi = param->value<double>("trunKi", 1.0);
    steer_ratio = param->value<double>("chassis/turn_ratio", 1);
    max_speed = param->value<double>("chassis/max_speed", 0.5);
    speedGain = param->value<double>("chassis/speedGain", 0.5);
    speedToThrottle = param->value<double>("chassis/speedToThrottle", 0.1);
    throttleAcc = param->value<double>("chassis/throttleAcc", 2);
    throttleDec = param->value<double>("chassis/ThrottleDec", 5);
    CRINFO<<"INIT OVER";

    vehicle_info->add_wheel_speed(0);
    vehicle_info->set_wheel_speed(0, NAN);
    vehicle_info->mutable_drive()->set_driving_mode(COWA::NavMsg::VehicleDrive::MANUAL);
  
    frame_heart.can_id = 31;
    frame_heart.can_dlc = 8;
    memset(frame_heart.data, 0, 8);

    frame_io.can_id = 30;
    frame_io.can_dlc = 8;
    memset(frame_io.data, 0, 8);

    turn.reset(new COWA::drivers::Elmo(can_drive[0],turn_slave,false));

    turn->Setup();
    turn->Disable();
    
    can_drive[0]->Register(10, [this](const CanStamped &frame) -> void {
      this->gear_status_10(frame);//00a
    });
    can_drive[0]->Register(40, [this](const CanStamped &frame) -> void {
      this->car_speed_40(frame);//028
    });
    can_drive[0]->Register(0x1A0, [this](const CanStamped &frame) -> void {
      this->ifm_position_1A0(frame);
    });
  }

  int UpdateCMD(std::shared_ptr<COWA::NavMsg::VehicleCommond> cmd) override
  {
    frame_heart.timestamp = (COWA::Time::Now().ToNanosecond());
    can_drive[0]->Write(frame_heart);
    frame_io.timestamp = (COWA::Time::Now().ToNanosecond());
    can_drive[0]->Write(frame_io);
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
    return 0;
  }

  int UpdateFBK(std::shared_ptr<COWA::NavMsg::VehicleInfo> fbk) override
  {
    return 0;
  }

  int DirectCtrl(std::shared_ptr<COWA::NavMsg::VehicleCommond> cmd) override
  {
    return 0;
    if(cmd->mutable_drive()->gear() == COWA::NavMsg::VehicleDrive_GearPosition_GEAR_NEUTRAL)
    { 
      throttle_request(0);
      turn->Disable();
    }
    else
    {
      if(cmd->mutable_drive()->gear() == COWA::NavMsg::VehicleDrive_GearPosition_GEAR_REVERSE)
        gear_request(COWA::NavMsg::VehicleDrive_GearPosition_GEAR_REVERSE);
      else
        gear_request(COWA::NavMsg::VehicleDrive_GearPosition_GEAR_DRIVE);

      turn->Enable();
      if(turn->CmdTarget(cmd->drive().steer() * steer_ratio ) )
      {
        vehicle_info->mutable_drive()->set_driving_mode(COWA::NavMsg::VehicleDrive::MANUAL);
        vehicle_info->set_error(COWA::NavMsg::DrivingError::STEER_ERROR);
      }
      throttle_request(cmd->drive().throttle());
    }

    frame_heart.timestamp = (COWA::Time::Now().ToNanosecond());
    can_drive[0]->Write(frame_heart);
    frame_io.timestamp = (COWA::Time::Now().ToNanosecond());
    can_drive[0]->Write(frame_io);
    return 0;
  }

  void reset()
  {
    steerCmd = 0;
    speedCmd = 0;
  }

  void gear_status_10(const CanStamped &frame)
  {
    const uint8_t null = 0;
    const uint8_t drive = 1;
    const uint8_t reverse = 2;
    int gearValue = (((frame.data[0] >> 2) & 0x01) << 1) | ((frame.data[0] >> 1) & 0x01);
    if (gearValue == drive)
      vehicle_info->mutable_drive()->set_gear(COWA::NavMsg::VehicleDrive::GEAR_DRIVE);
    else if(gearValue == reverse)
      vehicle_info->mutable_drive()->set_gear(COWA::NavMsg::VehicleDrive::GEAR_REVERSE);
    else if(gearValue == null)
      vehicle_info->mutable_drive()->set_gear(COWA::NavMsg::VehicleDrive::GEAR_NEUTRAL);
  }

  void car_speed_40(const CanStamped &frame)
  {
    can_recv* recv_data = (can_recv*)(frame.data);
    int16_t vehicle_speed = recv_data->can_recv_byte0_1.vehicle_speed;
    double speed = vehicle_speed / 3600.0;// m/s;
    if (vehicle_info->mutable_drive()->gear() == COWA::NavMsg::VehicleDrive_GearPosition_GEAR_REVERSE)
      vehicle_info->mutable_drive()->set_speed(-speed);
    else
      vehicle_info->mutable_drive()->set_speed(speed);
    vehicle_info->set_timestamp_speed(frame.timestamp);
  }

  void ifm_position_1A0(const CanStamped &frame)
  {
    int32_t tmp;
    uint32_t value = (frame.data[3] << 24) | (frame.data[2] << 16) | (frame.data[1] << 8) | frame.data[0];
    if (value > 4096)
    {
      tmp = value % 4096;
    }
    else
    {
      value = 4096 - value;
      tmp =  -(value % 4096);
    }
    double v = -(tmp - offset) * 2 * M_PI / 4096;
    vehicle_info->mutable_drive()->set_steer(v);
    vehicle_info->set_timestamp_steer(frame.timestamp);
  }

  // control BCM
  int turn_signal_request(COWA::NavMsg::VehicleBCM::TurnSignal target)
  {
    const bool right_light = false;//turn right light judge
    const bool left_light = false;//turn left light judge

    io_ctrl* can_io_ctrl = (io_ctrl*)(frame_io.data);

    if (target == COWA::NavMsg::VehicleBCM_TurnSignal_TURN_RIGHT)
    {
      can_io_ctrl->io_ctrl_byte5.right_signal = 1;
      can_io_ctrl->io_ctrl_byte5.left_signal = 0;
      can_io_ctrl->io_ctrl_byte5.warning_signal = 0;
    }
    else if (target == COWA::NavMsg::VehicleBCM_TurnSignal_TURN_LEFT)
    {
      can_io_ctrl->io_ctrl_byte5.right_signal = 0;
      can_io_ctrl->io_ctrl_byte5.left_signal = 1;
      can_io_ctrl->io_ctrl_byte5.warning_signal = 0;
    }
    else if(target == COWA::NavMsg::VehicleBCM_TurnSignal_TURN_RIGHT && target == COWA::NavMsg::VehicleBCM_TurnSignal_TURN_LEFT)
    {
      can_io_ctrl->io_ctrl_byte5.warning_signal = 1;
      can_io_ctrl->io_ctrl_byte5.right_signal = 0;
      can_io_ctrl->io_ctrl_byte5.left_signal = 0;
    }
    else
    {
      can_io_ctrl->io_ctrl_byte5.right_signal = 0;
      can_io_ctrl->io_ctrl_byte5.left_signal = 0;
      can_io_ctrl->io_ctrl_byte5.warning_signal = 0;
    }
    return 0;
  }

  //control car

  int gear_request(COWA::NavMsg::VehicleDrive::GearPosition target)
  {
    if (vehicle_info->mutable_drive()->speed() < 0.1)
    {
      io_ctrl *can_io_ctrl = (io_ctrl *)(frame_io.data);

      switch (target)
      {
        case COWA::NavMsg::VehicleDrive_GearPosition_GEAR_NEUTRAL:
          can_io_ctrl->io_ctrl_byte4.self_driving_enable = 0;
          can_io_ctrl->io_ctrl_byte4.drive = 0;
          can_io_ctrl->io_ctrl_byte4.reverse = 0;
          can_io_ctrl->io_ctrl_byte4.brake = 0;
          break;
        case COWA::NavMsg::VehicleDrive_GearPosition_GEAR_DRIVE:
          can_io_ctrl->io_ctrl_byte4.self_driving_enable = 1;
          can_io_ctrl->io_ctrl_byte4.drive = 1;
          can_io_ctrl->io_ctrl_byte4.reverse = 0;
          can_io_ctrl->io_ctrl_byte4.brake = 0;
          break;
        case COWA::NavMsg::VehicleDrive_GearPosition_GEAR_REVERSE:
          can_io_ctrl->io_ctrl_byte4.self_driving_enable = 1;
          can_io_ctrl->io_ctrl_byte4.drive = 0;
          can_io_ctrl->io_ctrl_byte4.reverse = 1;
          can_io_ctrl->io_ctrl_byte4.brake = 0;
          break;
        // case COWA::NavMsg::VehicleDrive_GearPosition_GEAR_PARKING:
        //   can_io_ctrl->io_ctrl_byte4.IDM_GearReq = 0xB;
        //   break;
        default:
          can_io_ctrl->io_ctrl_byte4.self_driving_enable = 0;
          can_io_ctrl->io_ctrl_byte4.drive = 0;
          can_io_ctrl->io_ctrl_byte4.reverse = 0;
          can_io_ctrl->io_ctrl_byte4.brake = 0;
      }
    }
    return 0;
  }

  int throttle_request(double value)
  {
    io_ctrl *can_io_ctrl = (io_ctrl *)(frame_io.data);
    if(value > 1)
      value = 1;
    else if(value < 0)
      value = 0;
    int16_t speed = value * 2400 * 1.6 + 700 + workThrottle;
    if (speed > 4000)
      speed = 4000;
    can_io_ctrl->io_ctrl_byte0_1.speed_pedal = speed;
    return 0;
  }

  int speed_request(double value)
  {
    return 0;
  }

  int brake_request(double value) /* 0 - 1, for 1 is 100% */
  {
    return 0;
  }

  double turnSum = 0;
  int steer_request(double angle)
  {
    CRINFO<<"angle:"<<angle;
    int32_t ppr = (1<<17);
    double turnAngle = vehicle_info->mutable_drive()->steer();
    double err = (angle) - turnAngle;
    CRINFO<<"ERR:"<<err;
    double temp_kp = 0;
    double turnmax = 1;
    if(fabs(err) < 0.005 )
    {
      err = 0;
      turnSum = 0;
    }
    else if (fabs(err) < 0.05) 
    {
      temp_kp = this->turnKp * 0.5 ;
      turnmax = 0.4;
    }
    else if (fabs(err) < 0.1)
    {
      temp_kp = this->turnKp * 0.7 ;
      turnmax = 0.7;
    }
    else if (fabs(err) < 0.2)
    {
      temp_kp = this->turnKp * 0.9 ;
      turnmax = 0.9; 
    }
    else
    {
      temp_kp = this->turnKp;
      turnmax = 1;
    }

    turnSum += err * this->trunKi * temp_kp;
    if(turnSum >turnmax)
      turnSum = turnmax;
    else if(turnSum < -turnmax)
      turnSum = -turnmax;

    int32_t vel = (err * temp_kp + turnSum) ;//* ppr;
    return turn->CmdTarget(vel);
  }

  //autonomous driving car

  int work_request(int mode)
  {
    io_ctrl* can_io_ctrl = (io_ctrl*)(frame_io.data);
    if(mode)
    {
      this->workThrottle = this->workthrottle;
      can_io_ctrl->io_ctrl_byte5.work_start = 1;
      can_io_ctrl->io_ctrl_byte5.work_stop= 0;
      can_io_ctrl->io_ctrl_byte4.engine_speed_set = this->air_blower_set;
    }
    else
    {
      this->workThrottle = 0;
      can_io_ctrl->io_ctrl_byte5.work_start = 0;
      can_io_ctrl->io_ctrl_byte5.work_stop= 1;
      can_io_ctrl->io_ctrl_byte4.engine_speed_set = 0;
    }
    return 0;
  }

  int request(std::string req)
  {

    io_ctrl* can_io_ctrl = (io_ctrl*)(frame_io.data);

    if(req == "AIR1" && this->air_blower_set != 0)
      this->air_blower_set = 0;
    else if(req == "AIR2" && this->air_blower_set != 1)
      this->air_blower_set = 1;
    else if(req == "AIR3" && this->air_blower_set != 2)
      this->air_blower_set = 2;
    else if(req == "AIR4" && this->air_blower_set != 3)
      this->air_blower_set = 3;

    if (req == "WORK1")
      can_io_ctrl->io_ctrl_byte6.encoder_count = -5;
    else if (req == "WORK2")
      can_io_ctrl->io_ctrl_byte6.encoder_count = -3;
    else if (req == "WORK3")
      can_io_ctrl->io_ctrl_byte6.encoder_count = -1;
    else if (req == "WORK4")
      can_io_ctrl->io_ctrl_byte6.encoder_count = 0;
    else if (req == "WORK5")
      can_io_ctrl->io_ctrl_byte6.encoder_count = 2;
    else if (req == "WORK6")
      can_io_ctrl->io_ctrl_byte6.encoder_count = 4;
    else if (req == "WORK7")
      can_io_ctrl->io_ctrl_byte6.encoder_count = 6;
    else if (req == "WORK8")
      can_io_ctrl->io_ctrl_byte6.encoder_count = 8;
    else if (req == "WORK9")
      can_io_ctrl->io_ctrl_byte6.encoder_count = 10;

    return 0;
  }

  int bcm_control(std::shared_ptr<COWA::NavMsg::VehicleCommond> vehicle_cmd)
  {
    turn_signal_request(vehicle_cmd->mutable_bcm()->turn_signal());
  }

  private:
  void StateManual(std::shared_ptr<COWA::NavMsg::VehicleCommond> cmd)
  {
    turn->Disable();
    if(vehicle_info->mutable_drive()->gear() != COWA::NavMsg::VehicleDrive::GEAR_NEUTRAL)
      gear_request(COWA::NavMsg::VehicleDrive::GEAR_NEUTRAL);
    throttle_request(0);

    if(cmd->drive().driving_mode() == COWA::NavMsg::VehicleDrive::STANDBY)
      {
        vehicle_info->mutable_drive()->set_driving_mode(cmd->drive().driving_mode());
      }
  }
  void StateStandby(std::shared_ptr<COWA::NavMsg::VehicleCommond> cmd)
  {
    if(cmd->drive().has_driving_mode())
    {
      if(cmd->drive().driving_mode() == COWA::NavMsg::VehicleDrive::AUTO_DRIVE ||
              cmd->drive().driving_mode() == COWA::NavMsg::VehicleDrive::AUTO_SPEED_ONLY ||
              cmd->drive().driving_mode() == COWA::NavMsg::VehicleDrive::AUTO_STEER_ONLY ||
              cmd->drive().driving_mode() == COWA::NavMsg::VehicleDrive::MANUAL)
      {
        vehicle_info->mutable_drive()->set_driving_mode(cmd->drive().driving_mode());
      }
    }
  }
  void StateAutonomous(std::shared_ptr<COWA::NavMsg::VehicleCommond> cmd)
  {
    if(1 == SwithGear(cmd) || !cmd->mutable_drive()->has_driving_mode())
      return;
    CRINFO<<"cmd->drive().driving_mode()::"<<cmd->drive().driving_mode();
    if(cmd->drive().driving_mode() == COWA::NavMsg::VehicleDrive::MANUAL)
      vehicle_info->mutable_drive()->set_driving_mode(COWA::NavMsg::VehicleDrive::MANUAL);

    else if(cmd->drive().driving_mode() == COWA::NavMsg::VehicleDrive::AUTO_SPEED_ONLY)
    {
      turn->Disable();
      ControlSpeed(cmd);
    }
    else if(cmd->drive().driving_mode() == COWA::NavMsg::VehicleDrive::AUTO_STEER_ONLY)
    {
      throttle_request(0);
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
      throttle_request(0);
      return 0;
    }
    if (gear == cmd->drive().gear())
      return 0;
    else if (cmd->drive().gear() == COWA::NavMsg::VehicleDrive::GEAR_NEUTRAL)
    {
      gear_request(COWA::NavMsg::VehicleDrive::GEAR_NEUTRAL);

    }
    else if (cmd->drive().gear() == COWA::NavMsg::VehicleDrive::GEAR_DRIVE)
    {
      gear_request(COWA::NavMsg::VehicleDrive::GEAR_DRIVE);
    }
    else if (cmd->drive().gear() == COWA::NavMsg::VehicleDrive::GEAR_REVERSE)
    {
      gear_request(COWA::NavMsg::VehicleDrive::GEAR_REVERSE);
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
        throttle_request(0);

      else if (this->throttleOut > 0.1)
      {
        throttle_request(this->throttleOut);
        vehicle_info->mutable_drive()->set_throttle(this->throttleOut);
      }
    }
    return 0;
  }
  int ControlSteer(std::shared_ptr<COWA::NavMsg::VehicleCommond> cmd)
  {
    if(!cmd->drive().has_steer())
      return -1;
    turn->Enable();
    steer_request(cmd->drive().steer() * steer_ratio);
    return 0;
  }
};

REGISTER_CAN(Zl18c, Zl18c);