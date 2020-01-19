
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <fcntl.h>
#include <sys/epoll.h>
#include <sys/poll.h>
//#include <catch2/catch.hpp>
#include "conti_radar.h"
#include <msgs/radar.pb.h>
#include <msgs/pose.pb.h>
#include <common/dispatch.h>
#include <sstream>
#include <Eigen/Eigen>
#include <map>
#include "common/can.hpp"
//#define CATCH_CONFIG_MAIN

using namespace COWA::SensorMsg;
using namespace COWA::drivers;

class RadarProcess
{
private:
  std::shared_ptr<COWA::Parameter> param;
  std::shared_ptr<COWA::SensorMsg::ContiRadar> conti_radar;
  std::shared_ptr<COWA::Writer<COWA::SensorMsg::ContiRadar>> fbk;
  std::shared_ptr<COWA::util::ObjectPool<COWA::SensorMsg::ContiRadar> > object_pool_;
  CanStamped conf_frame;
  std::string frame_id;
  uint8_t is_configured = 0;
  uint8_t configure_times = 0;
  uint64_t last_nsec_ = 0;
  uint32_t sequence = 0;
  int send_interval = 50 * 1000; // 50ms
  int  SensorID = 0;
  int cnt = 0;
  std::shared_ptr<CANDispatch> can_drive;
  std::map<uint8_t, uint8_t> id_idx;
  uint8_t index = 0;
  template <class T>
  T Param(YAML::Node &node, const std::string &name, const T t = T())
  {
    auto x = node[name];
    if (!x)
      return t;
    return x.as<T>();
  }
public:
  RadarProcess(std::shared_ptr<CANDispatch> can_, std::shared_ptr<COWA::Node> node_, YAML::Node config_, int id_)
  {
    can_drive = can_;
    conti_radar = std::make_shared<COWA::SensorMsg::ContiRadar>();

    frame_id = Param<std::string>(config_, "frame_id", "/front/radar");
    auto MaxDistance_valid = Param<bool>(config_, "max_distance_valid", true);
    auto SensorID_valid = Param<bool>(config_, "sensor_id_valid", false);
    auto RadarPower_valid = Param<bool>(config_, "radar/radar_power_valid", false);
    auto output_type_valid = Param<bool>(config_, "utput_type_valid", true);
    auto SendQuality_valid = Param<bool>(config_, "send_quality_valid", true);
    auto SendExtInfo_valid = Param<bool>(config_, "send_ext_info_valid", true);
    auto SortIndex_valid = Param<bool>(config_, "sort_index_valid", false);
    auto StoreInNVM_valid = Param<bool>(config_, "store_in_nvm_valid", true);
    auto MaxDistance =  Param<int>(config_, "max_distance", 250);
    auto OutputType = Param<uint16_t>(config_, "output_type", OUTPUT_TYPE_OBJECTS);
    uint16_t out;
    switch (OutputType) {
    case OUTPUT_TYPE_NONE:
      out = 0x0;
      break;
    case OUTPUT_TYPE_OBJECTS:
      out = 0x1;
      break;
    case OUTPUT_TYPE_CLUSTERS:
      out = 0x2;
      break;
    default:
      out = 0x3;
    }
    auto RadarPower = Param<int>(config_, "radar_power", 0);
    auto CtrlRelay_valid = Param<bool>(config_, "ctrl_relay_valid", false);
    auto CtrlRelay = Param<int>(config_, "ctrl_relay", 0);
    auto SendQuality = Param<bool>(config_, "send_quality", true);
    auto SendExtInfo = Param<bool>(config_, "send_ext_info", true);
    auto SortIndex = Param<int>(config_, "sort_index", 0);
    auto StoreInNVM = Param<int>(config_, "store_in_nvm", 0);
    auto RCS_Threshold = Param<uint16_t>(config_, "rcs_threshold", RCS_THRESHOLD_STANDARD);
    auto RCS_Threshold_Vaild = Param<bool>(config_, "rcs_threshold_valid", true);
    uint16_t out_;    
    switch (RCS_Threshold) {
    case RCS_THRESHOLD_STANDARD:
      out_ = 0x0;
      break;
    case RCS_THRESHOLD_HIGH_SENSITIVITY:
      out_ = 0x1;
      break;
    default:
      out_ = 0x2;
    }


    fbk = node_->CreateWriter<COWA::SensorMsg::ContiRadar>(frame_id);
    object_pool_ = std::make_shared<COWA::util::ObjectPool<COWA::SensorMsg::ContiRadar> > (10);
    SensorID = id_;
    
    conf_frame.can_id = 0x200  + 16 * SensorID;
    conf_frame.can_dlc = 8;

    radar_conf*  conti_radar_conf = (radar_conf*)(conf_frame.data);
    conti_radar_conf->radar_conf_byte0.RadarCfg_MaxDistance_valid = MaxDistance_valid;
    conti_radar_conf->radar_conf_byte0.RadarCfg_SensorID_valid = SensorID_valid;
    conti_radar_conf->radar_conf_byte0.RadarCfg_RadarPower_valid = RadarPower_valid;
    conti_radar_conf->radar_conf_byte0.RadarCfg_OutputType_valid = output_type_valid;
    conti_radar_conf->radar_conf_byte0.RadarCfg_SendQuality_valid = SendQuality_valid;
    conti_radar_conf->radar_conf_byte0.RadarCfg_SendExtInfo_valid = SendExtInfo_valid;
    conti_radar_conf->radar_conf_byte0.RadarCfg_SortIndex_valid = SortIndex_valid;
    conti_radar_conf->radar_conf_byte0.RadarCfg_StoreInNVM_valid = StoreInNVM_valid;

    MaxDistance /= 2; //resolution: 2m
    uint8_t low = static_cast<uint8_t>(MaxDistance & 0x03);
    uint8_t high = static_cast<uint8_t>(MaxDistance >> 2);

    conti_radar_conf->radar_conf_byte1.RadarCfg_MaxDistance_H = high;
    conti_radar_conf->radar_conf_byte2.RadarCfg_MaxDistance_L = low;
    conti_radar_conf->radar_conf_byte4.RadarCfg_SensorID = SensorID;
    conti_radar_conf->radar_conf_byte4.RadarCfg_RadarPower = RadarPower;
    conti_radar_conf->radar_conf_byte4.RadarCfg_OutputType = out;
    conti_radar_conf->radar_conf_byte5.RadarCfg_CtrlRelay = CtrlRelay;
    conti_radar_conf->radar_conf_byte5.RadarCfg_CtrlRelay_valid = CtrlRelay_valid;
    conti_radar_conf->radar_conf_byte5.RadarCfg_SendExtInfo = SendExtInfo;
    conti_radar_conf->radar_conf_byte5.RadarCfg_SendQuality = SendQuality;
    conti_radar_conf->radar_conf_byte5.RadarCfg_SortIndex = SortIndex;
    conti_radar_conf->radar_conf_byte5.RadarCfg_StoreInNVM = StoreInNVM;
    conti_radar_conf->radar_conf_byte6.RadarCfg_RCS_Threshold = out_;
    conti_radar_conf->radar_conf_byte6.RadarCfg_RCS_Threshold_valid = RCS_Threshold_Vaild;



    CRTRACE<< "radar: config frame: "
              <<std::hex<< (conf_frame.data[0])<<" "
              <<std::hex<< (conf_frame.data[1])<<" " 
              <<std::hex<< (conf_frame.data[2])<<" " 
              <<std::hex<< (conf_frame.data[3])<<" " 
              <<std::hex<< (conf_frame.data[4])<<" " 
              <<std::hex<< (conf_frame.data[5])<<" " 
              <<std::hex<< (conf_frame.data[6])<<" " 
              <<std::hex<< (conf_frame.data[7]);

    //send configure
    can_drive->Write(conf_frame);
    is_configured = 0;
    // receive
    can_drive->Register(0x201 + 16 * SensorID, [this](const struct CanStamped &frame) -> void {
      this->radar_status_check_201(frame);
    });
    can_drive->Register(0x60a + 16 * SensorID, [this](const struct CanStamped &frame) -> void {
      this->radar_conti_status_60A(frame);
    });  
    if(OutputType == OUTPUT_TYPE_OBJECTS)
    {

      can_drive->Register(0x60B + 16 * SensorID, [this](const struct CanStamped &frame) -> void {
        this->radar_conti_obs_60B(frame);
      });
      CRTRACE << "radar: outputs type is objects, id 201 60b 60a is registered";

      if(SendQuality == true)
      {
        can_drive->Register(0x60c + 16 * SensorID, [this](const struct CanStamped &frame) -> void {
          this->radar_conti_quality_60C(frame);
        });
      CRTRACE << "radar: need send-quality, id 60c is registered";
      }

      if(SendExtInfo == true)
      {
        can_drive->Register(0x60d + 16 * SensorID, [this](const struct CanStamped &frame) -> void {
          this->radar_conti_obs_extended_60D(frame);
        });
        CRTRACE << "radar: need send-ext-info, id 60d is registered";
      }
    }
    else if(OutputType == OUTPUT_TYPE_CLUSTERS)
    {
      can_drive->Register(0x600 + 16 * SensorID, [this](const struct CanStamped &frame) -> void {
        this->radar_conti_cluster_status_600(frame);
      });
      can_drive->Register(0x701 + 16 * SensorID, [this](const struct CanStamped &frame) -> void {
        this->radar_conti_cluster_701(frame);
      });
      can_drive->Register(0x702 + 16 * SensorID, [this](const struct CanStamped &frame) -> void {
        this->radar_conti_cluster_quality_702(frame); 
      });
      CRTRACE << "radar: outputs type is clusters, id 201 60a 600 701 702 is registered";
    }
  }
  void PoseCallback(const std::shared_ptr<COWA::NavMsg::PoseStamped> &msg)
  {
      if(frame_id.find("rear") != frame_id.npos)
        return;
      uint64_t now_nsec = COWA::Time().Now().ToNanosecond();
      if (last_nsec_ != 0 && (now_nsec - last_nsec_) < send_interval)
        return;
      last_nsec_ = now_nsec;
      float speed = static_cast<float>(msg->velocity().linear().x());
      float yaw_rate = static_cast<float>(msg->velocity().angular().z() * 180.0f / M_PI);

      //CRTRACE<<frame_id<< " v "<<speed <<" yaw rate "<< yaw_rate;
      struct CanStamped frame1, frame2;
      frame1.can_dlc = 8;
      frame1.can_id = 0x300 + 16 * SensorID;
      frame2.can_dlc = 8;
      frame2.can_id = 0x301 + 16 * SensorID;

      speed_input* speed_in = (speed_input*)frame1.data;
      uint16_t speed_resulotion = speed / 0.02;
      speed_in->speed_input_byte0.speedDirection = 1;
      speed_in->speed_input_byte0.speed_H = (speed_resulotion >> 8) & 0x1F;
      speed_in->speed_input_byte1.speed_L = speed_resulotion & 0xFF;

      yaw_rate_input* yaw_rate_in = (yaw_rate_input*)frame2.data;
      int16_t yaw_resulotion = yaw_rate / 0.01;

      yaw_rate_in->yaw_rate_input_byte0.yaw_rate_H = (yaw_resulotion >> 8) & 0xFF;
      yaw_rate_in->yaw_rate_input_byte1.yaw_rate_L = yaw_resulotion & 0xFF;

      
      can_drive->Write(frame1);
      //can_drive->Write(frame2);
  }
  int Update()
  {
    return 0;
  }
  void radar_status_check_201(const struct CanStamped &frame)
  {
    radar_status_check *conti_check = (radar_status_check *)(frame.data);
    uint16_t max_distnace = conti_check->radar_status_check_byte1.RadarState_MaxDistanceCfg_H << 2 + conti_check->radar_status_check_byte2.RadarState_MaxDistanceCfg_L;
    uint8_t out_type = conti_check->radar_status_check_byte5.RadarState_OutputTypeCfg;
    COWA::SensorMsg::OutputType out;
    switch (out_type) {
      case 0x0:
        out = OUTPUT_TYPE_NONE;
        break;
      case 0x1:
        out = OUTPUT_TYPE_OBJECTS;
        break;
      case 0x2:
        out = OUTPUT_TYPE_CLUSTERS;
        break;
      default:
        out = OUTPUT_TYPE_ERROR;
    }
    COWA::SensorMsg::RcsThreshold out_;    
    uint8_t rcs_threshold = conti_check->radar_status_check_byte7.RadarState_RCS_Threshold;
    switch (rcs_threshold) {
    case 0x0:
      out_ = RCS_THRESHOLD_STANDARD;
      break;
    case 0x1:
      out_ = RCS_THRESHOLD_HIGH_SENSITIVITY;
      break;
    default:
      out_ = RCS_THRESHOLD_ERROR;
    }
    uint16_t radar_power = (conti_check->radar_status_check_byte3.RadarState_RadarPowerCfg_H << 1) + conti_check->radar_status_check_byte4.RadarState_RadarPowerCfg_L;
    uint8_t send_quality = conti_check->radar_status_check_byte5.RadarState_SendQualityCfg;
    uint8_t send_ext_info = conti_check->radar_status_check_byte5.RadarState_SendExtInfoCfg;
    conti_radar->mutable_radar_state()->set_max_distance(max_distnace);
    conti_radar->mutable_radar_state()->set_output_type(out);
    conti_radar->mutable_radar_state()->set_rcs_threshold(out_);
    conti_radar->mutable_radar_state()->set_radar_power(radar_power);
    conti_radar->mutable_radar_state()->set_send_quality(send_quality);
    conti_radar->mutable_radar_state()->set_send_ext_info(send_ext_info);

    radar_conf*  conti_radar_conf = (radar_conf*)(conf_frame.data);
    if(conti_radar->mutable_radar_state()->max_distance() == conti_radar_conf->radar_conf_byte1.RadarCfg_MaxDistance_H<<2 + 
                                                              conti_radar_conf->radar_conf_byte2.RadarCfg_MaxDistance_L &&
      conti_radar->mutable_radar_state()->send_quality() == conti_radar_conf->radar_conf_byte5.RadarCfg_SendQuality &&
      conti_radar->mutable_radar_state()->send_ext_info() == conti_radar_conf->radar_conf_byte5.RadarCfg_SendExtInfo &&
      conti_radar->mutable_radar_state()->output_type() == conti_radar_conf->radar_conf_byte4.RadarCfg_OutputType &&
      conti_radar->mutable_radar_state()->rcs_threshold() == conti_radar_conf->radar_conf_byte6.RadarCfg_RCS_Threshold &&
      conti_radar->mutable_radar_state()->radar_power() == conti_radar_conf->radar_conf_byte4.RadarCfg_RadarPower)
    {
      CRTRACE << "radar 201 conf status:" 
              <<" max distance " << conti_radar->mutable_radar_state()->max_distance()
              <<" send quality: "<<  conti_radar->mutable_radar_state()->send_quality()
              <<" send ext info: "<< conti_radar->mutable_radar_state()->send_ext_info()
              <<" output type: "<<conti_radar->mutable_radar_state()->output_type()
              <<" rcs threshold: "<<conti_radar->mutable_radar_state()->rcs_threshold()
              <<" radar power: "<<conti_radar->mutable_radar_state()->radar_power();
      is_configured = 1;
    }
    else
    {
      can_drive->Write(conf_frame);
      is_configured = 0;
      CRTRACE<< "re-configure radar";
      return;
    }
    
  }

  void radar_conti_status_60A(const struct CanStamped &frame)
  {
    if(is_configured == 0)
      return;
    radar_conti_status *conti_status = (radar_conti_status *)(frame.data);
    
    uint8_t num_objects = conti_status->radar_conti_status_byte0.Object_NofObjects;   
    uint16_t cycle_counter = conti_status->radar_conti_status_byte1.Object_MeasCounter_H * 256 + conti_status->radar_conti_status_byte2.Object_MeasCounter_L;   
    uint8_t object_interfaceVersion = conti_status->radar_conti_status_byte3.Object_InterfaceVersion;

    if(conti_radar->contiobs_size() <= conti_radar->mutable_object_list_status()->nof_objects() && conti_radar->contiobs_size() > 0)
    {
      auto x = object_pool_->GetObject();
      if(x == NULL)
        CRWARN << "object get failed";
      else
      {
        x->CopyFrom(*conti_radar);
        fbk->Write(x);
      }
    }
    conti_radar->Clear();
    index = 0;
    id_idx.clear();
    conti_radar->set_timestamp(frame.timestamp);
    conti_radar->set_frame_id(frame_id);
    conti_radar->set_sequence(sequence);
    sequence += 1;
    conti_radar->mutable_object_list_status()->set_nof_objects(num_objects);
    conti_radar->mutable_object_list_status()->set_meas_counter(cycle_counter);
    conti_radar->mutable_object_list_status()->set_interface_version(object_interfaceVersion);
  }
  void radar_conti_obs_60B(const struct CanStamped &frame)
  {

    if(is_configured == 0)
      return;
    radar_conti_obs *conti_obs = (radar_conti_obs *)(frame.data);
    uint8_t object_id = conti_obs->radar_conti_obs_byte0.Object_ID;

    uint16_t Object_DistLong = (conti_obs->radar_conti_obs_byte1.Object_DistLong_H << 5) + conti_obs->radar_conti_obs_byte2.Object_DistLong_L;
    double dist_long = Object_DistLong * 0.2 - 500;
    uint16_t Object_DistLat = (conti_obs->radar_conti_obs_byte2.Object_DistLat_H << 8) + conti_obs->radar_conti_obs_byte3.Object_DistLat_L;
    double dist_lat = Object_DistLat * 0.2 - 204.6;
    uint16_t Object_VrelLong = (conti_obs->radar_conti_obs_byte4.Object_VrelLong_H << 2) + conti_obs->radar_conti_obs_byte5.Object_VrelLong_L;
    double vel_long = Object_VrelLong * 0.25 - 128;
    uint16_t Object_VrelLat = (conti_obs->radar_conti_obs_byte5.Object_VrelLat_H << 3) + conti_obs->radar_conti_obs_byte6.Object_VrelLat_L;
    double vel_lat = Object_VrelLat * 0.25 - 64;
    uint8_t dy_obs_prop = conti_obs->radar_conti_obs_byte6.Object_DynProp;   
    uint8_t object_RCS = conti_obs->radar_conti_obs_byte7.Object_RCS; 

    id_idx.insert(std::pair<uint8_t,uint8_t>(object_id,index));
    auto obs = conti_radar->add_contiobs();
    obs->set_obstacle_id(object_id);
    obs->set_longitude_dist(dist_long);
    obs->set_lateral_dist(dist_lat);
    obs->set_longitude_vel(vel_long);
    obs->set_lateral_vel(vel_lat);
    obs->set_dynprop(dy_obs_prop);
    obs->set_rcs(object_RCS);
    index++;
  }

  void radar_conti_quality_60C(const struct CanStamped &frame)
  {
    if(is_configured == 0)
      return;
    radar_conti_quality *conti_quality = (radar_conti_quality *)(frame.data);
    
    uint8_t obj_id = conti_quality->radar_conti_quality_byte0.Obj_ID;
    uint8_t obj_distLong_rms = conti_quality->radar_conti_quality_byte1.Obj_DistLong_rms;
    uint16_t obj_distLat_rms = (conti_quality->radar_conti_quality_byte2.Obj_VrelLat_rms_H << 2) + conti_quality->radar_conti_quality_byte2.Obj_DistLat_rms_L;
    uint8_t obj_vrelLong_rms = conti_quality->radar_conti_quality_byte2.Obj_VrelLong_rms;
    uint16_t obj_vrelLat_rms = (conti_quality->radar_conti_quality_byte2.Obj_VrelLat_rms_H << 4) + conti_quality->radar_conti_quality_byte3.Obj_VrelLat_rms_L;
    uint16_t obj_arelLong_rms = (conti_quality->radar_conti_quality_byte3.Obj_ArelLong_rms_H << 1) + conti_quality->radar_conti_quality_byte4.Obj_ArelLong_rms_L;
    uint8_t obj_arelLat_rms = conti_quality->radar_conti_quality_byte4.Obj_ArelLat_rms;
    uint16_t obj_orientation_rms = (conti_quality->radar_conti_quality_byte4.Obj_Orientation_rms_H << 3) + conti_quality->radar_conti_quality_byte5.Obj_Orientation_rms_L;
    uint8_t obj_probOfExist = conti_quality->radar_conti_quality_byte6.Obj_ProbOfExist;  
    uint8_t obj_measState = conti_quality->radar_conti_quality_byte6.Obj_MeasState;

    std::map<uint8_t,uint8_t>::iterator it = id_idx.find(obj_id);
    if(it != id_idx.end())
    {
      if(it->second > conti_radar->contiobs_size() - 1)
        return;
      auto obs = conti_radar->mutable_contiobs(it->second);
      obs->set_obstacle_id(obj_id);
      obs->set_longitude_dist_rms(LINEAR_RMS[obj_distLong_rms]);
      obs->set_lateral_dist_rms(LINEAR_RMS[obj_distLat_rms]);
      obs->set_longitude_vel_rms(LINEAR_RMS[obj_vrelLong_rms]);
      obs->set_lateral_vel_rms(LINEAR_RMS[obj_vrelLat_rms]);
      obs->set_longitude_accel_rms(LINEAR_RMS[obj_arelLong_rms]);
      obs->set_lateral_accel_rms(LINEAR_RMS[obj_arelLat_rms]);
      obs->set_oritation_angle_rms(LINEAR_RMS[obj_orientation_rms]);
      obs->set_probexist(obj_probOfExist);
      obs->set_meas_state(obj_measState);
    }
  }

  void radar_conti_obs_extended_60D(const struct CanStamped &frame)
  {
    if(is_configured == 0)
      return;
    radar_conti_obs_extended *conti_obs_extended = (radar_conti_obs_extended*)(frame.data);

    uint8_t obj_id = conti_obs_extended->radar_conti_obs_extended_byte0.Object_ID;
    uint16_t obj_arellong = (conti_obs_extended->radar_conti_obs_extended_byte1.Object_ArelLong_H << 3) + conti_obs_extended->radar_conti_obs_extended_byte2.Object_ArelLong_L;
    double obj_acc_long =  obj_arellong * 0.01 - 10;
    uint16_t obj_arellat = (conti_obs_extended->radar_conti_obs_extended_byte2.Object_ArelLat_H << 4) + conti_obs_extended->radar_conti_obs_extended_byte3.Object_ArelLat_L;
    double obj_acc_lat = obj_acc_lat * 0.01 - 2.50;
    uint8_t obj_oriangle = (conti_obs_extended->radar_conti_obs_extended_byte4.Object_OrientationAngel_H << 2)+ conti_obs_extended->radar_conti_obs_extended_byte5.Object_OrientationAngel_L;
    double obj_ori_angle = obj_ori_angle * 0.4 - 180;
    uint8_t obj_length = conti_obs_extended->radar_conti_obs_extended_byte6.Object_Length;
    uint8_t obj_width = conti_obs_extended->radar_conti_obs_extended_byte7.Object_Width;
    uint8_t obj_class = conti_obs_extended->radar_conti_obs_extended_byte3.Object_Class;

    std::map<uint8_t,uint8_t>::iterator it = id_idx.find(obj_id);
    if(it != id_idx.end())
    {
      if(it->second > conti_radar->contiobs_size() - 1)
        return;
      auto obs = conti_radar->mutable_contiobs(it->second);
      obs->set_obstacle_id(obj_id);
      obs->set_longitude_accel(obj_acc_long);
      obs->set_lateral_accel(obj_acc_lat);
      obs->set_oritation_angle(obj_ori_angle);
      obs->set_length(obj_length);
      obs->set_width(obj_width);
      obs->set_obstacle_class(obj_class);
    }
  }
  void radar_conti_cluster_status_600(const struct CanStamped &frame)
  {
    if(is_configured == 0)
      return;
    radar_conti_Cluster_status * cluster_status = (radar_conti_Cluster_status *)(frame.data);
    
    uint8_t num_cluster_near = cluster_status->radar_conti_cluster_status_byte0.Cluster_NofClustersNear;   
    uint8_t num_cluster_far = cluster_status->radar_conti_cluster_status_byte1.Cluster_NofClustersFar;
    uint8_t num_cluster_mea_counter = cluster_status->radar_conti_cluster_status_byte2.Cluster_MeasCounter;
    uint8_t cluster_interfaceVersion = cluster_status->radar_conti_cluster_status_byte3.Cluster_InterfaceVersion;

    if(conti_radar->contiobs_size() <= conti_radar->mutable_cluster_list_status()->near() +
                                      conti_radar->mutable_cluster_list_status()->far())
    {
      fbk->Write(conti_radar);
    }
    conti_radar->Clear();
    conti_radar->set_timestamp(frame.timestamp);
    conti_radar->mutable_cluster_list_status()->set_near(num_cluster_near);
    conti_radar->mutable_cluster_list_status()->set_far(num_cluster_far);
    conti_radar->mutable_cluster_list_status()->set_meas_counter(num_cluster_mea_counter);
    conti_radar->mutable_cluster_list_status()->set_interface_version(cluster_interfaceVersion);
  }

  void radar_conti_cluster_701(const struct CanStamped &frame)
  {
    if(is_configured == 0)
      return;
    radar_conti_cluster * cluster_status = (radar_conti_cluster *)(frame.data);
    
    uint8_t cluster_id = cluster_status->radar_conti_cluster_byte0.Cluster_ID;
    uint16_t cluster_DistLong = (cluster_status->radar_conti_cluster_byte1.Cluster_DistLong_H << 5) + cluster_status->radar_conti_cluster_byte2.Cluster_DistLong_L;
    double dist_long = cluster_DistLong * 0.2 - 500;
    uint16_t cluster_DistLat = (cluster_status->radar_conti_cluster_byte2.Cluster_DistLat_H * 256 + cluster_status->radar_conti_cluster_byte3.Cluster_DistLat_L);
    double dist_lat = cluster_DistLat * 0.2 - 102.3;
    uint16_t cluster_VrelLong = (cluster_status->radar_conti_cluster_byte4.Cluster_VrelLong_H << 2) + cluster_status->radar_conti_cluster_byte5.Cluster_VrelLong_L;
    double vel_long = cluster_VrelLong * 0.25 - 128;
    uint16_t cluster_VrelLat = (cluster_status->radar_conti_cluster_byte5.Cluster_VrelLat_H << 3) + cluster_status->radar_conti_cluster_byte6.Cluster_VrelLat_L;
    double vel_lat = cluster_VrelLat * 0.25 - 64;
    uint8_t dy_clus_prop = cluster_status->radar_conti_cluster_byte6.Cluster_DynProp;    
    uint8_t cluster_RCS = cluster_status->radar_conti_cluster_byte7.Cluster_RCS * 0.5 - 64; 

    auto obs = conti_radar->add_contiobs();
    obs->set_obstacle_id(cluster_id);
    obs->set_longitude_dist(dist_long);
    obs->set_lateral_dist(dist_lat);
    obs->set_longitude_vel(vel_long);
    obs->set_lateral_vel(vel_lat);
    obs->set_dynprop(dy_clus_prop);
    obs->set_rcs(cluster_RCS);
  }

  void radar_conti_cluster_quality_702(const struct CanStamped &frame)
  {
    if(is_configured == 0)
      return;
    radar_conti_Cluster_quality * cluster_quality = (radar_conti_Cluster_quality *)(frame.data);
    
    uint8_t cluster_id = cluster_quality->radar_conti_cluster_quality_byte0.Cluster_ID;
    uint16_t cluster_DistLat = (cluster_quality->radar_conti_cluster_quality_byte1.Cluster_DistLat_rms_H * 256 + cluster_quality->radar_conti_cluster_quality_byte2.Cluster_DistLat_rms_L);
    uint8_t cluster_DistLong = cluster_quality->radar_conti_cluster_quality_byte1.Cluster_DistLong_rms;
    uint16_t cluster_VrelLong = cluster_quality->radar_conti_cluster_quality_byte2.Cluster_VrelLong_rms;
    uint16_t cluster_VrelLat = (cluster_quality->radar_conti_cluster_quality_byte3.Cluster_VrelLat_rms_H * 256 + cluster_quality->radar_conti_cluster_quality_byte2.Cluster_VrelLat_rms_L);
    uint8_t clus_pdh = cluster_quality->radar_conti_cluster_quality_byte3.Cluster_Pdh;    
    uint8_t cluster_ambig_state = cluster_quality->radar_conti_cluster_quality_byte4.Cluster_Ambig_state; 
    
    // auto obs = conti_radar->add_contiobs();
    // obs->set_obstacle_id(cluster_id);
    // obs->set_longitude_dist(LINEAR_RMS[cluster_DistLong]);
    // obs->set_lateral_dist(LINEAR_RMS[cluster_DistLat]);
    // obs->set_longitude_vel(LINEAR_RMS[cluster_VrelLong]);
    // obs->set_lateral_vel(LINEAR_RMS[cluster_VrelLat]);
    // obs->set_probexist(clus_pdh);
    
  }
};

//参数需要整理
class Radar: public CANProcess
{
  private:
    std::shared_ptr<CANDispatch> can_drive;
    std::shared_ptr<RadarProcess> front;
    std::shared_ptr<RadarProcess> rear;
    std::shared_ptr<COWA::Parameter> param;
    std::shared_ptr<COWA::Reader<COWA::NavMsg::PoseStamped>> pose_cbk;
  public:
    Radar()
    {
      param = std::make_shared<COWA::Parameter>(node);

      can_drive = cans[param->value("radar/canbus", 1)];

      front.reset(new RadarProcess(can_drive, node, param->query("radar/front"), 0));
      rear.reset(new RadarProcess(can_drive, node, param->query("radar/rear"), 1));

      pose_cbk = node->CreateReader<COWA::NavMsg::PoseStamped>("/current_pose", 
      [this](const std::shared_ptr<COWA::NavMsg::PoseStamped> &msg) {
          this->PoseCallback(msg);
      });
    }
    int Update() override
    {
      front->Update();
      rear->Update();
    }
    void PoseCallback(const std::shared_ptr<COWA::NavMsg::PoseStamped> &msg)
    {
      front->PoseCallback(msg);
      rear->PoseCallback(msg);
    }


};

REGISTER_CAN(Conti, Radar);