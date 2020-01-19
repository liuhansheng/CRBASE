#pragma once
#include <stdint.h>

struct radar_conti_obs{//0x60B
    struct{
        volatile uint8_t  Object_ID;
    }radar_conti_obs_byte0;
    struct{
        volatile uint8_t  Object_DistLong_H;
    }radar_conti_obs_byte1;
    struct{
        volatile uint8_t  Object_DistLat_H : 3;
        volatile uint8_t  Object_DistLong_L : 5;    
    }radar_conti_obs_byte2;
    struct{
        volatile uint8_t  Object_DistLat_L;
    }radar_conti_obs_byte3;
    struct{
        volatile uint8_t  Object_VrelLong_H;
    }radar_conti_obs_byte4;
    struct{
        volatile uint8_t  Object_VrelLat_H : 6;
        volatile uint8_t  Object_VrelLong_L : 2;
    }radar_conti_obs_byte5;
    struct{
        volatile uint8_t  Object_DynProp : 3;
        volatile uint8_t undefined : 2;
        volatile uint8_t  Object_VrelLat_L : 3;
    }radar_conti_obs_byte6;
    struct{
        volatile uint8_t Object_RCS;
    }radar_conti_obs_byte7;
};
struct radar_conti_status{//0x60A
    struct{
        volatile uint8_t  Object_NofObjects;
    }radar_conti_status_byte0;
    struct{
        volatile uint8_t  Object_MeasCounter_H;
    }radar_conti_status_byte1;
    struct{
        volatile uint8_t  Object_MeasCounter_L;   
    }radar_conti_status_byte2;
    struct{
        volatile uint8_t undefined : 4;
        volatile uint8_t Object_InterfaceVersion : 4;
    }radar_conti_status_byte3;
};
struct radar_conti_quality{//0x60C
    struct{
        volatile uint8_t  Obj_ID;
    }radar_conti_quality_byte0;
    struct{
        volatile uint8_t  Obj_DistLat_rms_H : 3;
        volatile uint8_t  Obj_DistLong_rms : 5;
    }radar_conti_quality_byte1;
    struct{
        volatile uint8_t  Obj_VrelLat_rms_H : 1;
        volatile uint8_t  Obj_VrelLong_rms : 5;
        volatile uint8_t  Obj_DistLat_rms_L : 2;      
    }radar_conti_quality_byte2;
    struct{
        volatile uint8_t  Obj_ArelLong_rms_H : 4;
        volatile uint8_t  Obj_VrelLat_rms_L  : 4;
    }radar_conti_quality_byte3;
    struct{
        volatile uint8_t  Obj_Orientation_rms_H : 2;
        volatile uint8_t  Obj_ArelLat_rms : 5;
        volatile uint8_t  Obj_ArelLong_rms_L : 1;   
    }radar_conti_quality_byte4;
    struct{
        volatile uint8_t  undefined : 5;
        volatile uint8_t  Obj_Orientation_rms_L : 3;
    }radar_conti_quality_byte5;
    struct{
        volatile uint8_t  undefined : 2;
        volatile uint8_t  Obj_MeasState : 3;
        volatile uint8_t  Obj_ProbOfExist : 3;
    }radar_conti_quality_byte6;
};
struct radar_conti_obs_extended{//0x60D
    struct{
        volatile uint8_t  Object_ID;
    }radar_conti_obs_extended_byte0;
    struct{
        volatile uint8_t  Object_ArelLong_H;
    }radar_conti_obs_extended_byte1;
    struct{
        volatile uint8_t  Object_ArelLat_H : 5;
        volatile uint8_t  Object_ArelLong_L : 3;    
    }radar_conti_obs_extended_byte2;
    struct{
        volatile uint8_t  Object_Class : 3;
        volatile uint8_t  undefined : 1;
        volatile uint8_t  Object_ArelLat_L : 4;
    }radar_conti_obs_extended_byte3;
    struct{
        volatile uint8_t  Object_OrientationAngel_H;
    }radar_conti_obs_extended_byte4;
    struct{
        volatile uint8_t  undefined : 6;
        volatile uint8_t  Object_OrientationAngel_L : 2;
    }radar_conti_obs_extended_byte5;
    struct{
        volatile uint8_t  Object_Length;
    }radar_conti_obs_extended_byte6;
    struct{
        volatile uint8_t Object_Width;
    }radar_conti_obs_extended_byte7;
};

struct radar_conti_cluster{//0x701
    struct{
        volatile uint8_t  Cluster_ID;
    }radar_conti_cluster_byte0;
    struct{
        volatile uint8_t  Cluster_DistLong_H;
    }radar_conti_cluster_byte1;
    struct{
        volatile uint8_t  Cluster_DistLat_H : 2;
        volatile uint8_t  Cluster_DistLong_L : 5;    
    }radar_conti_cluster_byte2;
    struct{
        volatile uint8_t  Cluster_DistLat_L;
    }radar_conti_cluster_byte3;
    struct{
        volatile uint8_t  Cluster_VrelLong_H;
    }radar_conti_cluster_byte4;
    struct{
        volatile uint8_t  Cluster_VrelLat_H : 6;
        volatile uint8_t  Cluster_VrelLong_L : 2;
    }radar_conti_cluster_byte5;
    struct{
        volatile uint8_t  Cluster_DynProp : 3;
        volatile uint8_t undefined : 2;
        volatile uint8_t  Cluster_VrelLat_L : 3;
    }radar_conti_cluster_byte6;
    struct{
        volatile uint8_t Cluster_RCS;
    }radar_conti_cluster_byte7;
};

struct radar_conti_Cluster_status{//0x600
    struct{
        volatile uint8_t  Cluster_NofClustersNear;
    }radar_conti_cluster_status_byte0;
    struct{
        volatile uint8_t  Cluster_NofClustersFar;
    }radar_conti_cluster_status_byte1;
    struct{
        volatile uint8_t  Cluster_MeasCounter;   
    }radar_conti_cluster_status_byte2;
    struct{
        volatile uint8_t Cluster_InterfaceVersion;
    }radar_conti_cluster_status_byte3;
};

struct radar_conti_Cluster_quality{//0x702
    struct{
        volatile uint8_t  Cluster_ID;
    }radar_conti_cluster_quality_byte0;
    struct{
        volatile uint8_t  Cluster_DistLat_rms_H : 3;
        volatile uint8_t  Cluster_DistLong_rms : 5;
    }radar_conti_cluster_quality_byte1;
    struct{
        volatile uint8_t  Cluster_VrelLat_rms_L : 1;
        volatile uint8_t  Cluster_VrelLong_rms : 5;
        volatile uint8_t  Cluster_DistLat_rms_L : 2;      
    }radar_conti_cluster_quality_byte2;
    struct{
        volatile uint8_t  Cluster_Pdh : 3;
        volatile uint8_t  underfined : 1;
        volatile uint8_t  Cluster_VrelLat_rms_H : 4;
    }radar_conti_cluster_quality_byte3;
    struct{
        volatile uint8_t  Cluster_Ambig_state : 5;
        volatile uint8_t  underfined : 3;
    }radar_conti_cluster_quality_byte4;
};

const double LINEAR_RMS[32] = {0.005, 0.006, 0.008,  0.011, 0.014, 0.018, 0.023,
                               0.029, 0.038, 0.049,  0.063, 0.081, 0.105, 0.135,
                               0.174, 0.224, 0.288,  0.371, 0.478, 0.616, 0.794,
                               1.023, 1.317, 1.697,  2.187, 2.817, 3.630, 4.676,
                               6.025, 7.762, 10.000, 100.00};

struct radar_status_check{//0x201
    struct{
        volatile uint8_t  underfined : 6;
        volatile uint8_t  RadarState_NVMReadStatus : 1;
        volatile uint8_t  RadarState_NVMwriteStatus : 1;
    }radar_status_check_byte0;
    struct{
        volatile uint8_t  RadarState_MaxDistanceCfg_H;
    }radar_status_check_byte1;
    struct{
        volatile uint8_t  underfined : 1;
        volatile uint8_t  RadarState_Voltage_Error : 1;
        volatile uint8_t  RadarState_Temporary_Error : 1;
        volatile uint8_t  RadarState_Temperature_Error : 1;
        volatile uint8_t  RadarState_Interference : 1;
        volatile uint8_t  RadarState_Persistent_Error : 1;
        volatile uint8_t  RadarState_MaxDistanceCfg_L : 2;   
    }radar_status_check_byte2;
    struct{
        volatile uint8_t RadarState_RadarPowerCfg_H : 2;
        volatile uint8_t undefined : 6;    
    }radar_status_check_byte3;
    struct{
        volatile uint8_t RadarState_SensorID : 3;
        volatile uint8_t undefined : 1;
        volatile uint8_t RadarState_SortIndex : 3; 
        volatile uint8_t RadarState_RadarPowerCfg_L : 1;  
    }radar_status_check_byte4;
    struct{
        volatile uint8_t undefined : 1;
        volatile uint8_t RadarState_CtrlRelayCfg : 1;
        volatile uint8_t RadarState_OutputTypeCfg : 2; 
        volatile uint8_t RadarState_SendQualityCfg : 1;
        volatile uint8_t RadarState_SendExtInfoCfg : 1;  
        volatile uint8_t RadarState_MotionRxState : 2;    
    }radar_status_check_byte5;
    struct{
        volatile uint8_t undefined1 : 2;
        volatile uint8_t RadarState_RCS_Threshold : 3;
        volatile uint8_t undefined2 : 3;    
    }radar_status_check_byte7;
};
struct radar_conf{//0x200
    struct{
        volatile uint8_t  RadarCfg_MaxDistance_valid : 1;
        volatile uint8_t  RadarCfg_SensorID_valid : 1;
        volatile uint8_t  RadarCfg_RadarPower_valid : 1;
        volatile uint8_t  RadarCfg_OutputType_valid : 1;
        volatile uint8_t  RadarCfg_SendQuality_valid : 1;
        volatile uint8_t  RadarCfg_SendExtInfo_valid : 1;
        volatile uint8_t  RadarCfg_SortIndex_valid : 1;
        volatile uint8_t  RadarCfg_StoreInNVM_valid : 1;
    }radar_conf_byte0;
    struct{
        volatile uint8_t  RadarCfg_MaxDistance_H;
    }radar_conf_byte1;
    struct{
        volatile uint8_t  underfined : 6;
        volatile uint8_t  RadarCfg_MaxDistance_L : 2;
    }radar_conf_byte2;
    struct{
        uint8_t reserved;
    }radar_conf_byte3;
    struct{
        volatile uint8_t  RadarCfg_SensorID : 3;
        volatile uint8_t  RadarCfg_OutputType : 2;
        volatile uint8_t  RadarCfg_RadarPower : 3;
    }radar_conf_byte4;
    struct{
        volatile uint8_t  RadarCfg_CtrlRelay_valid : 1;
        volatile uint8_t  RadarCfg_CtrlRelay : 1;
        volatile uint8_t  RadarCfg_SendQuality : 1;
        volatile uint8_t  RadarCfg_SendExtInfo : 1;
        volatile uint8_t  RadarCfg_SortIndex : 3;
        volatile uint8_t  RadarCfg_StoreInNVM : 1;
    }radar_conf_byte5;
    struct{
        volatile uint8_t  RadarCfg_RCS_Threshold_valid : 1;
        volatile uint8_t  RadarCfg_RCS_Threshold : 3;
        volatile uint8_t  underfined : 4;
    }radar_conf_byte6;
        struct{
        uint8_t reserved;
    }radar_conf_byte7;
};

struct speed_input{//0x300
    struct{
        uint8_t speed_H : 5;
        uint8_t reserved : 1;
        uint8_t speedDirection : 2; // 0x0: standstill 0x1: forward 0x2: backward
    }speed_input_byte0;

    struct{
        volatile uint8_t  speed_L; //range 0 - 163.8 m/s resulotion: 0.02
    }speed_input_byte1;
};
struct yaw_rate_input{//0x301
    struct{
        uint8_t yaw_rate_H;
    }yaw_rate_input_byte0;

    struct{
        volatile uint8_t  yaw_rate_L; 
    }yaw_rate_input_byte1;
};


