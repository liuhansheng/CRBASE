#pragma once
#include <stdint.h>

struct idm1{ // ID:0x2C5
    struct{ //byte 0
        volatile uint8_t  IDM_AutomaticDriveModeReq : 1;
        volatile uint8_t  IDM_RemoteStartReq : 1;
        volatile uint8_t  IDM_HandBrakeReq : 1;
        volatile uint8_t : 0;
    }idm_byte0; 
    struct{
        volatile uint8_t IDM_TurnlightReq_R : 1;
        volatile uint8_t IDM_TurnlightReq_L : 1;
        volatile uint8_t IDM_Reserved_HazardLightReq : 1;
        volatile uint8_t IDM_Reserved_ParkTailLightReq : 1;
        volatile uint8_t IDM_Reserved_HighBeamReq : 1;
        volatile uint8_t IDM_LowBeamReq : 1;
        volatile uint8_t IDM_ReverseLightReq : 1;
        volatile uint8_t IDM_Reserved_RearFogLightReq :1;
    }idm_byte1; 
    struct{
        volatile uint8_t IDM_BrakeLightReq : 1;
        volatile uint8_t IDM_Reserved_HornReq : 1;
        volatile uint8_t IDM_Reserved_CentralLockReq : 2;
        volatile uint8_t : 0;
    }idm_byte2;
    uint8_t idm_byte3;
    uint8_t idm_byte4;
    uint8_t idm_byte5;
    struct{
        volatile uint8_t IDM_Reserved1 : 4;
        volatile uint8_t IDM_Reserved_MsgCounter : 4;
    }idm_byte6;
    struct{
        volatile uint8_t IDM_Reserved_Cherksum;
    }idm_byte7;
};

struct idm2{ // ID:0x1C5
    struct{
        volatile uint8_t IDM_GearReq : 4;
        volatile uint8_t : 0;
    }idm_byte0;
    struct{
        volatile uint8_t IDM_VehSpd;
    }idm_byte1;
    struct{
        volatile uint8_t IDM_BrakePedalPositionReq;
    }idm_byte2;
    struct{
        volatile uint8_t IDM_AccelerateReq;
    }idm_byte3;
    struct{
        volatile uint16_t IDM_SteeringAngleReq;
    }idm_byte4_5;
    struct{
        volatile uint8_t IDM_Reserved_MsgCounter : 4;
        volatile uint8_t IDM_EmergencyBrakeReq : 1;
        volatile uint8_t : 0;
    }idm_byte6;
    struct{
        volatile uint8_t IDM_Reserved_Cherksum;
    }idm_byte7;
};

//recv
struct eps_status{ // 0x1D5
    struct{
        volatile uint8_t EPS_AutoCtrlModeSts : 1;
        volatile uint8_t EPS_ManualCtrlDetectionSts : 1;
        volatile uint8_t EPS_Reserved : 5;
        volatile uint8_t EPS_SystemFaultSts : 1;
    }eps_status_byte0;
    struct{
        volatile uint8_t EPS_SteeringAngleH;
    }eps_status_byte1;
    struct{
        volatile uint8_t EPS_SteeringAngleL;
    }eps_status_byte2;
    struct{
        volatile uint8_t EPS_SteeringAngleSpd;
    }eps_status_byte3;
    struct{
        volatile uint8_t EPS_Reserved1[2];
    }eps_status_byte4_5;
    struct{
        volatile uint8_t EPS_Reserved_MsgCounter : 4;
        volatile uint8_t : 0;
    }eps_status_byte6;
    struct{
        volatile uint8_t EPS_Reserved_Cherksum;
    }eps_status_byte7;
};

struct vcu_status{//0x151
    struct{
        volatile uint8_t VCU_TargetGear : 4;
        volatile uint8_t VCU_ActGear : 4;
    }vcu_status_byte0;
    struct{
        volatile uint8_t VCU_AccelPedalPosition;
    }vcu_status_byte1;
    struct{
        volatile uint8_t VCU_BrakePedalSts : 1;
        volatile uint8_t VCU_AccelPedal_VD : 1;
        volatile uint8_t VCU_BrakePedal_VD : 1;
        volatile uint8_t VCU_TargetGear_VD : 1;
        volatile uint8_t VCU_ActGear_VD : 1;
        volatile uint8_t VCU_ReadySts : 1;
        volatile uint8_t VCU_AutoCtrlModeSts : 1;
        volatile uint8_t VCU_Reserved : 1;
    }vcu_status_byte2;
    struct{
        volatile uint8_t VCU_Reserved1[2];
    }vcu_status_byte3_4;
    struct{
        volatile uint8_t VCU_VehGradient;
    }vcu_status_byte5;
    struct{
        volatile uint8_t VCU_Reserved_MsgCounter : 4;
        volatile uint8_t : 0;  
    }vcu_status_byte6;
    struct{
        volatile uint8_t VCU_Reserved_Cherksum;
    }vcu_status_byte7;
};

struct abs_wheel_status{ //0x300
    struct{
        volatile uint8_t ABS_WheelSpd_FL_H : 6;
        volatile uint8_t ABS_WheelSpd_FL_VD : 2;
    }abs_status_byte0;
    struct{
        volatile uint8_t ABS_WheelSpd_FL_L;
    }abs_status_byte1;
    struct{
        volatile uint8_t ABS_WheelSpd_FR_H : 6;
        volatile uint8_t ABS_WheelSpd_FR_VD : 2;
    }abs_status_byte2;
    struct{
        volatile uint8_t ABS_WheelSpd_FR_L;
    }abs_status_byte3;
    struct{
        volatile uint8_t ABS_WheelSpd_RL_H : 6;
        volatile uint8_t ABS_WheelSpd_RL_VD : 2;
    }abs_status_byte4;
    struct{
        volatile uint8_t ABS_WheelSpd_RL_L;
    }abs_status_byte5;
    struct{
        volatile uint8_t ABS_WheelSpd_RR_H : 6;
        volatile uint8_t ABS_WheelSpd_RR_VD : 2;
    }abs_status_byte6;
    struct{
        volatile uint8_t ABS_WheelSpd_RR_L;
    }abs_status_byte7;
};

struct abs_car_status{
    struct{
        uint8_t ABS_Reserved[2];
    }abs_status_byte0_1;
    struct{
        uint8_t ABS_Reserved : 7;
        uint8_t ABS_VehSpd_VD : 1;
    }abs_status_byte2;
    struct{
        uint8_t ABS_Car_Speed_H;
    }abs_status_byte3;
    struct{
        uint8_t ABS_Car_Speed_L;
    }abs_status_byte4;
    struct{
        uint8_t ABS_Reserved[3];
    }abs_status_byte5_7;
};
