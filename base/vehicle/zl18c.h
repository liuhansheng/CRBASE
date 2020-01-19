#pragma once
#include <stdint.h>

struct io_ctrl{//ID:30
    struct 
    {
        volatile uint16_t speed_pedal; 
    }io_ctrl_byte0_1;
    struct
    {
        volatile uint16_t speed_vehicle;     
    }io_ctrl_byte2_3;  
        
    struct{
        volatile uint8_t self_driving_enable : 1;
        volatile uint8_t engine_speed_set : 2;
        volatile uint8_t drive : 1;
        volatile uint8_t reverse : 1;
        volatile uint8_t brake :1;
        volatile uint8_t : 0;
    }io_ctrl_byte4;
    struct{
        volatile uint8_t work_start : 1;
        volatile uint8_t work_stop : 1;
        volatile uint8_t left_signal : 1;
        volatile uint8_t right_signal : 1;
        volatile uint8_t warning_signal : 1;
        volatile uint8_t : 0;
    }io_ctrl_byte5;
    struct
    {
        volatile uint8_t encoder_count;    
    }io_ctrl_byte6;
        
    volatile uint8_t io_ctrl_byte7;
};

struct auto_heart{//31
    volatile uint8_t auto_heart_byte0;
    struct
    {
        volatile uint8_t reserved[7];
    }auto_heart_byte1_7;
};

struct can_recv {//ID:40
    struct 
    {
        volatile uint16_t vehicle_speed;   
    }can_recv_byte0_1;
    struct 
    {
        volatile uint16_t engine_speed;    
    }can_recv_byte2_3;
        
    struct{
        volatile uint8_t trash_container_signal : 1;
        volatile uint8_t seat_signal : 1;
        volatile uint8_t : 0;
    }can_recv_byte4;
    struct
    {
        volatile uint8_t reserved[3];  
    }can_recv_byte5_7;
};

struct auto_status{
    struct 
    {
        volatile uint8_t auto_mode : 1;
        volatile uint8_t : 0;
    }auto_status_byte0;
    struct 
    {
        volatile uint8_t reserved[7];
    }auto_status_byte1_7;
};