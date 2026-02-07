#ifndef LIMIT_FILTER_H
#define LIMIT_FILTER_H
#include "referee_system.h"
typedef struct {
    float data_0;
    float data_1;
    float data_2;
    float data_3;
    float data_4;
    float data_5;
    float data_6;
    short data_7;
} smoothed_robot_data_t;
extern smoothed_robot_data_t smoothed_data;

// 初始化限幅滤波器
void limit_filter_init(ext_custom_robot_data_t* filtered_data);

// 更新限幅滤波器
void update_limit_filter(ext_custom_robot_data_t* raw_data, ext_custom_robot_data_t* filtered_data);



void init_smoothed_data_init(void);
void update_smoothed_data(unsigned long current_time_ms);
#endif // LIMIT_FILTER_H