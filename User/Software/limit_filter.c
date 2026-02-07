#include "limit_filter.h"
#include <math.h>
#include "referee_system.h"
#include "ramp_generator.h"

// 限幅阈值（可以根据实际情况调整）
#define THRESHOLD_DATA_0 2.0f
#define THRESHOLD_DATA_1 2.0f
#define THRESHOLD_DATA_2 2.0f
#define THRESHOLD_DATA_3 2.0f
#define THRESHOLD_DATA_4 2.0f
#define THRESHOLD_DATA_5 1000.0f
#define THRESHOLD_DATA_6 2.0f
#define THRESHOLD_DATA_7 100
// 定义斜坡发生器实例，每个需要平滑的数据字段对应一个
RampGenerator data_0_ramp;
RampGenerator data_1_ramp;
RampGenerator data_2_ramp;
RampGenerator data_3_ramp;
RampGenerator data_4_ramp;
RampGenerator data_5_ramp;
RampGenerator data_6_ramp;
RampGenerator data_7_ramp;
// 全局滤波变量
static ext_custom_robot_data_t static_filtered_data;
smoothed_robot_data_t smoothed_data;
// 初始化限幅滤波器
void limit_filter_init(ext_custom_robot_data_t* filtered_data)
{
    static_filtered_data.data_0 = filtered_data->data_0;
    static_filtered_data.data_1 = filtered_data->data_1;
    static_filtered_data.data_2 = filtered_data->data_2;
    static_filtered_data.data_3 = filtered_data->data_3;
    static_filtered_data.data_4 = filtered_data->data_4;
    static_filtered_data.data_5 = filtered_data->data_5;
    static_filtered_data.data_6 = filtered_data->data_6;
    static_filtered_data.data_7 = filtered_data->data_7;


}

// 更新限幅滤波器
void update_limit_filter(ext_custom_robot_data_t* raw_data, ext_custom_robot_data_t* filtered_data)
{
    // 对 data_0 应用限幅滤波
    if (fabs(raw_data->data_0 - static_filtered_data.data_0) <= THRESHOLD_DATA_0)
    {
        static_filtered_data.data_0 = raw_data->data_0;
    }

    // 对 data_1 应用限幅滤波
    if (fabs(raw_data->data_1 - static_filtered_data.data_1) <= THRESHOLD_DATA_1)
    {
        static_filtered_data.data_1 = raw_data->data_1;
    }

    // 对 data_2 应用限幅滤波
    if (fabs(raw_data->data_2 - static_filtered_data.data_2) <= THRESHOLD_DATA_2)
    {
        static_filtered_data.data_2 = raw_data->data_2;
    }

    // 对 data_3 应用限幅滤波
    if (fabs(raw_data->data_3 - static_filtered_data.data_3) <= THRESHOLD_DATA_3)
    {
        static_filtered_data.data_3 = raw_data->data_3;
    }

    // 对 data_4 应用限幅滤波
    if (fabs(raw_data->data_4 - static_filtered_data.data_4) <= THRESHOLD_DATA_4)
    {
        static_filtered_data.data_4 = raw_data->data_4;
    }

    // 对 data_5 应用限幅滤波
    if (fabs(raw_data->data_5 - static_filtered_data.data_5) <= THRESHOLD_DATA_5)
    {
        static_filtered_data.data_5 = raw_data->data_5;
    }

    // 对 data_6 应用限幅滤波
    if (fabs(raw_data->data_6 - static_filtered_data.data_6) <= THRESHOLD_DATA_6)
    {
        static_filtered_data.data_6 = raw_data->data_6;
    }

    // 对 data_7 应用限幅滤波
    if (abs(raw_data->data_7 - static_filtered_data.data_7) <= THRESHOLD_DATA_7)
    {
        static_filtered_data.data_7 = raw_data->data_7;
    }

    // 更新滤波后的数据
    *filtered_data = static_filtered_data;
}
 // 初始化每个斜坡发生器实例
void init_smoothed_data_init()
 {
   
    RampGenerator_Init(&data_0_ramp, 10, 1.0f, 1.0f, 3.0f);  //
    RampGenerator_Init(&data_1_ramp, 10, 1.0f, 1.0f, 3.0f);  //
    RampGenerator_Init(&data_2_ramp, 10, 2.0f, 2.0f, 3.0f);
    RampGenerator_Init(&data_3_ramp, 10, 2.0f, 2.0f, 3.0f);
    RampGenerator_Init(&data_4_ramp, 10, 2.0f, 2.0f, 3.0f);
    RampGenerator_Init(&data_5_ramp, 10, 60.0f, 60.0f, 360.0f);//末端待调试
    RampGenerator_Init(&data_6_ramp, 10, 1.0f, 1.0f, 3.0f);
    RampGenerator_Init(&data_7_ramp, 10, 1.0f, 1.0f, 3.0f); // 假设data_7也在-1.5到1.5范围内
    
    // 初始化平滑数据结构
    smoothed_data.data_0 = filtered_data.data_0;
    smoothed_data.data_1 = filtered_data.data_1;
    smoothed_data.data_2 = filtered_data.data_2;
    smoothed_data.data_3 = filtered_data.data_3;
    smoothed_data.data_4 = filtered_data.data_4;
    smoothed_data.data_5 = filtered_data.data_5;
    smoothed_data.data_6 = filtered_data.data_6;
    smoothed_data.data_7 = filtered_data.data_7;
}
// 在主循环或定时器中断中定期调用的更新函数
void update_smoothed_data(unsigned long current_time_ms)
 {
    // 获取当前原始数据
    float target_data_0 = filtered_data.data_0;
    float target_data_1 = filtered_data.data_1;
    float target_data_2 = filtered_data.data_2;
    float target_data_3 = filtered_data.data_3;
    float target_data_4 = filtered_data.data_4;
    float target_data_5 = filtered_data.data_5;
    float target_data_6 = filtered_data.data_6;
    short target_data_7 = filtered_data.data_7;
    
    // 更新每个斜坡发生器的目标值
    RampGenerator_SetTarget(&data_0_ramp, target_data_0);
    RampGenerator_SetTarget(&data_1_ramp, target_data_1);
    RampGenerator_SetTarget(&data_2_ramp, target_data_2);
    RampGenerator_SetTarget(&data_3_ramp, target_data_3);
    RampGenerator_SetTarget(&data_4_ramp, target_data_4);
    RampGenerator_SetTarget(&data_5_ramp, target_data_5);
    RampGenerator_SetTarget(&data_6_ramp, target_data_6);
    RampGenerator_SetTarget(&data_7_ramp, (float)target_data_7); // 转换为浮点数
    
    // 更新所有斜坡发生器实例
    RampGenerator_Update(&data_0_ramp, current_time_ms);
    RampGenerator_Update(&data_1_ramp, current_time_ms);
    RampGenerator_Update(&data_2_ramp, current_time_ms);
    RampGenerator_Update(&data_3_ramp, current_time_ms);
    RampGenerator_Update(&data_4_ramp, current_time_ms);
    RampGenerator_Update(&data_5_ramp, current_time_ms);
    RampGenerator_Update(&data_6_ramp, current_time_ms);
    RampGenerator_Update(&data_7_ramp, current_time_ms);
    
    // 获取平滑后的值
    smoothed_data.data_0 = RampGenerator_GetCurrent(&data_0_ramp);
    smoothed_data.data_1 = RampGenerator_GetCurrent(&data_1_ramp);
    smoothed_data.data_2 = RampGenerator_GetCurrent(&data_2_ramp);
    smoothed_data.data_3 = RampGenerator_GetCurrent(&data_3_ramp);
    smoothed_data.data_4 = RampGenerator_GetCurrent(&data_4_ramp);
    smoothed_data.data_5 = RampGenerator_GetCurrent(&data_5_ramp);
    smoothed_data.data_6 = RampGenerator_GetCurrent(&data_6_ramp);
    smoothed_data.data_7 = (short)RampGenerator_GetCurrent(&data_7_ramp); // 转回短整型
}