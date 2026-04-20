#ifndef __OMNI_WHEEL_H__
#define __OMNI_WHEEL_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* 底盘速度指令：
 * x: 车体前向速度，单位 m/s
 * y: 车体右向速度，单位 m/s
 * w: 车体顺时针角速度，单位 rad/s
 */
extern volatile float x;
extern volatile float y;
extern volatile float w;

/* 调试观察量，方便在在线调试窗口里直接看底盘指令与反馈。 */
typedef struct
{
    float x_cmd;
    float y_cmd;
    float w_cmd_in;
    float w_cmd_out;
    float yaw_now;
    float yaw_target;
    float yaw_correction;
    float wheel_cmd1;
    float wheel_cmd2;
    float wheel_cmd3;
    float fb_speed1;
    float fb_speed2;
    float fb_speed3;
    uint8_t run_mode1;
    uint8_t run_mode2;
    uint8_t run_mode3;
    uint8_t pattern1;
    uint8_t pattern2;
    uint8_t pattern3;
    uint32_t update_tick_ms;
    uint32_t update_count;
} Omni_Wheel_Debug_t;

extern volatile Omni_Wheel_Debug_t omni_debug;

/* 初始化 3 个全向轮电机为 FDCAN3 速度模式。 */
void Omni_Wheel_Init(void);
/* 根据全局 x/y/w 计算 3 个轮子的目标角速度并下发。 */
void Omni_Wheel_Update(void);
/* 处理 FDCAN3 上 RobStride 扩展帧反馈。 */
void Omni_Wheel_RxCallback(uint32_t ext_id, uint8_t *data);

#ifdef __cplusplus
}
#endif

#endif
