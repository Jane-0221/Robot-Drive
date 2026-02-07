#ifndef __CHASSIS_H__
#define __CHASSIS_H__

#include "CAN_receive_send.h"

#include "User_math.h"

#include "motor.h"
/*底盘参数*/
#define WHEEL_FR CAN_1_1
#define WHEEL_FL CAN_1_2
#define WHEEL_BL CAN_1_3
#define WHEEL_BR CAN_1_4
#define R_body 0.273  // 底盘安装半径（单位：米，根据实际值修改）
#define r_wheel 0.077 // 轮子半径（单位：米，根据实际值修改）

//---------------------------
// 逆运动学宏定义（速度 → 轮速）（M_S → RAD_S）
//---------------------------
// 输入：底盘速度 vx, vy, 角速度 omega
// 输出：四个轮子的转速（浮点型）
#define USE_OMNI_CHASSIS
#ifdef USE_OMNI_CHASSIS

// 前左轮 FL 转速计算单位
#define IK_WHEEL_FL(vx, vy, omega) \
    ((SQRT1_2 * (vy) + SQRT1_2 * (vx) + (R_body) * (omega)) / r_wheel)

// 前右轮 FR 转速计算
#define IK_WHEEL_FR(vx, vy, omega) \
    ((-SQRT1_2 * (vy) + SQRT1_2 * (vx) + (R_body) * (omega)) / r_wheel)

// 后左轮 BL 转速计算
#define IK_WHEEL_BL(vx, vy, omega) \
    ((+SQRT1_2 * (vy) - SQRT1_2 * (vx) + (R_body) * (omega)) / r_wheel)

// 后右轮 BR 转速计算
#define IK_WHEEL_BR(vx, vy, omega) \
    ((-SQRT1_2 * (vy) - SQRT1_2 * (vx) + (R_body) * (omega)) / r_wheel)

#endif // USE_OMNI_CHASSIS

//---------------------------
// 正运动学宏定义（轮速 → 速度）（RAD_S → M_S）
//---------------------------
// 输入：四个轮速 wFL, wFR, wBL, wBR
// 输出：底盘速度 vx, vy, 角速度 omega（通过指针返回）

#ifdef USE_OMNI_CHASSIS

// 计算 vx
#define FK_VX(wFL, wFR, wBL, wBR) \
    (((-(wBR) + (wFR) + (wFL) - (wBL)) * SQRT2 * r_wheel) / 4.0f)

// 计算 vy
#define FK_VY(wFL, wFR, wBL, wBR) \
    ((((wFL) - (wBR) - (wFR) + (wBL)) * SQRT2 * r_wheel) / 4.0f)

// 计算 omega（角速度）
#define FK_OMEGA(wFL, wFR, wBL, wBR) \
    ((((wFL) + (wFR) + (wBL) + (wBR)) * r_wheel) / (4.0f * R_body))

#endif

//---------------------------
// 逆动力学宏定义（底盘驱动力 → 轮毂力矩）（F → T）
//---------------------------
// 输入：底盘驱动力 Fx, Fy, 旋转力矩   T
// 输出：四个轮子的力矩（浮点型）
#ifdef USE_OMNI_CHASSIS

// 前左轮 FL 力矩计算
#define IK_WHEEL_FL_T(Fx, Fy, T) \
    ((SQRT2 * (Fy) + SQRT2 * (Fx) + (T) / 4.0f / (R_body)) * r_wheel)

// 前右轮 FR 力矩计算
#define IK_WHEEL_FR_T(Fx, Fy, T) \
    ((-SQRT2 * (Fy) + SQRT2 * (Fx) + (T) / 4.0f / (R_body)) * r_wheel)

// 后左轮 BL 力矩计算
#define IK_WHEEL_BL_T(Fx, Fy, T) \
    ((+SQRT2 * (Fy) - SQRT2 * (Fx) + (T) / 4.0f / (R_body)) * r_wheel)

// 后右轮 BR 力矩计算
#define IK_WHEEL_BR_T(Fx, Fy, T) \
    ((-SQRT2 * (Fy) - SQRT2 * (Fx) + (T) / 4.0f / (R_body)) * r_wheel)

#endif // USE_OMNI_CHASSIS

/*电机*/
#define USE_DJIMotor

/*电机配置*/
#ifdef USE_DJIMotor

#define CHASSISMotor_init DJIMotor_init
#define CHASSISMotor_set DJIMotor_set
#define CHASSISMotor_get_data DJIMotor_get_data
// 声明一下省的报错
void CHASSISMotor_init(Motor_Type_e motor_type, DJIcan_id motor_id);
void CHASSISMotor_set(int16_t val, DJIcan_id motor_id);
DJI_motor_data_s CHASSISMotor_get_data(DJIcan_id motor_id);
extern float AHRS_yaw_set;
/*电机参数*/
#define CHASSISMOTOR_MAX_CURRENT DJIMOTOR_MAX_CURRENT
#define CHASSISMOTOR_T_A DJIMOTOR_T_A

#endif // USE_DJIMotor

/*内部数据类型*/
struct chassis_status
{
struct
{
float x, y, r;
 float now_x, now_y, now_r;
 float last_x, last_y, last_r;
 float max_x, max_y, max_r; // m/s
} speed;

struct
{
 float now_x, now_y, now_r;
 float max_x, max_y, max_r; // m/s^2
 } acc;
 enum chassis_status_e
{
 park = 0,
 move,
 }status;
 
 int32_t wheel_current[4]; // PID输出的电调电流
 double wheel_speed[4];
 double angle_set[4];
 double angle_now[4];
 
 uint8_t is_open_cap;
};
/*外部调用*/
void Chassis_init(void);
void Chassis_move(void);
void Chassis_set_x(float x);
void Chassis_set_y(float y);
void Chassis_set_r(float r);
void Chassis_set_accel(float acc);
void status(void);
#define CHASSIS_TASK_TIME 1 // 底盘任务刷新间隔

#endif // !__CHASSIS_H__
