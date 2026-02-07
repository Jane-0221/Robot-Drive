#ifndef __SHOOT_H__
#define __SHOOT_H__

#include "CAN_receive_send.h"

#include "motor.h"
/*发射机构参数*/
enum shoot_speed_e // 摩擦轮速度
{
    SHOOT_SPEED_BEGIN = 2000,//开始反转值
    SHOOT_SPEED_CLOSE = 0,    //停止速度值
    SHOOT_SPEED_READY = -6500,  //正常工作值
    SHOOT_SPEED_DEGUG=-1500, //退弹低速值
};
extern enum shoot_speed_e shoot_speed;

enum trigger_speed_e // 拨弹电机速度
{
    TRIGGER_SPEED_CLOSE=0,
    TRIGGER_SPEED_HIGH=1000,
    TRIGGER_SPEED_MID=3000,
    TRIGGER_SPEED_LOW=1000,
    TRIGGER_SPEED_DEBUG=3000,
};
extern enum trigger_speed_e trigger_speed;

/* 电机 */
#define USE_DJIMOTOR_AS_SHOOTMOTOR

#ifdef USE_DJIMOTOR_AS_SHOOTMOTOR
/* 电机ID*/
#define ShootMotor_L CAN_2_2
#define ShootMotor_R CAN_2_4

#define TRIGGER_MOTOR CAN_2_7

/* 内部调用 */
#define SHOOTMotor_init DJIMotor_init
#define SHOOTMotor_set DJIMotor_set
#define TriggerMotor_init DJIMotor_init
#define TriggerMotor_set DJIMotor_set
#define TriggerMotor_get_data DJIMotor_get_data
#define SHOOTMotor_get_data DJIMotor_get_data
// 声明
void SHOOTMotor_init(Motor_Type_e motor_type, DJIcan_id motor_id);
void TriggerMotor_init(Motor_Type_e motor_type, DJIcan_id motor_id);
void SHOOTMotor_set(int16_t val, DJIcan_id motor_id);
void TriggerMotor_set(int16_t val, DJIcan_id motor_id);
DJI_motor_data_s SHOOTMotor_get_data(DJIcan_id motor_id);
DJI_motor_data_s TriggerMotor_get_data(DJIcan_id motor_id);

/* 电机参数 */
#define SHOOTMOTOR_MAX_CURRENT DJIMOTOR_MAX_CURRENT

#endif // USE_DJIMOTOR_AS_SHOOTMOTOR

/* 外部调用 */
void Shoot_init();
void Shoot_task();

#endif //__SHOOT_H__