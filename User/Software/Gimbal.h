#ifndef __GIMBAL_H__
#define __GIMBAL_H__

#include "CAN_receive_send.h"

#include "motor.h"

/*云台电机*/
#define USE_DJIMOTOR_AS_GIMBALMOTOR

#ifdef USE_DJIMOTOR_AS_GIMBALMOTOR
/*电机ID*/
#define YAWMotor CAN_2_5
#define PITCHMotor CAN_2_6
/*函数*/
#define GIMBALMotor_init DJIMotor_init 
#define GIMBALMotor_set DJIMotor_set
#define GIMBALMotor_get_data DJIMotor_get_data
#define GIMBALMotor_setzero DJIMotor_setzero
//声明一下
void GIMBALMotor_init(Motor_Type_e type, uint8_t motor_id);
void GIMBALMotor_set(int16_t current, uint8_t motor_id);
void GIMBALMotor_setzero(double zero_angle,uint8_t motor_id);
DJI_motor_data_s GIMBALMotor_get_data(uint8_t motor_id);
/*电机参数*/
#define GIMBALMOTOR_MAX_CURRENT DJIMOTOR_MAX_CURRENT

#endif // USE_DJIMOTOR_AS_GIMBALMOTOR

/*云台参数*/
//上下限位
#define PITCHI_MIN_ANGLE -15.0f
#define PITCHI_MAX_ANGLE 25.0f


void Gimbal_init();
void Gimbal_set_pitch_angle(float angle);
void Gimbal_set_yaw_angle(float angle);
void Gimbal_control();

float degree_to_rad(float degrees);
float rad_to_degree(float radians);

#endif // !__GIMBAL_H__
