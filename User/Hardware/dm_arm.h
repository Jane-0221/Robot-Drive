#ifndef __DM_ARM_H__
#define __DM_ARM__H__

#include "can_receive_send.h"
#include "dm4310_drv.h"
#include "motor.h"
// dji
/*end_row*/
#define End_ROW_MOTOR CAN_2_6
#define ROWMotor_init DJIMotor_init
#define ROWMotor_set DJIMotor_set
#define ROWMotor_get_data DJIMotor_get_data
// 声明一下省的报错
void ROWMotor_init(Motor_Type_e motor_type, DJIcan_id motor_id);
void ROWMotor_set(int16_t val, DJIcan_id motor_id);
DJI_motor_data_s ROWMotor_get_data(DJIcan_id motor_id);

typedef enum Enum_Motor_DM_Status
{
    Motor_DM_Status_DISABLE = 0,
    Motor_DM_Status_ENABLE,
} Motor_DM_Status;
extern Motor_DM_Status DM_Status[6];
typedef struct
{
    float set, now, last, endrow_offset; // 设置角度值 目标值 实际值
    float endrow_speed;
    float set_endrow_speed;

} Endrow_motor;
extern Endrow_motor endrow_motor;
// dji
void Arm_DJI_pid_cal(void);
void Arm_DJI_updata(void);
// damiao
void Arm_Init(void);
void Arm_DJImotor(void);
void Arm_motor1(void);
void Arm_motor2(void);
void Arm_motor3(void);
void Arm_motor4(void);
void Arm_motor5(void);
void Arm_motor1_DISCON(void);
void Arm_motor2_DISCON(void);
void Arm_motor3_DISCON(void);
void Arm_motor4_DISCON(void);
void Arm_motor5_DISCON(void);
void Arm_motor3_DISCON_OUT(void);


#endif