#include "dm_arm.h"
#include "motor.h"
#include "dm4310_drv.h"
#include "pid.h"
#include "motor.h"
#include "remote_control.h"
extern Motor_DM_Status DM_Status[6];
void Arm_Init()
{
  arm_motor_init(&arm_motor[Motor1], 0x05, POS_MODE); // 设置ID和控制模式  大yaw
  arm_motor_init(&arm_motor[Motor2], 0x02, POS_MODE); // 设置ID和控制模式  大pitch
  arm_motor_init(&arm_motor[Motor3], 0x03, POS_MODE); // 设置ID和控制模式  中yaw
  arm_motor_init(&arm_motor[Motor4], 0x04, POS_MODE); // 设置ID和控制模式  中row
  arm_motor_init(&arm_motor[Motor5], 0x05, POS_MODE); // 设置ID和控制模式  末端pitch

  enable_motor_mode(&hfdcan1, 0x05, POS_MODE);
  enable_motor_mode(&hfdcan2, 0x02, POS_MODE);
  enable_motor_mode(&hfdcan2, 0x03, POS_MODE);
  enable_motor_mode(&hfdcan2, 0x04, POS_MODE);
  enable_motor_mode(&hfdcan2, 0x05, POS_MODE);
  enable_motor_mode(&hfdcan2, 0x06, POS_MODE);
  // 速度

  vel_motor.MT04 = 5.0;
  // 位置

  pos_motor.MT04 = 1;
}

void Arm_motor4()
{

  // // 判断该时间段内是否接收过电机数据
  // if (Flag_damiao[4] == Pre_Flag_damiao[4])
  // {
  //   // 电机断开连接
  //   DM_Status[4] = Motor_DM_Status_DISABLE;
  //   // 错误清除，重新使能
  //   // CAN_Send_Clear_Error(&hfdcan2,0x04);
  //   CAN_Send_Enter(&hfdcan2, 0x04);
  // }
  // else
  // {
  //   // 电机保持连接
  //   DM_Status[4] = Motor_DM_Status_ENABLE;
  // 正常数据发送
  // if (pos_motor.MT04 >= 3.4f)
  // {
  //   pos_motor.MT04 = 3.4f;
  // }
  // if (pos_motor.MT04 <= -1.9f)
  // {
  //   pos_motor.MT04 = -1.9f;
  // }
  set_DM_mode(Motor4, POS_MODE);
  set_DM_pos_vel(pos_motor.MT04, vel_motor.MT04, Motor4);
  pos_speed_ctrl(&hfdcan2, 0x04, 20,1);

  pos_motor.MT04=20;
  Pre_Flag_damiao[4] = Flag_damiao[4];
}
void Arm_motor5()
{

  // // 判断该时间段内是否接收过电机数据
  // if (Flag_damiao[5] == Pre_Flag_damiao[5])
  // {
  //   // 电机断开连接
  //   DM_Status[5] = Motor_DM_Status_DISABLE;
  //   // 错误清除，重新使能
  //   //	CAN_Send_Clear_Error(&hfdcan2,0x05);
  //   CAN_Send_Enter(&hfdcan2, 0x05);
  // }
  // else
  // {
  //   // 电机保持连接
  //   DM_Status[5] = Motor_DM_Status_ENABLE;
  //   // 正常数据发送
  //     if (pos_motor.MT05 >= 1.9f)
  //   {
  //     pos_motor.MT05 = 1.9f;
  //   }
  //    if (pos_motor.MT05 <= -1.9f)
  //   {
  //     pos_motor.MT05 = -1.9f;
  //   }
  set_DM_mode(Motor5, POS_MODE);
  set_DM_pos_vel(pos_motor.MT05, vel_motor.MT05, Motor5);
  pos_motor.MT05 += 0.01;
  pos_speed_ctrl(&hfdcan2, 0x05, arm_motor[Motor5].ctrl.pos_set, 1);

  Pre_Flag_damiao[5] = Flag_damiao[5];
}
void Arm_motor6()
{

  // // 判断该时间段内是否接收过电机数据
  // if (Flag_damiao[5] == Pre_Flag_damiao[5])
  // {
  //   // 电机断开连接
  //   DM_Status[5] = Motor_DM_Status_DISABLE;
  //   // 错误清除，重新使能
  //   //	CAN_Send_Clear_Error(&hfdcan2,0x05);
  //   CAN_Send_Enter(&hfdcan2, 0x05);
  // }
  // else
  // {
  //   // 电机保持连接
  //   DM_Status[5] = Motor_DM_Status_ENABLE;
  //   // 正常数据发送
  //     if (pos_motor.MT05 >= 1.9f)
  //   {
  //     pos_motor.MT05 = 1.9f;
  //   }
  //    if (pos_motor.MT05 <= -1.9f)
  //   {
  //     pos_motor.MT05 = -1.9f;
  //   }
  set_DM_mode(Motor6, POS_MODE);
  set_DM_pos_vel(pos_motor.MT06, vel_motor.MT06, Motor6);
  pos_motor.MT06 += 0.01;
  pos_speed_ctrl(&hfdcan2, 0x06, arm_motor[Motor6].ctrl.pos_set, 1);

  Pre_Flag_damiao[6] = Flag_damiao[6];
}