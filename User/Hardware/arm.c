#include "arm.h"
#include "dm4310_drv.h"
#include "Robstride04.h"
#include "pid.h"
#include "remote_control.h"
extern Motor_DM_Status DM_Status[6];
 RobStride_Motor_t motor1;  // ID为1的电机对象
void Arm_Init()
{
/*                                        灵足                                                  */
// 1. 初始化电机（使用私有协议，非MIT模式）
RobStride_Motor_Init(&motor1, 0x01, false);  // ID=0x01, MIT模式=false

// 2. 设置为位置控制模式（CSP模式）
// 注意：这里我们使用私有协议的CSP控制模式
// 首先读取当前模式（修改：补充hcan参数，传入&hfdcan2并转换为hcan_t*）
Get_RobStride_Motor_parameter(&motor1, &hfdcan2, 0X7005);

// 等待一小段时间让电机响应（实际应用中应该使用更可靠的通信机制）
HAL_Delay(10);

// 设置电机为CSP位置模式（修改：补充hcan参数）
Set_RobStride_Motor_parameter(&motor1, &hfdcan2, 0X7005, CSP_control_mode, 'j');

// 3. 使能电机（修改：补充hcan参数）
Enable_Motor(&motor1, (hcan_t*)&hfdcan2);

// 4. 设置位置控制参数
// 设置速度限制为1 rad/s（修改：补充hcan参数）
Set_RobStride_Motor_parameter(&motor1, &hfdcan2, 0X7017, 1.0f, 'p');

HAL_Delay(10);  // 等待参数设置完成
/*                                        灵足电机                                                  */





/////                                          达妙电机                                     ///////

  arm_motor_init(&arm_motor[Motor4], 0x04, POS_MODE); // 设置ID和控制模式  
  arm_motor_init(&arm_motor[Motor5], 0x05, POS_MODE); // 设置ID和控制模式  
  arm_motor_init(&arm_motor[Motor3], 0x06, POS_MODE); // 设置ID和控制模式  

  enable_motor_mode(&hfdcan2, 0x04, POS_MODE);
  enable_motor_mode(&hfdcan2, 0x05, POS_MODE);
  enable_motor_mode(&hfdcan2, 0x06, POS_MODE);
  // 速度

  vel_motor.MT04 = 5.0;
  // 位置

  pos_motor.MT04 = 0;
    //pos_motor.MT05 = 3;
/////                                          达妙电机                                     ///////
}
void Arm_motor1()
{



    RobStride_Motor_CSP_control(&motor1,&hfdcan2, 20.0f, 1.0f);




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

  set_DM_mode(Motor4, POS_MODE);
  set_DM_pos_vel(pos_motor.MT04, vel_motor.MT04, Motor4);
  pos_speed_ctrl(&hfdcan2, 0x04, 5,10);

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
  set_DM_mode(Motor5, POS_MODE);
  set_DM_pos_vel(pos_motor.MT05, vel_motor.MT05, Motor5);

  pos_speed_ctrl(&hfdcan2, 0x05, 10, 1);

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