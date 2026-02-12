#include "arm.h"
#include "dm4310_drv.h"
#include "Robstride04.h"
#include "pid.h"
#include "remote_control.h"
#include "gpio.h"
#include "gom_protocol.h"
#include "usart.h"

#define RS485_REDE_Pin GPIO_PIN_14
#define RS485_REDE_GPIO_Port GPIOB

#define RS485_RxMode() (HAL_GPIO_WritePin(RS485_REDE_GPIO_Port, RS485_REDE_Pin, GPIO_PIN_RESET))
#define RS485_TxMode() (HAL_GPIO_WritePin(RS485_REDE_GPIO_Port, RS485_REDE_Pin, GPIO_PIN_SET))

// 宇树
// #define RS485_RxMode() (HAL_GPIO_WritePin(RS485_REDE_GPIO_Port, RS485_REDE_Pin, GPIO_PIN_RESET))
// #define RS485_TxMode() (HAL_GPIO_WritePin(RS485_REDE_GPIO_Port, RS485_REDE_Pin, GPIO_PIN_SET))
MotorCmd_t cmd = {0};
MotorData_t data = {0};
HAL_StatusTypeDef tx_res;
HAL_StatusTypeDef rx_res;

extern Motor_DM_Status DM_Status[6];
RobStride_Motor_t motor1; // ID为1的电机对象

void Arm_Init()
{

  /*                                        灵足                                                  */
  // 1. 初始化电机（使用私有协议，非MIT模式）
  RobStride_Motor_Init(&motor1, 0x01, false); // ID=0x01, MIT模式=false

  // 2. 设置为位置控制模式（CSP模式）
  // 注意：这里我们使用私有协议的CSP控制模式
  // 首先读取当前模式（修改：补充hcan参数，传入&hfdcan2并转换为hcan_t*）
  Get_RobStride_Motor_parameter(&motor1, &hfdcan2, 0X7005);

  // 等待一小段时间让电机响应（实际应用中应该使用更可靠的通信机制）
  HAL_Delay(10);

  // 设置电机为CSP位置模式（修改：补充hcan参数）
  Set_RobStride_Motor_parameter(&motor1, &hfdcan2, 0X7005, CSP_control_mode, 'j');

  // 3. 使能电机（修改：补充hcan参数）
  Enable_Motor(&motor1, (hcan_t *)&hfdcan2);

  // 4. 设置位置控制参数
  // 设置速度限制为1 rad/s（修改：补充hcan参数）
  Set_RobStride_Motor_parameter(&motor1, &hfdcan2, 0X7017, 1.0f, 'p');

  HAL_Delay(10); // 等待参数设置完成
                 /*                                        灵足                                                 */

  /*                                        宇树                                                  */

  // RS485_RxMode();
  // 1. 初始化指令参数（确保参数在电机允许范围）
  cmd.id = 3;      // 目标电机ID=3
  cmd.mode = 1;    // FOC闭环模式（需电机支持）
  cmd.K_P = 0.02f; // 刚度系数（0~25.599，建议先设5.0）
  cmd.K_W = 0.0f;  // 阻尼系数（0~25.599，建议先设5.0）
  cmd.Pos = 20.0f; // 目标位置（rad，先设小值1rad，避免电机动作过大）
  cmd.W = 0.0f;    // 目标速度（rad/s，先设0）
  cmd.T = 0.0f;    // 目标扭矩（Nm，先设0，靠位置环驱动）
                   /*                                        宇树                                                  */

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
  // pos_motor.MT05 = 3;
  /////                                          达妙电机                                     ///////
}
void Arm_motor1()
{
  RobStride_Motor_CSP_control(&motor1, &hfdcan2, 20.0f, 1.0f);
}
void Arm_motor3()
{

   RS485_TxMode();
  HAL_Delay(10);
  modify_data(&cmd);
  HAL_UART_Transmit(&huart3, (uint8_t *)&cmd.motor_send_data, sizeof(cmd.motor_send_data), 0xFFFF);
  // modify_data(&cmd);

  // RS485_TxMode();
  // tx_res = HAL_UART_Transmit(&huart1, (uint8_t *)&cmd.motor_send_data, sizeof(cmd.motor_send_data), 0xFFFF);

  // RS485_RxMode();
  // rx_res = HAL_UART_Receive(&huart1, (uint8_t *)&data.motor_recv_data, sizeof(data.motor_recv_data), 0xFFFF);
  // if (rx_res == HAL_TIMEOUT)
  // {
  //   data.timeout++;
  // }

  // extract_data(&data);

  // if (data.MError != 0)
  // {
  //   /* 电机MError不等于0 代表电机进入了错误状态 此时电机进入阻尼模式 且不会响应任何运动控制指令 */
  //   /* 你的机器人系统需要为此 Do something ... 例如: 控制其他电机进入阻尼状态 防止继续运动 */
  // }

  // HAL_Delay(1);
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
  pos_speed_ctrl(&hfdcan2, 0x04, 5, 10);

  pos_motor.MT04 = 20;
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