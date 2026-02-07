#include "dm_arm.h"
#include "motor.h"
#include "dm4310_drv.h"
#include "pid.h"
#include "motor.h"
#include "remote_control.h"
Motor_DM_Status DM_Status[6] = {Motor_DM_Status_DISABLE};
// DJI

Endrow_motor endrow_motor;
// pid
pid_t endrow_location_pid;
pid_t endrow_speed_pid;

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

  // 速度
  vel_motor.vel_rootpitch = 10.0;
  vel_motor.vel_rootyaw = 3.0;
  vel_motor.vel_middleyaw = 1.0;
  vel_motor.vel_middlerow = 4.0;
  vel_motor.vel_endyaw = 5.0;
  // 位置
  pos_motor.pos_endyaw = 0;
  pos_motor.pos_middlerow = 0;
  pos_motor.pos_middleyaw = 0;
  pos_motor.pos_rootpitch = 0;
  pos_motor.pos_rootyaw = 0;
  // dji
  ROWMotor_init(M2006, End_ROW_MOTOR);
  // pid_set(&endrow_speed_pid, 1.2f, 0.0f, 5.0f, 8000.0f, 0.01f);
  //   pid_set(&endrow_location_pid, 28.0f, 0.0f, 70.0f, 4000.0f, 30.0f); // 20/0/10
  pid_set(&endrow_speed_pid, 35.0f, 0.0f, 10.0f, 8000.0f, 0.01f);
  pid_set(&endrow_location_pid, 3.0f, 0.01f, 5.0f, 4000.0f, 30.0f); // 20/0/10
} // 35 0 200  2 0  0
void Arm_DJI_updata()
{
  endrow_motor.now = ROWMotor_get_data(End_ROW_MOTOR).angle_cnt / 60; //
  endrow_motor.endrow_speed = ROWMotor_get_data(End_ROW_MOTOR).round_speed;
  //  endrow_motor.now = DJIMotor_data[1][5].;
}
void Arm_DJI_pid_cal()
{
  // total_angle = 0;
  endrow_motor.set_endrow_speed = pid_cal(&endrow_location_pid, endrow_motor.now, total_angle);
  // ROWMotor_set(pid_cal(&endrow_speed_pid, endrow_motor.endrow_speed, endrow_motor.set_endrow_speed), End_ROW_MOTOR);
  ROWMotor_set(pid_cal(&endrow_speed_pid, endrow_motor.endrow_speed, endrow_motor.set_endrow_speed), End_ROW_MOTOR);
}

void Arm_DJImotor()
{
  Arm_DJI_updata();
  Arm_DJI_pid_cal();
}

void Arm_motor1()
{

  // 判断该时间段内是否接收过电机数据
  if (Flag_damiao[1] == Pre_Flag_damiao[1])
  {
    // 电机断开连接
    DM_Status[1] = Motor_DM_Status_DISABLE;
    // 错误清除，重新使能
    // CAN_Send_Clear_Error(&hfdcan1,0x01);
    CAN_Send_Enter(&hfdcan1, 0x05);
  }
  else
  {
    // 电机保持连接
    DM_Status[1] = Motor_DM_Status_ENABLE;
    // 正常数据发送
      if (pos_motor.pos_rootyaw >= 1.55f)
    {
      pos_motor.pos_rootyaw = 1.55f;
    }
     if (pos_motor.pos_rootyaw<= -1.55f)
    {
      pos_motor.pos_rootyaw = -1.55f;
    }
    set_DM_mode(Motor1, POS_MODE);
    set_DM_pos_vel(pos_motor.pos_rootyaw, vel_motor.vel_rootyaw, Motor1);
    pos_speed_ctrl(&hfdcan1, 0x05, arm_motor[Motor1].ctrl.pos_set, arm_motor[Motor1].ctrl.vel_set);
  }
  Pre_Flag_damiao[1] = Flag_damiao[1];
}

void Arm_motor2()
{

  // 判断该时间段内是否接收过电机数据
  if (Flag_damiao[2] == Pre_Flag_damiao[2])
  {
    // 电机断开连接
    DM_Status[2] = Motor_DM_Status_DISABLE;
    // 错误清除，重新使能
    // CAN_Send_Clear_Error(&hfdcan2,0x02);
    CAN_Send_Enter(&hfdcan2, 0x02);
  }
  else
  {
    // 电机保持连接
    DM_Status[2] = Motor_DM_Status_ENABLE;
    // 正常数据发送
    if (pos_motor.pos_rootpitch >= 0.86f)
    {
      pos_motor.pos_rootpitch = 0.86f;
    }
     if (pos_motor.pos_rootpitch <= -0.17f)
    {
      pos_motor.pos_rootpitch = -0.17f;
    }
    set_DM_mode(Motor2, POS_MODE);
    set_DM_pos_vel(pos_motor.pos_rootpitch, vel_motor.vel_rootpitch, Motor2);
    pos_speed_ctrl(&hfdcan2, 0x02, arm_motor[Motor2].ctrl.pos_set, arm_motor[Motor2].ctrl.vel_set);
  }
  Pre_Flag_damiao[2] = Flag_damiao[2];
}
void Arm_motor3()
{

  // 判断该时间段内是否接收过电机数据
  if (Flag_damiao[3] == Pre_Flag_damiao[3])
  {
    // 电机断开连接
    DM_Status[3] = Motor_DM_Status_DISABLE;
    // 错误清除，重新使能
    //	CAN_Send_Clear_Error(&hfdcan2,0x03);
    CAN_Send_Enter(&hfdcan2, 0x03);
  }
  else
  {
    // 电机保持连接
    DM_Status[3] = Motor_DM_Status_ENABLE;
    // 正常数据发送
      if (pos_motor.pos_middleyaw >= 2.1f)
    {
      pos_motor.pos_middleyaw = 2.1f;
    }
     if (pos_motor.pos_middleyaw <= -2.1f)
    {
      pos_motor.pos_middleyaw = -2.1f;
    }
    set_DM_mode(Motor3, POS_MODE);
    set_DM_pos_vel(pos_motor.pos_middleyaw, vel_motor.vel_middleyaw, Motor3);
    mit_ctrl(&hfdcan2, 0x03, arm_motor[Motor3].ctrl.pos_set, 1.5, 100, 50, 6);
  }

  Pre_Flag_damiao[3] = Flag_damiao[3];
}
void Arm_motor4()
{

  // 判断该时间段内是否接收过电机数据
  if (Flag_damiao[4] == Pre_Flag_damiao[4])
  {
    // 电机断开连接
    DM_Status[4] = Motor_DM_Status_DISABLE;
    // 错误清除，重新使能
    // CAN_Send_Clear_Error(&hfdcan2,0x04);
    CAN_Send_Enter(&hfdcan2, 0x04);
  }
  else
  {
    // 电机保持连接
    DM_Status[4] = Motor_DM_Status_ENABLE;
    // 正常数据发送
      if (pos_motor.pos_middlerow >= 3.4f)
    {
      pos_motor.pos_middlerow = 3.4f;
    }
     if (pos_motor.pos_middlerow <= -1.9f)
    {
      pos_motor.pos_middlerow = -1.9f;
    }
    set_DM_mode(Motor4, POS_MODE);
    set_DM_pos_vel(pos_motor.pos_middlerow, vel_motor.vel_middlerow, Motor4);
    pos_speed_ctrl(&hfdcan2, 0x04, arm_motor[Motor4].ctrl.pos_set, arm_motor[Motor4].ctrl.vel_set);
  }

  Pre_Flag_damiao[4] = Flag_damiao[4];
}

void Arm_motor5()
{

  // 判断该时间段内是否接收过电机数据
  if (Flag_damiao[5] == Pre_Flag_damiao[5])
  {
    // 电机断开连接
    DM_Status[5] = Motor_DM_Status_DISABLE;
    // 错误清除，重新使能
    //	CAN_Send_Clear_Error(&hfdcan2,0x05);
    CAN_Send_Enter(&hfdcan2, 0x05);
  }
  else
  {
    // 电机保持连接
    DM_Status[5] = Motor_DM_Status_ENABLE;
    // 正常数据发送
      if (pos_motor.pos_endyaw >= 1.9f)
    {
      pos_motor.pos_endyaw = 1.9f;
    }
     if (pos_motor.pos_endyaw <= -1.9f)
    {
      pos_motor.pos_endyaw = -1.9f;
    }
    set_DM_mode(Motor5, POS_MODE);
    set_DM_pos_vel(pos_motor.pos_endyaw, vel_motor.vel_endyaw, Motor5);
    pos_speed_ctrl(&hfdcan2, 0x05, arm_motor[Motor5].ctrl.pos_set, arm_motor[Motor5].ctrl.vel_set);
  }
  Pre_Flag_damiao[5] = Flag_damiao[5];
}
/// @brief ////////////////行走////////////
void Arm_motor1_DISCON()
{

  // 判断该时间段内是否接收过电机数据
  if (Flag_damiao[1] == Pre_Flag_damiao[1])
  {
    // 电机断开连接
    DM_Status[1] = Motor_DM_Status_DISABLE;
    // 错误清除，重新使能
    // CAN_Send_Clear_Error(&hfdcan1,0x01);
    CAN_Send_Enter(&hfdcan1, 0x05);
  }
  else
  {
    // 电机保持连接
    DM_Status[1] = Motor_DM_Status_ENABLE;
    // 正常数据发送
        DM_Status[1] = Motor_DM_Status_ENABLE;
    // 正常数据发送
      if (pos_motor.pos_rootyaw >= 1.55f)
    {
      pos_motor.pos_rootyaw = 1.55f;
    }
     if (pos_motor.pos_rootyaw<= -1.55f)
    {
      pos_motor.pos_rootyaw = -1.55f;
    }
    set_DM_mode(Motor1, POS_MODE);
    set_DM_pos_vel(pos_motor.pos_rootyaw, vel_motor.vel_rootyaw, Motor1);
    pos_speed_ctrl(&hfdcan1, 0x05, arm_motor[Motor1].ctrl.pos_set, 0.6);
  }
    Pre_Flag_damiao[1] = Flag_damiao[1];

}
void Arm_motor2_DISCON()
{

  // 判断该时间段内是否接收过电机数据
  if (Flag_damiao[2] == Pre_Flag_damiao[2])
  {
    // 电机断开连接
    DM_Status[2] = Motor_DM_Status_DISABLE;
    // 错误清除，重新使能
    // CAN_Send_Clear_Error(&hfdcan2,0x02);
    CAN_Send_Enter(&hfdcan2, 0x02);
  }
  else
  {
    // 电机保持连接
    DM_Status[2] = Motor_DM_Status_ENABLE;
    // 正常数据发送
     if (pos_motor.pos_rootpitch >= 0.8f)
    {
      pos_motor.pos_rootpitch = 0.8f;
    }
     if (pos_motor.pos_rootpitch <= -0.17f)
    {
      pos_motor.pos_rootpitch = -0.17f;
    }
    set_DM_mode(Motor2, POS_MODE);
    set_DM_pos_vel(pos_motor.pos_rootpitch, vel_motor.vel_rootpitch, Motor2);
    pos_speed_ctrl(&hfdcan2, 0x02, arm_motor[Motor2].ctrl.pos_set, 0.6);
  }
  Pre_Flag_damiao[2] = Flag_damiao[2];
}
void Arm_motor3_DISCON()
{

  // 判断该时间段内是否接收过电机数据
  if (Flag_damiao[3] == Pre_Flag_damiao[3])
  {
    // 电机断开连接
    DM_Status[3] = Motor_DM_Status_DISABLE;
    // 错误清除，重新使能
    //	CAN_Send_Clear_Error(&hfdcan2,0x03);
    CAN_Send_Enter(&hfdcan2, 0x03);
  }
  else
  {
    // 电机保持连接
    DM_Status[3] = Motor_DM_Status_ENABLE;
    // 正常数据发送
          if (pos_motor.pos_middleyaw >= 2.1f)
    {
      pos_motor.pos_middleyaw = 2.1f;
    }
     if (pos_motor.pos_middleyaw <= -2.1f)
    {
      pos_motor.pos_middleyaw = -2.1f;
    }
    set_DM_mode(Motor3, POS_MODE);
    set_DM_pos_vel(pos_motor.pos_middleyaw, vel_motor.vel_middleyaw, Motor3);
    mit_ctrl(&hfdcan2, 0x03, arm_motor[Motor3].ctrl.pos_set, 0.01, 10, 30, 1.8);//0.01/15/42/2出     //0.01/10/30/1.8收可以
  }

  Pre_Flag_damiao[3] = Flag_damiao[3];
}
void Arm_motor4_DISCON()
{

  // 判断该时间段内是否接收过电机数据
  if (Flag_damiao[4] == Pre_Flag_damiao[4])
  {
    // 电机断开连接
    DM_Status[4] = Motor_DM_Status_DISABLE;
    // 错误清除，重新使能
    // CAN_Send_Clear_Error(&hfdcan2,0x04);
    CAN_Send_Enter(&hfdcan2, 0x04);
  }
  else
  {
    // 电机保持连接
    DM_Status[4] = Motor_DM_Status_ENABLE;
    // 正常数据发送
          if (pos_motor.pos_middlerow >= 3.4f)
    {
      pos_motor.pos_middlerow = 3.4f;
    }
     if (pos_motor.pos_middlerow <= -1.9f)
    {
      pos_motor.pos_middlerow = -1.9f;
    }
    set_DM_mode(Motor4, POS_MODE);
    set_DM_pos_vel(pos_motor.pos_middlerow, vel_motor.vel_middlerow, Motor4);
    pos_speed_ctrl(&hfdcan2, 0x04, arm_motor[Motor4].ctrl.pos_set, 2);
  }

  Pre_Flag_damiao[4] = Flag_damiao[4];
}
void Arm_motor5_DISCON()
{

  // 判断该时间段内是否接收过电机数据
  if (Flag_damiao[5] == Pre_Flag_damiao[5])
  {
    // 电机断开连接
    DM_Status[5] = Motor_DM_Status_DISABLE;
    // 错误清除，重新使能
    //	CAN_Send_Clear_Error(&hfdcan2,0x05);
    CAN_Send_Enter(&hfdcan2, 0x05);
  }
  else
  {
    // 电机保持连接
    DM_Status[5] = Motor_DM_Status_ENABLE;
    // 正常数据发送
          if (pos_motor.pos_endyaw >= 1.9f)
    {
      pos_motor.pos_endyaw = 1.9f;
    }
     if (pos_motor.pos_endyaw <= -1.9f)
    {
      pos_motor.pos_endyaw = -1.9f;
    }
    set_DM_mode(Motor5, POS_MODE);
    set_DM_pos_vel(pos_motor.pos_endyaw, vel_motor.vel_endyaw, Motor5);
    pos_speed_ctrl(&hfdcan2, 0x05, arm_motor[Motor5].ctrl.pos_set, 2.0);
  }
  Pre_Flag_damiao[5] = Flag_damiao[5];
}
void Arm_motor3_DISCON_OUT()
{

  // 判断该时间段内是否接收过电机数据
  if (Flag_damiao[3] == Pre_Flag_damiao[3])
  {
    // 电机断开连接
    DM_Status[3] = Motor_DM_Status_DISABLE;
    // 错误清除，重新使能
    //	CAN_Send_Clear_Error(&hfdcan2,0x03);
    CAN_Send_Enter(&hfdcan2, 0x03);
  }
  else
  {
    // 电机保持连接
    DM_Status[3] = Motor_DM_Status_ENABLE;
    // 正常数据发送
          if (pos_motor.pos_middleyaw >= 2.1f)
    {
      pos_motor.pos_middleyaw = 2.1f;
    }
     if (pos_motor.pos_middleyaw <= -2.1f)
    {
      pos_motor.pos_middleyaw = -2.1f;
    }
    set_DM_mode(Motor3, POS_MODE);
    set_DM_pos_vel(pos_motor.pos_middleyaw, vel_motor.vel_middleyaw, Motor3);
    mit_ctrl(&hfdcan2, 0x03, arm_motor[Motor3].ctrl.pos_set, 0.01, 15, 42, 2);//0.01/15/42/2出     //0.01/10/30/1.8收可以
  }

  Pre_Flag_damiao[3] = Flag_damiao[3];
}