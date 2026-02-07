#include "Chassis.h"
#include "Global_status.h"
#include "Gimbal.h"
#include "ui.h"
#include "IMU_updata.h"
#include "referee_system.h"
#include "supercup.h"
#include "stm32_time.h"
#include "remote_control.h"
#include "pid.h"
#include "ramp_generator.h"
#include "dm4310_drv.h"
#include "User_math.h"
#include "DT7.h"
#include "stdio.h"
#define PI 3.1415926
pid_t chassis_pos_pid_FL, chassis_pos_pid_FR, chassis_pos_pid_BL, chassis_pos_pid_BR;
pid_t chassis_speed_FL, chassis_speed_FR, chassis_speed_BL, chassis_speed_BR;
pid_t chassis_speed_pid_FL, chassis_speed_pid_FR, chassis_speed_pid_BL, chassis_speed_pid_BR; // 轮子速度控制Pid
RampGenerator Vx_ramp, Vy_ramp;
struct chassis_status chassis;

extern pid_t yaw_auto_location_pid;

pid_t chassis_follow_pid; // 底盘跟随

/**
 * @brief 底盘初始化
 *
 */
void Chassis_init()
{
    /*电机初始化*/
    CHASSISMotor_init(M3508_P, WHEEL_FL);
    CHASSISMotor_init(M3508_P, WHEEL_FR);
    CHASSISMotor_init(M3508_P, WHEEL_BL);
    CHASSISMotor_init(M3508_P, WHEEL_BR);

    ///////双环////////
    pid_set(&chassis_pos_pid_FR, 1, 0, 0, 1000, 5000);
    pid_set(&chassis_pos_pid_FL, 1, 0, 0, 1000, 5000);
    pid_set(&chassis_pos_pid_BL, 1, 0, 0, 1000, 5000);
    pid_set(&chassis_pos_pid_BR, 1, 0, 0, 1000, 5000);

    pid_set(&chassis_speed_FR, 15, 0, 100, 12000, 5000);
    pid_set(&chassis_speed_FL, 15, 0, 100, 12000, 5000);
    pid_set(&chassis_speed_BL, 15, 0, 100, 12000, 5000);
    pid_set(&chassis_speed_BR, 15, 0, 100, 12000, 5000);
    pid_set(&chassis_follow_pid, 1.0f, 0.0f, 200.0f, 200, 40);
}

/**
 * @brief 设置底盘移动速度
 *
 */
void Chassis_set_x(float x)
{
    // RampGenerator_SetTarget(&Vx_ramp, x);
    Global.Chssis.input.x = x;
}
void Chassis_set_y(float y)
{
    // RampGenerator_SetTarget(&Vy_ramp, y);
    Global.Chssis.input.y = y;
}
void Chassis_set_r(float r)
{
    Global.Chssis.input.r = r;
}

void Chassis_set_accel(float acc)
{
    RampGenerator_SetAccel(&Vx_ramp, acc);
    RampGenerator_SetAccel(&Vy_ramp, acc);
}
void status()
{
    if (IF_KEY_PRESSED_W || IF_KEY_PRESSED_S || IF_KEY_PRESSED_A || IF_KEY_PRESSED_D || IF_KEY_PRESSED_Q || IF_KEY_PRESSED_E || remote_control.mouse_x > 30.0f || remote_control.mouse_x < -30.0f)
    {
        chassis.status = move;
    }
    else
    {
        chassis.status = park;
    }
}
/**
 * @brief 底盘电机控制
 *
 * @param FL_speed 左前轮速度
 * @param FR_speed 右前轮速度
 * @param BL_speed 左后轮速度
 * @param BR_speed 右后轮速度
 */
float Vx_now, Vy_now, W_now;
float T_FR, T_FL, T_BR, T_BL, K;
void ChassisMotor_Control(float Vx_speed, float Vy_speed, float R_speed)
{
    if (arm_connect_status == ARM_CONNECT_STATUS_CONNECTED)
    {
        // 单环//
        pid_set(&chassis_speed_pid_FR, 8000, 0, 0, 16384, 5000);
        pid_set(&chassis_speed_pid_FL, 8000, 0, 0, 16384, 5000);
        pid_set(&chassis_speed_pid_BL, 6000, 0, 0, 16384, 5000);
        pid_set(&chassis_speed_pid_BR, 6000, 0, 0, 16384, 5000);
    }
     if (arm_connect_status == ARM_CONNECT_STATUS_DISCONNECTED)
    {
        // 单环//
        pid_set(&chassis_speed_pid_FR, 8000, 0, 0, 16384, 5000);
        pid_set(&chassis_speed_pid_FL, 10000, 0, 0, 16384, 5000);
        pid_set(&chassis_speed_pid_BL, 8000, 0, 0, 16384, 5000);
        pid_set(&chassis_speed_pid_BR, 10000, 0, 0, 16384, 5000);
    }
    float current[4] = {0};
    float FL_speed, FR_speed, BL_speed, BR_speed;

    float CHASSIS_WZ_SET_SCALE = 0.0f; // 底盘转角速度设置比例

    /*逆解轮速*/
    FR_speed = +Vx_speed - Vy_speed + (1 + CHASSIS_WZ_SET_SCALE) * R_speed;
    FL_speed = +Vx_speed + Vy_speed + (1 + CHASSIS_WZ_SET_SCALE) * R_speed;
    BL_speed = -Vx_speed + Vy_speed + (1 - CHASSIS_WZ_SET_SCALE) * R_speed;
    BR_speed = -Vx_speed - Vy_speed + (1 - CHASSIS_WZ_SET_SCALE) * R_speed;

    /*正解车速*/
    Vx_now = CHASSISMotor_get_data(WHEEL_FL).speed_rpm / 2.0f - CHASSISMotor_get_data(WHEEL_BL).speed_rpm / 2.0f;
    Vy_now = CHASSISMotor_get_data(WHEEL_FL).speed_rpm / 2.0f - CHASSISMotor_get_data(WHEEL_FR).speed_rpm / 2.0f;
    W_now = CHASSISMotor_get_data(WHEEL_FR).speed_rpm / 2.0f + CHASSISMotor_get_data(WHEEL_BL).speed_rpm / 2.0f;

    chassis.angle_now[WHEEL_FL] = CHASSISMotor_get_data(WHEEL_FL).angle_cnt;
    chassis.angle_now[WHEEL_FR] = CHASSISMotor_get_data(WHEEL_FR).angle_cnt;
    chassis.angle_now[WHEEL_BL] = CHASSISMotor_get_data(WHEEL_BL).angle_cnt;
    chassis.angle_now[WHEEL_BR] = CHASSISMotor_get_data(WHEEL_BR).angle_cnt;

    // 双环
    if (chassis.status == park && IMU_data.AHRS.pitch >= 0.3f)
    { //
        chassis.wheel_speed[WHEEL_FR] = pid_cal(&chassis_pos_pid_FR, chassis.angle_now[WHEEL_FR], chassis.angle_set[WHEEL_FR]);
        chassis.wheel_speed[WHEEL_FL] = pid_cal(&chassis_pos_pid_FL, chassis.angle_now[WHEEL_FL], chassis.angle_set[WHEEL_FL]);
        chassis.wheel_speed[WHEEL_BL] = pid_cal(&chassis_pos_pid_BL, chassis.angle_now[WHEEL_BL], chassis.angle_set[WHEEL_BL]);
        chassis.wheel_speed[WHEEL_BR] = pid_cal(&chassis_pos_pid_BR, chassis.angle_now[WHEEL_BR], chassis.angle_set[WHEEL_BR]);

        current[1] = pid_cal(&chassis_speed_FR, (CHASSISMotor_get_data(WHEEL_FR).speed_rpm) / 19.0f * 0.104719755 * r_wheel, chassis.wheel_speed[WHEEL_FR]); // 右前
        current[3] = pid_cal(&chassis_speed_BR, (CHASSISMotor_get_data(WHEEL_BR).speed_rpm) / 19.0f * 0.104719755 * r_wheel, chassis.wheel_speed[WHEEL_BR]);
        current[0] = pid_cal(&chassis_speed_FL, (CHASSISMotor_get_data(WHEEL_FL).speed_rpm) / 19.0f * 0.104719755 * r_wheel, chassis.wheel_speed[WHEEL_FL]);
        current[2] = pid_cal(&chassis_speed_BL, (CHASSISMotor_get_data(WHEEL_BL).speed_rpm) / 19.0f * 0.104719755 * r_wheel, chassis.wheel_speed[WHEEL_BL]);
        goto end;
    }
    /*电流计算*/
    /*电流计算*/
    current[1] = pid_cal(&chassis_speed_pid_FR, (CHASSISMotor_get_data(WHEEL_FR).speed_rpm) / 19.0f * 0.104719755 * r_wheel, FR_speed); // 右前
    current[3] = pid_cal(&chassis_speed_pid_BR, (CHASSISMotor_get_data(WHEEL_BR).speed_rpm) / 19.0f * 0.104719755 * r_wheel, BR_speed);
    current[0] = pid_cal(&chassis_speed_pid_FL, (CHASSISMotor_get_data(WHEEL_FL).speed_rpm) / 19.0f * 0.104719755 * r_wheel, FL_speed);
    current[2] = pid_cal(&chassis_speed_pid_BL, (CHASSISMotor_get_data(WHEEL_BL).speed_rpm) / 19.0f * 0.104719755 * r_wheel, BL_speed);

    chassis.angle_set[WHEEL_FL] = CHASSISMotor_get_data(WHEEL_FL).angle_cnt;
    chassis.angle_set[WHEEL_FR] = CHASSISMotor_get_data(WHEEL_FR).angle_cnt;
    chassis.angle_set[WHEEL_BL] = CHASSISMotor_get_data(WHEEL_BL).angle_cnt;
    chassis.angle_set[WHEEL_BR] = CHASSISMotor_get_data(WHEEL_BR).angle_cnt;

end:
    for (int i = 0; i < 4; i++)
    {
        if (current[i] >= (CHASSISMOTOR_MAX_CURRENT - 1))
            current[i] = CHASSISMOTOR_MAX_CURRENT - 1;
        if (current[i] <= -(CHASSISMOTOR_MAX_CURRENT - 1))
            current[i] = -(CHASSISMOTOR_MAX_CURRENT - 1);
    }

    /*电流设置*/
    CHASSISMotor_set(current[0], WHEEL_FL);
    CHASSISMotor_set(current[1], WHEEL_FR);
    CHASSISMotor_set(current[2], WHEEL_BL);
    CHASSISMotor_set(current[3], WHEEL_BR);
}

/**
 * @brief 底盘移动解算
 *
 */
float relative_angle;
float flow_speed(float relative)
{
    if (abs(relative) > 120)
        return 0.5;
    else if (abs(relative) > 60)
        return 0.8;
    else
        return 1;
}
void Chassis_move()
{
    float X_speed, Y_speed, R_speed;
    float sin_beta, cos_beta;
    /*化简多圈角度*/
    // uint32_t mul;
    // mul = fabs(relative_angle) / 180.0f;
    // if (relative_angle > 180.0f)
    // {
    //     if (mul % 2 == 1) // 处于-180度
    //         relative_angle -= (mul + 1) * 180.0f;
    //     else // 处于180度
    //         relative_angle -= mul * 180.0f;
    // }
    // if (relative_angle < -180.0f)
    // {
    //     if (mul % 2 == 1) // 处于180度
    //         relative_angle += (mul + 1) * 180.0f;
    //     else // 处于-180度
    //         relative_angle += mul * 180.0f;
    // }
    // // relative_angle = 0;
    // sin_beta = sinf(relative_angle / 180.0f * PI);
    // cos_beta = cosf(relative_angle / 180.0f * PI);
    /*运动分解*/
    // X_speed = Global.Chssis.input.x * cos_beta - sin_beta * Global.Chssis.input.y;
    // Y_speed = Global.Chssis.input.x * sin_beta + Global.Chssis.input.y * cos_beta;
    // R_speed = Global.Chssis.input.r;
    X_speed = Global.Chssis.input.x;
    Y_speed = Global.Chssis.input.y;

    R_speed = Global.Chssis.input.r;

    // Chassisui_change(relative_angle);//
    /*模式選擇*/

    // switch (Global.Chssis.mode)
    // {
    // case FLOW:
    //     // if(Global.Auto.input.Auto_control_online <= 0 || Global.Auto.mode == NONE)
    //     // // R_speed = pid_cal(&chassis_follow_pid, DEG_TO_RAD * relative_angle, 0.0f) - 0.9 * yaw_location_pid.total_out * DEG_TO_RAD;
    //     // R_speed = 0;
    //     // else

    //     R_speed = pid_cal(&chassis_follow_pid, DEG_TO_RAD * relative_angle, 0.0f);
    //     relative_angle = (total_angle * RAD_TO_DEG);
    //     //    relative_angle = 0;
    //     break;
    // case SPIN_P:
    //     R_speed = 1.0f;
    //     relative_angle = (total_angle * RAD_TO_DEG) - arm_motor[Motor1].para.vel * (60 / 2 * PI) * (-0.0f); //-10.9
    //     break;
    // case SPIN_N:
    //     R_speed = -1.0f;
    //     relative_angle = (total_angle * RAD_TO_DEG) - arm_motor[Motor1].para.vel * (60 / 2 * PI) * (-0.0f);
    //     break;
    // }

    /*底盘电机控制*/
    ChassisMotor_Control(X_speed, Y_speed, R_speed);
}