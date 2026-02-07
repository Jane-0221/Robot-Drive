#include "Gimbal.h"
#include "Global_status.h"

#include "User_math.h"
#include "pid.h"
#include "ramp_generator.h"

#include "IMU_updata.h"
/*pid*/
pid_t pitch_speed_pid, pitch_location_pid;
pid_t yaw_speed_pid, yaw_location_pid;
pid_t pitch_auto_speed_pid, pitch_auto_location_pid;
pid_t yaw_auto_speed_pid, yaw_auto_location_pid;
/*斜坡*/
RampGenerator pitch_ramp, yaw_ramp;

/**
 * @brief 云台初始化
 *
 */
void Gimbal_init()
{
    /*电机初始化*/
    GIMBALMotor_init(GM6020, YAWMotor);
    GIMBALMotor_init(GM6020, PITCHMotor);
    /*PID速度环初始化*/
    // 遥控
    pid_set(&pitch_speed_pid, 150.0f, 0.0f, 0.0f, GIMBALMOTOR_MAX_CURRENT, 1000);
    pid_set(&yaw_speed_pid, 200.0f, 0.0f, 5.0f, GIMBALMOTOR_MAX_CURRENT, GIMBALMOTOR_MAX_CURRENT);
    // 自瞄
    pid_set(&pitch_auto_speed_pid, 100.0f, 0, 0, GIMBALMOTOR_MAX_CURRENT, 1000);
    pid_set(&yaw_auto_speed_pid, 200.0f, 0.0f, 5.0f, GIMBALMOTOR_MAX_CURRENT, GIMBALMOTOR_MAX_CURRENT);
    /*PID位置环初始化*/
    // 遥控
    pid_set(&pitch_location_pid, 10.0f, 0.0f, 0, GIMBALMOTOR_MAX_CURRENT, 100);
    pid_set(&yaw_location_pid, 14.0f, 0.0f, 0.0f, GIMBALMOTOR_MAX_CURRENT, 1000);
    // 自瞄
    pid_set(&pitch_auto_location_pid, 30.0f, 0.05f, 0, GIMBALMOTOR_MAX_CURRENT, 100);
    pid_set(&yaw_auto_location_pid, 15.0f, 0.0f, 0.0f, GIMBALMOTOR_MAX_CURRENT, 1000);
    /*斜坡初始化*/
    // RampGenerator_Init(&pitch_ramp,1,)
    /*零点与限位*/
    GIMBALMotor_setzero(102.5f, YAWMotor);
}

/**
 * @brief 设置云台PITCHI轴角度
 *
 * @param angle 云台PITCHI轴角度
 */
void Gimbal_set_pitch_angle(float angle)
{
    if (angle < PITCHI_MIN_ANGLE)
        angle = PITCHI_MIN_ANGLE;
    if (angle > PITCHI_MAX_ANGLE)
        angle = PITCHI_MAX_ANGLE;
    Global.Gimbal.input.pitch = angle;
}

/**
 * @brief 设置云台YAW轴角度
 *
 * @param angle 云台YAW轴角度
 */
void Gimbal_set_yaw_angle(float angle)
{
    Global.Gimbal.input.yaw = angle;
}

void Gimbal_control()
{
    float pitch_speed, yaw_speed;
    if (Global.Auto.input.Auto_control_online <= 0 || Global.Auto.mode == NONE)
    { // 非自瞄
        pitch_speed = pid_cal(&pitch_location_pid, RAD_TO_DEG * IMU_data.AHRS.pitch, Global.Gimbal.input.pitch);
        yaw_speed = pid_cal(&yaw_location_pid, RAD_TO_DEG * IMU_data.AHRS.yaw_rad_cnt, Global.Gimbal.input.yaw);
        GIMBALMotor_set(-pid_cal(&pitch_speed_pid, -RAD_TO_DEG * IMU_data.gyro[0], pitch_speed), PITCHMotor);
        GIMBALMotor_set(pid_cal(&yaw_speed_pid, (cos(IMU_data.AHRS.pitch) * RAD_TO_DEG * IMU_data.gyro[2] - sin(IMU_data.AHRS.pitch) * RAD_TO_DEG * IMU_data.gyro[1]), yaw_speed), YAWMotor);
    }
    else
    { // 自瞄
        pitch_speed = pid_cal(&pitch_auto_location_pid, RAD_TO_DEG * IMU_data.AHRS.pitch, Global.Gimbal.input.pitch);
        yaw_speed = pid_cal(&yaw_auto_location_pid, RAD_TO_DEG * IMU_data.AHRS.yaw_rad_cnt, Global.Gimbal.input.yaw);
        GIMBALMotor_set(-pid_cal(&pitch_auto_speed_pid, -RAD_TO_DEG * IMU_data.gyro[0], pitch_speed), PITCHMotor);
        GIMBALMotor_set(pid_cal(&yaw_auto_speed_pid, (cos(IMU_data.AHRS.pitch) * RAD_TO_DEG * IMU_data.gyro[2] - sin(IMU_data.AHRS.pitch) * RAD_TO_DEG * IMU_data.gyro[1]), yaw_speed), YAWMotor);
        Global.Auto.input.Auto_control_online--;
    }
}
