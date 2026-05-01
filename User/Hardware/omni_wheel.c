#include "omni_wheel.h"

#include <math.h>

#include "IMU_updata.h"
#include "Robstride04.h"
#include "fdcan.h"
#include "main.h"
#include "pid.h"
#include "ramp_generator.h"

/* 全向轮底盘固定在 FDCAN3。 */
#define OMNI_CAN_HANDLE (&hfdcan3)

/* 电机编号约定：
 * 1 号在后方，2 号在左前，3 号在右前。 */
#define OMNI_MOTOR1_ID 0x01U
#define OMNI_MOTOR2_ID 0x02U
#define OMNI_MOTOR3_ID 0x03U

/* 运动学几何参数：
 * R 为底盘中心到轮中心距离，r 为全向轮半径。 */
#define OMNI_CHASSIS_RADIUS_M 0.195f
#define OMNI_WHEEL_RADIUS_M 0.0635f
#define OMNI_SQRT3_OVER_2 0.8660254f

/* 每个轮子的方向系数，现场发现反转时只改这里。 */
#define OMNI_DIR1 1.0f
#define OMNI_DIR2 1.0f
#define OMNI_DIR3 1.0f

/* RobStride 速度模式保护参数。 */
#define OMNI_CURRENT_LIMIT 10.0f
#define OMNI_ACCEL_LIMIT 10.0f
#define OMNI_SPEED_LIMIT 20.0f

/* Chassis command ramp fixed limits. Slopes are set at the top of Omni_Wheel_Update(). */
#define OMNI_RAMP_INTERVAL_MS 1U
#define OMNI_RAMP_LINEAR_LIMIT 2.3f
#define OMNI_RAMP_YAW_LIMIT 3.5f

/* 航向保持 PID 与启用阈值。 */
#define OMNI_YAW_HOLD_KP 2.0f
#define OMNI_YAW_HOLD_KI 0.02f
#define OMNI_YAW_HOLD_KD 0.1f
#define OMNI_YAW_HOLD_LIM_OUT 1.5f
#define OMNI_YAW_HOLD_LIM_I_OUT 0.5f
#define OMNI_TRANSLATION_ENABLE_THRESHOLD 0.02f
#define OMNI_YAW_CMD_DEADBAND 0.05f

/* 底盘速度指令，全局暴露给其他模块直接赋值。 */
volatile float x = 0.0f;
volatile float y = 0.0f;
volatile float w = 0.0f;
volatile Omni_Wheel_Debug_t omni_debug = {0};

/* 模块内部电机对象，不对外暴露。 */
static RobStride_Motor_t omni_motor1;
static RobStride_Motor_t omni_motor2;
static RobStride_Motor_t omni_motor3;
static uint8_t omni_initialized = 0U;
static RampGenerator omni_x_ramp;
static RampGenerator omni_y_ramp;
static RampGenerator omni_w_ramp;

/* 航向保持状态。
 * target 使用连续角 yaw_rad_cnt，避免跨 -pi/pi 时跳变。 */
static pid_t omni_yaw_hold_pid;
static float omni_yaw_target = 0.0f;
static uint8_t omni_yaw_lock_valid = 0U;

static void Omni_Wheel_Reset_Yaw_Pid(void)
{
    /* 切换锁定状态时清空 PID 内部状态，避免积分残留导致突跳。 */
    omni_yaw_hold_pid.set = 0.0f;
    omni_yaw_hold_pid.err = 0.0f;
    omni_yaw_hold_pid.err_last = 0.0f;
    omni_yaw_hold_pid.diff = 0.0f;
    omni_yaw_hold_pid.p_out = 0.0f;
    omni_yaw_hold_pid.i_out = 0.0f;
    omni_yaw_hold_pid.d_out = 0.0f;
    omni_yaw_hold_pid.total_out = 0.0f;
}

static void Omni_Wheel_Init_Motor(RobStride_Motor_t *motor, uint8_t can_id)
{
    /* 启动流程参考 arm 里的 Linzu 私有协议初始化，
     * 但这里固定切到速度模式。 */
    RobStride_Motor_Init(motor, can_id, false);
    Get_RobStride_Motor_parameter(motor, OMNI_CAN_HANDLE, 0x7005);
    HAL_Delay(10);
    Set_RobStride_Motor_parameter(motor, OMNI_CAN_HANDLE, 0x7005, Speed_control_mode, 'j');
    Enable_Motor(motor, OMNI_CAN_HANDLE);
    Set_RobStride_Motor_parameter(motor, OMNI_CAN_HANDLE, 0x7018, OMNI_CURRENT_LIMIT, 'p');
    Set_RobStride_Motor_parameter(motor, OMNI_CAN_HANDLE, 0x7022, OMNI_ACCEL_LIMIT, 'p');
    HAL_Delay(10);
    RobStride_Motor_ProactiveEscalationSet(motor, OMNI_CAN_HANDLE, 0x01);
}

static void Omni_Wheel_Limit(float *cmd1, float *cmd2, float *cmd3)
{
    float max_abs = fabsf(*cmd1);
    float scale;

    /* 找出 3 个轮速指令中的最大绝对值。 */
    if (fabsf(*cmd2) > max_abs)
    {
        max_abs = fabsf(*cmd2);
    }
    if (fabsf(*cmd3) > max_abs)
    {
        max_abs = fabsf(*cmd3);
    }

    if (max_abs <= OMNI_SPEED_LIMIT)
    {
        return;
    }

    /* 超限时按同一比例缩放，保持原有运动方向不变。 */
    scale = OMNI_SPEED_LIMIT / max_abs;
    *cmd1 *= scale;
    *cmd2 *= scale;
    *cmd3 *= scale;
}

void Omni_Wheel_Init(void)
{
    /* 上电默认底盘静止。 */
    x = 0.0f;
    y = 0.0f;
    w = 0.0f;
    omni_debug = (Omni_Wheel_Debug_t){0};
    RampGenerator_Init(&omni_x_ramp, OMNI_RAMP_INTERVAL_MS, 0.0f, 0.0f, OMNI_RAMP_LINEAR_LIMIT);
    RampGenerator_Init(&omni_y_ramp, OMNI_RAMP_INTERVAL_MS, 0.0f, 0.0f, OMNI_RAMP_LINEAR_LIMIT);
    RampGenerator_Init(&omni_w_ramp, OMNI_RAMP_INTERVAL_MS, 0.0f, 0.0f, OMNI_RAMP_YAW_LIMIT);

    /* 航向保持以上电时当前朝向为初值。 */
    pid_set(&omni_yaw_hold_pid,
            OMNI_YAW_HOLD_KP,
            OMNI_YAW_HOLD_KI,
            OMNI_YAW_HOLD_KD,
            OMNI_YAW_HOLD_LIM_OUT,
            OMNI_YAW_HOLD_LIM_I_OUT);
    omni_yaw_target = IMU_data.AHRS.yaw_rad_cnt;
    omni_yaw_lock_valid = 1U;
    Omni_Wheel_Reset_Yaw_Pid();

    Omni_Wheel_Init_Motor(&omni_motor1, OMNI_MOTOR1_ID);
    Omni_Wheel_Init_Motor(&omni_motor2, OMNI_MOTOR2_ID);
    Omni_Wheel_Init_Motor(&omni_motor3, OMNI_MOTOR3_ID);

    omni_initialized = 1U;
}

void Omni_Wheel_Update(void)
{
    const float ramp_linear_accel = 1.3f;
    const float ramp_linear_decel = 1.83f;
    const float ramp_yaw_accel = 1.5f;
    const float ramp_yaw_decel = 1.83f;

    float x_cmd;
    float y_cmd;
    float w_cmd;
    float x_target;
    float y_target;
    float w_target;
    float w_cmd_in;
    float v_cmd;
    float yaw_now;
    float yaw_correction;
    float cmd1;
    float cmd2;
    float cmd3;
    uint32_t tick_ms;

    if (omni_initialized == 0U)
    {
        return;
    }

    /* 先把全局指令读到局部变量，避免一次计算中被其他任务改写。 */
    RampGenerator_SetAccel(&omni_x_ramp, ramp_linear_accel);
    RampGenerator_SetDecel(&omni_x_ramp, ramp_linear_decel);
    RampGenerator_SetAccel(&omni_y_ramp, ramp_linear_accel);
    RampGenerator_SetDecel(&omni_y_ramp, ramp_linear_decel);
    RampGenerator_SetAccel(&omni_w_ramp, ramp_yaw_accel);
    RampGenerator_SetDecel(&omni_w_ramp, ramp_yaw_decel);

    x_target = x;
    y_target = y;
    w_target = w;
    RampGenerator_SetTarget(&omni_x_ramp, x_target);
    RampGenerator_SetTarget(&omni_y_ramp, y_target);
    RampGenerator_SetTarget(&omni_w_ramp, w_target);

    tick_ms = HAL_GetTick();
    RampGenerator_Update(&omni_x_ramp, tick_ms);
    RampGenerator_Update(&omni_y_ramp, tick_ms);
    RampGenerator_Update(&omni_w_ramp, tick_ms);

    x_cmd = RampGenerator_GetCurrent(&omni_x_ramp);
    y_cmd = RampGenerator_GetCurrent(&omni_y_ramp);
    w_cmd = RampGenerator_GetCurrent(&omni_w_ramp);
    w_cmd_in = w_cmd;
    yaw_now = IMU_data.AHRS.yaw_rad_cnt;
    v_cmd = sqrtf(x_cmd * x_cmd + y_cmd * y_cmd);
    yaw_correction = 0.0f;

    /* 只在有平移指令且基本没有人工转向时启用航向保持。
     * 这解决的是“车头跑偏导致走不直”，不是对横向侧滑做位移估计。 */
    if ((v_cmd > OMNI_TRANSLATION_ENABLE_THRESHOLD) && (fabsf(w_cmd) < OMNI_YAW_CMD_DEADBAND))
    {
        if (omni_yaw_lock_valid == 0U)
        {
            /* 从不锁定切回平移时，用当前朝向作为新的锁定目标。 */
            omni_yaw_target = yaw_now;
            Omni_Wheel_Reset_Yaw_Pid();
        }

        yaw_correction = pid_cal(&omni_yaw_hold_pid, yaw_now, omni_yaw_target);
        w_cmd += yaw_correction;
        omni_yaw_lock_valid = 1U;
    }
    else
    {
        /* 明显转向或没有平移时不做走直纠偏，
         * 同时把当前朝向更新成下一次平移的锁定目标。 */
        omni_yaw_target = yaw_now;
        Omni_Wheel_Reset_Yaw_Pid();
        omni_yaw_lock_valid = 0U;
    }

    /* 三轮 120 度全向轮逆运动学：
     * x 为前向，y 为右向，w 为顺时针。
     * 计算结果为每个轮子的目标角速度，单位 rad/s。 */
    cmd1 = (y_cmd + OMNI_CHASSIS_RADIUS_M * w_cmd) / OMNI_WHEEL_RADIUS_M;
    cmd2 = (-OMNI_SQRT3_OVER_2 * x_cmd - 0.5f * y_cmd + OMNI_CHASSIS_RADIUS_M * w_cmd) / OMNI_WHEEL_RADIUS_M;
    cmd3 = (OMNI_SQRT3_OVER_2 * x_cmd - 0.5f * y_cmd + OMNI_CHASSIS_RADIUS_M * w_cmd) / OMNI_WHEEL_RADIUS_M;

    /* 方向修正独立放在矩阵之后，方便现场调反。 */
    cmd1 *= OMNI_DIR1;
    cmd2 *= OMNI_DIR2;
    cmd3 *= OMNI_DIR3;

    Omni_Wheel_Limit(&cmd1, &cmd2, &cmd3);

    omni_debug.x_cmd = x_cmd;
    omni_debug.y_cmd = y_cmd;
    omni_debug.w_cmd_in = w_cmd_in;
    omni_debug.w_cmd_out = w_cmd;
    omni_debug.yaw_now = yaw_now;
    omni_debug.yaw_target = omni_yaw_target;
    omni_debug.yaw_correction = yaw_correction;
    omni_debug.wheel_cmd1 = cmd1;
    omni_debug.wheel_cmd2 = cmd2;
    omni_debug.wheel_cmd3 = cmd3;
    omni_debug.fb_speed1 = omni_motor1.Pos_Info.Speed;
    omni_debug.fb_speed2 = omni_motor2.Pos_Info.Speed;
    omni_debug.fb_speed3 = omni_motor3.Pos_Info.Speed;
    omni_debug.run_mode1 = (uint8_t)omni_motor1.drw.run_mode.data;
    omni_debug.run_mode2 = (uint8_t)omni_motor2.drw.run_mode.data;
    omni_debug.run_mode3 = (uint8_t)omni_motor3.drw.run_mode.data;
    omni_debug.pattern1 = omni_motor1.Pos_Info.pattern;
    omni_debug.pattern2 = omni_motor2.Pos_Info.pattern;
    omni_debug.pattern3 = omni_motor3.Pos_Info.pattern;
    omni_debug.update_tick_ms = tick_ms;
    omni_debug.update_count++;

    RobStride_Motor_Speed_control(&omni_motor1, OMNI_CAN_HANDLE, cmd1, OMNI_CURRENT_LIMIT);
    RobStride_Motor_Speed_control(&omni_motor2, OMNI_CAN_HANDLE, cmd2, OMNI_CURRENT_LIMIT);
    RobStride_Motor_Speed_control(&omni_motor3, OMNI_CAN_HANDLE, cmd3, OMNI_CURRENT_LIMIT);
}

void Omni_Wheel_RxCallback(uint32_t ext_id, uint8_t *data)
{
    uint8_t target_id;

    /* 只处理有效数据指针。 */
    if (data == 0)
    {
        return;
    }

    /* RobStride 私有协议扩展帧里，目标电机 ID 在 bit[15:8]。 */
    target_id = (uint8_t)((ext_id >> 8) & 0xFFU);

    switch (target_id)
    {
    case OMNI_MOTOR1_ID:
        RobStride_Motor_Analysis(&omni_motor1, data, ext_id);
        break;
    case OMNI_MOTOR2_ID:
        RobStride_Motor_Analysis(&omni_motor2, data, ext_id);
        break;
    case OMNI_MOTOR3_ID:
        RobStride_Motor_Analysis(&omni_motor3, data, ext_id);
        break;
    default:
        break;
    }
}
