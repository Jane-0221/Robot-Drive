#include "arm.h"
#include "dm4310_drv.h"
#include "Robstride04.h"
#include "pid.h"
#include "remote_control.h"
#include "gpio.h"
#include "gom_protocol.h"
#include "usart.h"
#include "LZ_motor_driver.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "DrEmpower_can.h"
#include "stdio.h"
#include "ramp_generator.h"
#include "stp23l.h"

// ===================== 全局变量定义 =====================
/**
 * @brief 机械臂肩部电机类型（枚举值：SHOULDER_TYPE_LINGZU/DAREN/DAMIAO）
 * @note  用于切换不同品牌电机的控制逻辑（灵足/大然/达妙）
 */
ShoulderType_t g_ShoulderType;
int16_t arm_distance_final = 0;

void Arm_RefreshDistance(void)
{
    if (arm_stp23l_data.parse_ok == 1)
    {
        arm_distance_final = Arm_STP23L_GetFinalDistPerFrame();
        Arm_STP23L_ClearOkFlag();
    }
}

/**
 * @brief 大然舵机状态结构体数组（3路）
 * @note  存储大然舵机的状态信息（角度、速度、扭矩等）
 */
struct servo_state servo_state_daran[3];

/**
 * @brief 大然舵机电压/电流结构体数组（3路）
 * @note  存储大然舵机的电气参数（输入电压、相电流等）
 */
struct servo_volcur servo_volcur_daran[3];

/**
 * @brief 达妙电机状态结构体数组（6路）
 * @note  存储DM4310电机的位置、速度、电流等状态
 */
extern Motor_DM_Status DM_Status[6];

/**
 * @brief 灵足电机对象（3路）
 * @note  Robstride04协议驱动，对应灵足1/2/3号电机
 */
RobStride_Motor_t motor1; // 灵足1号电机
RobStride_Motor_t motor2; // 灵足2号电机
RobStride_Motor_t motor3; // 灵足3号电机

/**
 * @brief 灵足电机数据结构体数组（3路）
 * @note  存储灵足电机的目标、当前角度、速度等控制参数
 */
ArmMotorData_t Linzu_motor_data[3];

/**
 * @brief 大然电机数据结构体数组（3路）
 * @note  存储大然电机的目标、当前角度、速度等控制参数
 */
ArmMotorData_t Daran_motor_data[3];

/**
 * @brief 达妙电机数据结构体数组（3路）
 * @note  存储达妙电机的目标、当前角度、速度等控制参数
 */
ArmMotorData_t Damiao_motor_data[3];

/**
 * @brief 数据回复使能标志
 * @note  浮点型：0.0f=禁用回复，非0.0f=启用回复（未在本代码中实际使用）
 */
float reply_enable = 0.0f;

#define ARM_REENABLE_CHECK_PERIOD_MS 100U
#define ARM_DAMIAO_ENABLE_STATE 2U
#define ARM_PI_RAD 3.141592654f
#define ARM_TWO_PI_RAD 6.283185307f
#define ARM_TARGET_ANGLE_LIMIT_RAD 2.617993878f
#define ARM_TARGET_CURRENT_MAX_DIFF_RAD ARM_PI_RAD
#define ARM_TARGET_ANGLE_RAMP_INTERVAL_MS 10U
#define ARM_TARGET_ANGLE_RAMP_MAX_ELAPSED_MS 50U
#define ARM_TARGET_ANGLE_RAMP_SLOW_RAD_PER_S 1.0f
#define ARM_TARGET_ANGLE_RAMP_FAST_RAD_PER_S 2.0f
#define ARM_PC_TARGET_STABLE_MS 25U
#define ARM_PC_TARGET_STREAM_FORCE_MS 60U
#define ARM_PC_TARGET_NOISE_RAD 0.003f
#define ARM_PC_TARGET_MAX_DT_MS 50U
#define ARM_PC_TARGET_SLOW_MAX_VEL_RAD_PER_S 0.8f
#define ARM_PC_TARGET_SLOW_MAX_ACC_RAD_PER_S2 1.2f
#define ARM_PC_TARGET_FAST_MAX_VEL_RAD_PER_S 1.5f
#define ARM_PC_TARGET_FAST_MAX_ACC_RAD_PER_S2 3.0f
#define ARM_DISABLED_FEEDBACK_PERIOD_MS 50U

volatile uint8_t arm_motor_disabled_mask_debug = 0U;
volatile uint32_t arm_feedback_count_debug[ARM_LOGICAL_MOTOR_COUNT] = {0U};
volatile uint32_t arm_feedback_last_tick_debug[ARM_LOGICAL_MOTOR_COUNT] = {0U};
volatile float arm_pc_target_debug[ARM_LOGICAL_MOTOR_COUNT] = {0.0f};
volatile float arm_planned_target_debug[ARM_LOGICAL_MOTOR_COUNT] = {0.0f};
volatile float arm_planned_velocity_debug[ARM_LOGICAL_MOTOR_COUNT] = {0.0f};
volatile uint32_t arm_planner_dt_ms_debug = 0U;

static RampGenerator arm_target_angle_ramps[ARM_LOGICAL_MOTOR_COUNT];
static uint8_t arm_target_angle_ramps_initialized = 0U;

typedef struct
{
    float latest_target;
    float pending_target;
    float accepted_target;
    float planned_angle;
    float planned_velocity;
    uint32_t pending_since_ms;
    uint32_t stream_start_ms;
    uint32_t last_input_ms;
    uint32_t last_update_ms;
    uint8_t initialized;
} ArmPcTargetPlanner_t;

static ArmPcTargetPlanner_t arm_pc_target_planners[ARM_LOGICAL_MOTOR_COUNT];
static uint8_t arm_pc_target_planner_enabled = 0U;

static ArmMotorData_t *Arm_GetMotorDataByIndex(uint8_t logical_motor);

static uint8_t Arm_GetMotorMask(uint8_t logical_motor)
{
    if (logical_motor >= ARM_LOGICAL_MOTOR_COUNT)
    {
        return 0U;
    }

    return (uint8_t)(1U << logical_motor);
}

static uint8_t Arm_MotorTxDisabledByIndex(uint8_t logical_motor)
{
    uint8_t motor_mask = Arm_GetMotorMask(logical_motor);

    if (motor_mask == 0U)
    {
        return 1U;
    }

    return ((arm_motor_disabled_mask_debug & motor_mask) != 0U) ? 1U : 0U;
}

static void Arm_SetMotorTxDisabledByIndex(uint8_t logical_motor, uint8_t disabled)
{
    uint8_t motor_mask = Arm_GetMotorMask(logical_motor);

    if (motor_mask == 0U)
    {
        return;
    }

    if (disabled != 0U)
    {
        arm_motor_disabled_mask_debug |= motor_mask;
    }
    else
    {
        arm_motor_disabled_mask_debug &= (uint8_t)(~motor_mask);
    }
}

float Arm_WrapAngleToPi(float angle)
{
    while (angle > ARM_PI_RAD)
    {
        angle -= ARM_TWO_PI_RAD;
    }

    while (angle < -ARM_PI_RAD)
    {
        angle += ARM_TWO_PI_RAD;
    }

    return angle;
}

static float Arm_GetEquivalentAngleNearCurrent(float target_angle, float current_angle)
{
    return current_angle + Arm_WrapAngleToPi(target_angle - current_angle);
}

static float Arm_ClampTargetAngle(float angle)
{
    if (angle > ARM_TARGET_ANGLE_LIMIT_RAD)
    {
        return ARM_TARGET_ANGLE_LIMIT_RAD;
    }

    if (angle < -ARM_TARGET_ANGLE_LIMIT_RAD)
    {
        return -ARM_TARGET_ANGLE_LIMIT_RAD;
    }

    return angle;
}

static float Arm_GetTargetAngleRampRate(uint8_t logical_motor)
{
    static const float ramp_rates[ARM_LOGICAL_MOTOR_COUNT] = {
        ARM_TARGET_ANGLE_RAMP_SLOW_RAD_PER_S,
        ARM_TARGET_ANGLE_RAMP_SLOW_RAD_PER_S,
        ARM_TARGET_ANGLE_RAMP_FAST_RAD_PER_S,
        ARM_TARGET_ANGLE_RAMP_FAST_RAD_PER_S,
        ARM_TARGET_ANGLE_RAMP_FAST_RAD_PER_S,
        ARM_TARGET_ANGLE_RAMP_FAST_RAD_PER_S,
    };

    if (logical_motor >= ARM_LOGICAL_MOTOR_COUNT)
    {
        return ARM_TARGET_ANGLE_RAMP_FAST_RAD_PER_S;
    }

    return ramp_rates[logical_motor];
}

static void Arm_SetTargetAngleRampValue(uint8_t logical_motor, float target_angle)
{
    RampGenerator *ramp;

    if (logical_motor >= ARM_LOGICAL_MOTOR_COUNT)
    {
        return;
    }

    ramp = &arm_target_angle_ramps[logical_motor];
    target_angle = Arm_ClampTargetAngle(target_angle);
    ramp->current_value = target_angle;
    ramp->target_value = target_angle;
    ramp->last_update_time = HAL_GetTick();
}

static void Arm_TargetAngleRampsInit(void)
{
    uint8_t logical_motor;

    for (logical_motor = 0U; logical_motor < ARM_LOGICAL_MOTOR_COUNT; logical_motor++)
    {
        float ramp_rate = Arm_GetTargetAngleRampRate(logical_motor);

        RampGenerator_Init(&arm_target_angle_ramps[logical_motor],
                           ARM_TARGET_ANGLE_RAMP_INTERVAL_MS,
                           ramp_rate,
                           ramp_rate,
                           ARM_TARGET_ANGLE_LIMIT_RAD);
        Arm_SetTargetAngleRampValue(logical_motor, 0.0f);
    }

    arm_target_angle_ramps_initialized = 1U;
}

static void Arm_TargetAngleRampsInitIfNeeded(void)
{
    if (arm_target_angle_ramps_initialized == 0U)
    {
        Arm_TargetAngleRampsInit();
    }
}

static void Arm_SyncTargetAngleRamp(uint8_t logical_motor, ArmMotorData_t *motor_data);

static float Arm_LimitTargetAngle(ArmMotorData_t *motor_data)
{
    motor_data->target_angle = Arm_ClampTargetAngle(motor_data->target_angle);
    return motor_data->target_angle;
}

static uint8_t Arm_IsAngleDiffSafe(float target_angle, float current_angle)
{
    float angle_diff;

    angle_diff = target_angle - current_angle;

    if (angle_diff < 0.0f)
    {
        angle_diff = -angle_diff;
    }

    return (angle_diff <= ARM_TARGET_CURRENT_MAX_DIFF_RAD) ? 1U : 0U;
}

static uint8_t Arm_GetSafeTargetAngle(ArmMotorData_t *motor_data, float *target_angle)
{
    *target_angle = Arm_LimitTargetAngle(motor_data);
    return Arm_IsAngleDiffSafe(*target_angle, motor_data->current_angle);
}

static float Arm_AbsFloat(float value)
{
    return (value < 0.0f) ? -value : value;
}

static float Arm_GetCurrentAngleByIndex(uint8_t logical_motor)
{
    ArmMotorData_t *motor_data = Arm_GetMotorDataByIndex(logical_motor);

    if (motor_data == NULL)
    {
        return 0.0f;
    }

    return Arm_ClampTargetAngle(motor_data->current_angle);
}

static float Arm_GetPcPlannerMaxVel(uint8_t logical_motor)
{
    return (logical_motor < 3U) ?
           ARM_PC_TARGET_SLOW_MAX_VEL_RAD_PER_S :
           ARM_PC_TARGET_FAST_MAX_VEL_RAD_PER_S;
}

static float Arm_GetPcPlannerMaxAcc(uint8_t logical_motor)
{
    return (logical_motor < 3U) ?
           ARM_PC_TARGET_SLOW_MAX_ACC_RAD_PER_S2 :
           ARM_PC_TARGET_FAST_MAX_ACC_RAD_PER_S2;
}

static void Arm_UpdatePcPlannerDebug(uint8_t logical_motor)
{
    ArmPcTargetPlanner_t *planner;

    if (logical_motor >= ARM_LOGICAL_MOTOR_COUNT)
    {
        return;
    }

    planner = &arm_pc_target_planners[logical_motor];
    arm_pc_target_debug[logical_motor] = planner->latest_target;
    arm_planned_target_debug[logical_motor] = planner->planned_angle;
    arm_planned_velocity_debug[logical_motor] = planner->planned_velocity;
}

static void Arm_SyncPcPlannerByIndex(uint8_t logical_motor, float target_angle, uint32_t now_ms)
{
    ArmPcTargetPlanner_t *planner;

    if (logical_motor >= ARM_LOGICAL_MOTOR_COUNT)
    {
        return;
    }

    target_angle = Arm_ClampTargetAngle(target_angle);
    planner = &arm_pc_target_planners[logical_motor];
    planner->latest_target = target_angle;
    planner->pending_target = target_angle;
    planner->accepted_target = target_angle;
    planner->planned_angle = target_angle;
    planner->planned_velocity = 0.0f;
    planner->pending_since_ms = now_ms;
    planner->stream_start_ms = now_ms;
    planner->last_input_ms = now_ms;
    planner->last_update_ms = now_ms;
    planner->initialized = 1U;
    Arm_UpdatePcPlannerDebug(logical_motor);
}

static void Arm_InitPcPlannerByIndex(uint8_t logical_motor, uint32_t now_ms)
{
    float current_angle;

    if (logical_motor >= ARM_LOGICAL_MOTOR_COUNT)
    {
        return;
    }

    current_angle = Arm_GetCurrentAngleByIndex(logical_motor);
    Arm_SyncPcPlannerByIndex(logical_motor, current_angle, now_ms);
}

static void Arm_AcceptPendingPcTargetIfReady(uint8_t logical_motor, uint32_t now_ms)
{
    ArmPcTargetPlanner_t *planner;
    uint8_t accept_target = 0U;

    if (logical_motor >= ARM_LOGICAL_MOTOR_COUNT)
    {
        return;
    }

    planner = &arm_pc_target_planners[logical_motor];
    if (planner->initialized == 0U)
    {
        Arm_InitPcPlannerByIndex(logical_motor, now_ms);
    }

    if (Arm_AbsFloat(planner->pending_target - planner->accepted_target) <= ARM_PC_TARGET_NOISE_RAD)
    {
        return;
    }

    if ((now_ms - planner->pending_since_ms) >= ARM_PC_TARGET_STABLE_MS)
    {
        accept_target = 1U;
    }
    else if ((now_ms - planner->stream_start_ms) >= ARM_PC_TARGET_STREAM_FORCE_MS)
    {
        accept_target = 1U;
    }

    if (accept_target != 0U)
    {
        planner->accepted_target = planner->pending_target;
        planner->pending_since_ms = now_ms;
        planner->stream_start_ms = now_ms;
    }
}

void Arm_SetPcTargetAngles(const float target_angles[ARM_LOGICAL_MOTOR_COUNT], uint32_t now_ms)
{
    uint8_t logical_motor;

    if (target_angles == NULL)
    {
        return;
    }

    arm_pc_target_planner_enabled = 1U;

    for (logical_motor = 0U; logical_motor < ARM_LOGICAL_MOTOR_COUNT; logical_motor++)
    {
        ArmPcTargetPlanner_t *planner = &arm_pc_target_planners[logical_motor];
        ArmMotorData_t *motor_data = Arm_GetMotorDataByIndex(logical_motor);
        float target_angle = Arm_ClampTargetAngle(target_angles[logical_motor]);

        if (planner->initialized == 0U)
        {
            Arm_InitPcPlannerByIndex(logical_motor, now_ms);
        }

        if (motor_data != NULL)
        {
            motor_data->target_angle = target_angle;
        }

        if (Arm_AbsFloat(target_angle - planner->pending_target) > ARM_PC_TARGET_NOISE_RAD)
        {
            if ((now_ms - planner->last_input_ms) > ARM_PC_TARGET_STABLE_MS)
            {
                planner->stream_start_ms = now_ms;
            }

            planner->pending_target = target_angle;
            planner->pending_since_ms = now_ms;
        }

        planner->latest_target = target_angle;
        planner->last_input_ms = now_ms;
        Arm_AcceptPendingPcTargetIfReady(logical_motor, now_ms);
        Arm_UpdatePcPlannerDebug(logical_motor);
    }
}

static void Arm_UpdatePcTargetPlanner(uint32_t now_ms)
{
    uint8_t logical_motor;

    if (arm_pc_target_planner_enabled == 0U)
    {
        return;
    }

    for (logical_motor = 0U; logical_motor < ARM_LOGICAL_MOTOR_COUNT; logical_motor++)
    {
        ArmPcTargetPlanner_t *planner = &arm_pc_target_planners[logical_motor];
        uint32_t elapsed_ms;
        float dt_s;
        float error;
        float direction;
        float max_vel;
        float max_acc;
        float stop_distance;
        float old_error;
        float new_angle;
        float new_error;

        if (planner->initialized == 0U)
        {
            Arm_InitPcPlannerByIndex(logical_motor, now_ms);
        }

        Arm_AcceptPendingPcTargetIfReady(logical_motor, now_ms);

        elapsed_ms = now_ms - planner->last_update_ms;
        if (elapsed_ms == 0U)
        {
            Arm_UpdatePcPlannerDebug(logical_motor);
            continue;
        }

        planner->last_update_ms = now_ms;
        if (elapsed_ms > ARM_PC_TARGET_MAX_DT_MS)
        {
            elapsed_ms = ARM_PC_TARGET_MAX_DT_MS;
        }
        arm_planner_dt_ms_debug = elapsed_ms;

        dt_s = (float)elapsed_ms * 0.001f;
        error = planner->accepted_target - planner->planned_angle;

        if (Arm_AbsFloat(error) <= ARM_PC_TARGET_NOISE_RAD)
        {
            planner->planned_angle = planner->accepted_target;
            planner->planned_velocity = 0.0f;
            Arm_UpdatePcPlannerDebug(logical_motor);
            continue;
        }

        direction = (error >= 0.0f) ? 1.0f : -1.0f;
        max_vel = Arm_GetPcPlannerMaxVel(logical_motor);
        max_acc = Arm_GetPcPlannerMaxAcc(logical_motor);
        stop_distance = (planner->planned_velocity * planner->planned_velocity) / (2.0f * max_acc);

        if ((planner->planned_velocity * direction) < 0.0f)
        {
            planner->planned_velocity += direction * max_acc * dt_s;
        }
        else if (Arm_AbsFloat(error) <= stop_distance)
        {
            planner->planned_velocity -= direction * max_acc * dt_s;
        }
        else
        {
            planner->planned_velocity += direction * max_acc * dt_s;
        }

        if (planner->planned_velocity > max_vel)
        {
            planner->planned_velocity = max_vel;
        }
        else if (planner->planned_velocity < -max_vel)
        {
            planner->planned_velocity = -max_vel;
        }

        old_error = planner->accepted_target - planner->planned_angle;
        new_angle = planner->planned_angle + planner->planned_velocity * dt_s;
        new_error = planner->accepted_target - new_angle;

        if (((old_error > 0.0f) && (new_error < 0.0f)) ||
            ((old_error < 0.0f) && (new_error > 0.0f)) ||
            (Arm_AbsFloat(new_error) <= ARM_PC_TARGET_NOISE_RAD))
        {
            planner->planned_angle = planner->accepted_target;
            planner->planned_velocity = 0.0f;
        }
        else
        {
            planner->planned_angle = Arm_ClampTargetAngle(new_angle);
        }

        Arm_UpdatePcPlannerDebug(logical_motor);
    }
}

static void Arm_SyncTargetAngleRamp(uint8_t logical_motor, ArmMotorData_t *motor_data)
{
    if (motor_data == NULL)
    {
        return;
    }

    Arm_TargetAngleRampsInitIfNeeded();
    Arm_SetTargetAngleRampValue(logical_motor, Arm_LimitTargetAngle(motor_data));
    Arm_SyncPcPlannerByIndex(logical_motor, motor_data->target_angle, HAL_GetTick());
}

static float Arm_GetSmoothedTargetAngle(uint8_t logical_motor, ArmMotorData_t *motor_data)
{
    RampGenerator *ramp;
    uint32_t now_tick;
    float target_angle;

    target_angle = Arm_LimitTargetAngle(motor_data);

    if (logical_motor >= ARM_LOGICAL_MOTOR_COUNT)
    {
        return target_angle;
    }

    Arm_TargetAngleRampsInitIfNeeded();

    if (arm_pc_target_planner_enabled != 0U)
    {
        Arm_UpdatePcTargetPlanner(HAL_GetTick());
        return arm_pc_target_planners[logical_motor].planned_angle;
    }

    ramp = &arm_target_angle_ramps[logical_motor];
    now_tick = HAL_GetTick();

    if ((ramp->last_update_time != 0U) &&
        ((now_tick - ramp->last_update_time) > ARM_TARGET_ANGLE_RAMP_MAX_ELAPSED_MS))
    {
        ramp->last_update_time = now_tick;
    }

    RampGenerator_SetTarget(ramp, target_angle);
    RampGenerator_Update(ramp, now_tick);

    return RampGenerator_GetCurrent(ramp);
}

static uint8_t Arm_SendLinzuTarget(RobStride_Motor_t *motor, ArmMotorData_t *motor_data)
{
    float target_angle;
    float motor_angle;

    target_angle = Arm_LimitTargetAngle(motor_data);
    motor_angle = motor->Pos_Info.Angle;
    target_angle = Arm_GetEquivalentAngleNearCurrent(target_angle, motor_angle);

    if (Arm_IsAngleDiffSafe(target_angle, motor_angle) == 0U)
    {
        return 0U;
    }

    RobStride_Motor_CSP_control(motor, CAN_HANDLE_2, target_angle, motor_data->target_velocity);
    return 1U;
}

static uint8_t Arm_SendLinzuSmoothedTarget(uint8_t logical_motor, RobStride_Motor_t *motor, ArmMotorData_t *motor_data)
{
    float target_angle;
    float motor_angle;

    target_angle = Arm_GetSmoothedTargetAngle(logical_motor, motor_data);
    motor_angle = motor->Pos_Info.Angle;
    target_angle = Arm_GetEquivalentAngleNearCurrent(target_angle, motor_angle);

    if (Arm_IsAngleDiffSafe(target_angle, motor_angle) == 0U)
    {
        return 0U;
    }

    RobStride_Motor_CSP_control(motor, CAN_HANDLE_2, target_angle, motor_data->target_velocity);
    return 1U;
}

static uint8_t Arm_SendDaranTarget(uint8_t motor_id, ArmMotorData_t *motor_data)
{
    float target_angle;

    if (Arm_GetSafeTargetAngle(motor_data, &target_angle) == 0U)
    {
        return 0U;
    }

    set_angle(CAN_HANDLE_2, motor_id, target_angle, motor_data->target_velocity, 10.0f, 1);
    return 1U;
}

static uint8_t Arm_SendDaranSmoothedTarget(uint8_t logical_motor, uint8_t motor_id, ArmMotorData_t *motor_data)
{
    float target_angle;

    target_angle = Arm_GetSmoothedTargetAngle(logical_motor, motor_data);

    if (Arm_IsAngleDiffSafe(target_angle, motor_data->current_angle) == 0U)
    {
        return 0U;
    }

    set_angle(CAN_HANDLE_2, motor_id, target_angle, motor_data->target_velocity, 10.0f, 1);
    return 1U;
}

static uint8_t Arm_SendDamiaoTarget(uint16_t motor_id, ArmMotorData_t *motor_data)
{
    float target_angle;

    if (Arm_GetSafeTargetAngle(motor_data, &target_angle) == 0U)
    {
        return 0U;
    }

    pos_speed_ctrl(CAN_HANDLE_2, motor_id, target_angle, motor_data->target_velocity);
    return 1U;
}

static uint8_t Arm_SendDamiaoSmoothedTarget(uint8_t logical_motor, uint16_t motor_id, ArmMotorData_t *motor_data)
{
    float target_angle;

    target_angle = Arm_GetSmoothedTargetAngle(logical_motor, motor_data);

    if (Arm_IsAngleDiffSafe(target_angle, motor_data->current_angle) == 0U)
    {
        return 0U;
    }

    pos_speed_ctrl(CAN_HANDLE_2, motor_id, target_angle, motor_data->target_velocity);
    return 1U;
}

// ===================== 函数定义 =====================
/**
 * @brief 机械臂初始化函数
 * @retval 无
 * @note   1. 核心功能：选择肩部电机类型，初始化灵足/大然/达妙电机，配置CAN2通信；
 *         2. 灵足电机：初始化RobStride协议，设置CSP位置模式，使能电机，开启主动上报；
 *         3. 大然电机：清除错误，设置模式2（位置模式），配置参数22001（比例系数）；
 *         4. 达妙电机：初始化位置模式，暂未配置具体参数；
 *         5. 所有电机均挂载在CAN2总线上。
 */
void Arm_Init()
{
    // 选择默认肩部电机类型（注释为灵足，实际启用大然）
    g_ShoulderType = SHOULDER_TYPE_LINGZU;
    // g_ShoulderType = SHOULDER_TYPE_DARAN;

    /* 灵足电机初始化（使用CAN2总线） */
    // 1号灵足电机初始化
    RobStride_Motor_Init(&motor1, MOTOR_LINGZU_1_ID, false);                             // 初始化电机对象（ID为灵足1号）
    Get_RobStride_Motor_parameter(&motor1, CAN_HANDLE_2, 0X7005);                        // 读取电机参数，0X7005为参数地址
    HAL_Delay(10);                                                                       // 延时确保通信稳定
    Set_RobStride_Motor_parameter(&motor1, CAN_HANDLE_2, 0X7005, CSP_control_mode, 'j'); // 设置CSP位置控制模式
    // Enable_Motor(&motor1, (hcan_t *)CAN_HANDLE_2);                                       // 使能电机
    Set_RobStride_Motor_parameter(&motor1, CAN_HANDLE_2, 0X7017, 1.0f, 'p'); // 设置参数0X7017，比例1.0
    HAL_Delay(10);
    // 开启主动上报（0x00=关闭，0x01=开启）
    RobStride_Motor_ProactiveEscalationSet(&motor1, CAN_HANDLE_2, 0x01);

    // 2号灵足电机初始化（逻辑2号）
    RobStride_Motor_Init(&motor2, MOTOR_LINGZU_2_ID, false);
    Get_RobStride_Motor_parameter(&motor2, CAN_HANDLE_2, 0X7005);
    HAL_Delay(10);
    Set_RobStride_Motor_parameter(&motor2, CAN_HANDLE_2, 0X7005, CSP_control_mode, 'j');
    // Enable_Motor(&motor2, (hcan_t *)CAN_HANDLE_2);
    Set_RobStride_Motor_parameter(&motor2, CAN_HANDLE_2, 0X7017, 1.0f, 'p');
    HAL_Delay(10);
    RobStride_Motor_ProactiveEscalationSet(&motor2, CAN_HANDLE_2, 0x01);

    // 3号灵足电机初始化（逻辑3号）
    RobStride_Motor_Init(&motor3, MOTOR_LINGZU_3_ID, false);
    Get_RobStride_Motor_parameter(&motor3, CAN_HANDLE_2, 0X7005);
    HAL_Delay(10);
    Set_RobStride_Motor_parameter(&motor3, CAN_HANDLE_2, 0X7005, CSP_control_mode, 'j');
    // Enable_Motor(&motor3, (hcan_t *)CAN_HANDLE_2);
    Set_RobStride_Motor_parameter(&motor3, CAN_HANDLE_2, 0X7017, 1.0f, 'p');
    HAL_Delay(10);
    RobStride_Motor_ProactiveEscalationSet(&motor3, CAN_HANDLE_2, 0x01);

    /* 大然电机初始化（使用CAN2总线） */
    clear_error(CAN_HANDLE_2, MOTOR_DARAN_1_ID);                    // 清除1号大然电机错误
    set_mode(CAN_HANDLE_2, MOTOR_DARAN_1_ID, 2);                    // 设置模式2（位置模式）
    write_property(CAN_HANDLE_2, MOTOR_DARAN_1_ID, 22001, 3, 1.0f); // 写入参数22001（比例系数1.0）

    clear_error(CAN_HANDLE_2, MOTOR_DARAN_2_ID);                    // 清除2号大然电机错误
    set_mode(CAN_HANDLE_2, MOTOR_DARAN_2_ID, 2);                    // 设置位置模式
    write_property(CAN_HANDLE_2, MOTOR_DARAN_2_ID, 22001, 3, 1.0f); // 配置比例系数

    clear_error(CAN_HANDLE_2, MOTOR_DARAN_3_ID);                    // 清除3号大然电机错误
    set_mode(CAN_HANDLE_2, MOTOR_DARAN_3_ID, 2);                    // 设置位置模式
    write_property(CAN_HANDLE_2, MOTOR_DARAN_3_ID, 22001, 3, 1.0f); // 配置比例系数

    /* 达妙电机初始化（使用CAN2总线） */
    arm_motor_init(&arm_motor[Motor4], MOTOR_DAMIAO_4_ID, POS_MODE); // 4号达妙电机初始化（位置模式）
    arm_motor_init(&arm_motor[Motor5], MOTOR_DAMIAO_5_ID, POS_MODE); // 5号达妙电机初始化（位置模式）
    arm_motor_init(&arm_motor[Motor6], MOTOR_DAMIAO_6_ID, POS_MODE); // 6号达妙电机初始化（位置模式）

    enable_motor_mode(CAN_HANDLE_2, MOTOR_DAMIAO_4_ID, POS_MODE); // 使能4号达妙电机位置模式
    enable_motor_mode(CAN_HANDLE_2, MOTOR_DAMIAO_5_ID, POS_MODE); // 使能5号达妙电机位置模式
    enable_motor_mode(CAN_HANDLE_2, MOTOR_DAMIAO_6_ID, POS_MODE); // 使能6号达妙电机位置模式
    set_DM_mode(Motor4, POS_MODE);
    set_DM_mode(Motor5, POS_MODE);
    set_DM_mode(Motor6, POS_MODE);

    Damiao_motor_data[0].target_angle = 0.0f;
    Damiao_motor_data[1].target_angle = 0.0f;
    Damiao_motor_data[2].target_angle = 0.0f;
    Damiao_motor_data[0].target_velocity = 1.0f;
    Damiao_motor_data[1].target_velocity = 1.0f;
    Damiao_motor_data[2].target_velocity = 1.0f;
    // 初始化灵足电机目标参数（角度10°，速度1.0r/min）
    // Set_ZeroPos(&motor1, CAN_HANDLE_2);
    // Set_ZeroPos(&motor2, CAN_HANDLE_2);
    // Set_ZeroPos(&motor3, CAN_HANDLE_2);

    Linzu_motor_data[0].target_angle = 0.0f;
    Linzu_motor_data[1].target_angle = 0.0f;
    Linzu_motor_data[2].target_angle = 0.0f;
    Linzu_motor_data[0].target_velocity = 1.0f;
    Linzu_motor_data[1].target_velocity = 1.0f;
    Linzu_motor_data[2].target_velocity = 1.0f;

    // 初始化大然电机目标参数（角度10°，速度分别为90/20/20r/min）
    Daran_motor_data[0].target_angle = 0.0f;
    Daran_motor_data[1].target_angle = 0.0f;
    Daran_motor_data[2].target_angle = 0.0f;
    Daran_motor_data[0].target_velocity = 20.0f;
    Daran_motor_data[1].target_velocity = 20.0f;
    Daran_motor_data[2].target_velocity = 20.0f;
    Arm_TargetAngleRampsInit();
    for (uint8_t logical_motor = 0U; logical_motor < ARM_LOGICAL_MOTOR_COUNT; logical_motor++)
    {
        Arm_SyncPcPlannerByIndex(logical_motor, 0.0f, HAL_GetTick());
    }
    // 初始化灵足电机位置为0

    // 初始化电机失能
    // Disenable_Motor(&motor1, CAN_HANDLE_2, 0U);
    // osDelay(1);
    // Disenable_Motor(&motor2, CAN_HANDLE_2, 0U);
    // osDelay(1);
    // Disenable_Motor(&motor3, CAN_HANDLE_2, 0U);
    // osDelay(1);
    // disable_motor_mode(CAN_HANDLE_2, MOTOR_DAMIAO_4_ID, POS_MODE);
    // osDelay(1);
    // disable_motor_mode(CAN_HANDLE_2, MOTOR_DAMIAO_5_ID, POS_MODE);
    // osDelay(1);
    // disable_motor_mode(CAN_HANDLE_2, MOTOR_DAMIAO_6_ID, POS_MODE);
}

/**
 * @brief 控制1号灵足电机（CSP位置模式）
 * @retval 无
 * @note   根据Linzu_motor_data[0]的目标角度、速度，通过CAN2发送控制指令
 */
void Arm_Linzu_motor1()
{
    (void)Arm_SendLinzuSmoothedTarget(0U, &motor1, &Linzu_motor_data[0]);
}

/**
 * @brief 控制2号灵足电机（CSP位置模式）
 * @retval 无
 * @note   根据Linzu_motor_data[1]的目标角度、速度，通过CAN2发送控制指令
 */
void Arm_Linzu_motor2()
{
    (void)Arm_SendLinzuSmoothedTarget(1U, &motor2, &Linzu_motor_data[1]);
}

/**
 * @brief 控制3号灵足电机（CSP位置模式）
 * @retval 无
 * @note   根据Linzu_motor_data[2]的目标角度、速度，通过CAN2发送控制指令
 */
void Arm_Linzu_motor3()
{
    (void)Arm_SendLinzuSmoothedTarget(2U, &motor3, &Linzu_motor_data[2]);
}

/**
 * @brief 控制1号大然电机（位置模式）
 * @retval 无
 * @note   1. 延时1ms确保通信稳定；
 *         2. 设置目标角度、速度、加速度（10.0f）、立即生效（1）；
 *         3. 指令通过CAN2发送至1号大然电机。
 */
void Arm_Daran_motor1()
{
    (void)Arm_SendDaranSmoothedTarget(0U, MOTOR_DARAN_1_ID, &Daran_motor_data[0]);
}

/**
 * @brief 控制2号大然电机（位置模式）
 * @retval 无
 * @note   逻辑2号大然电机，使用Daran_motor_data[1]的目标参数
 */
void Arm_Daran_motor2()
{
    (void)Arm_SendDaranSmoothedTarget(1U, MOTOR_DARAN_2_ID, &Daran_motor_data[1]);
}

/**
 * @brief 控制3号大然电机（位置模式）
 * @retval 无
 * @note   逻辑3号大然电机，使用Daran_motor_data[2]的目标参数
 */
void Arm_Daran_motor3()
{
    (void)Arm_SendDaranSmoothedTarget(2U, MOTOR_DARAN_3_ID, &Daran_motor_data[2]);
}

/**
 * @brief 控制4号达妙电机（位置模式）
 * @retval 无
 * @note   1. 设置位置模式；
 *         2. 配置目标位置/速度；
 *         3. 设置位置速度控制参数（5=位置参数，10=速度参数）；
 *         4. 固定目标位置为20（可根据需求修改）。
 */
void Arm_Damiao_motor4()
{
    (void)Arm_SendDamiaoSmoothedTarget(3U, MOTOR_DAMIAO_4_ID, &Damiao_motor_data[0]);
}

/**
 * @brief 控制5号达妙电机（位置模式）
 * @retval 无
 * @note   逻辑5号达妙电机，位置参数10，速度参数1
 */
void Arm_Damiao_motor5()
{

    (void)Arm_SendDamiaoSmoothedTarget(4U, MOTOR_DAMIAO_5_ID, &Damiao_motor_data[1]);
}

/**
 * @brief 控制6号达妙电机（位置模式）
 * @retval 无
 * @note   1. 设置位置模式；
 *         2. 配置目标位置/速度；
 *         3. 固定目标位置为10，使用电机6的位置设定值作为控制参数；
 *         4. 速度参数固定为1。
 */
void Arm_Damiao_motor6()
{
    (void)Arm_SendDamiaoSmoothedTarget(5U, MOTOR_DAMIAO_6_ID, &Damiao_motor_data[2]);
}

/**
 * @brief 更新灵足电机当前状态数据
 * @retval 无
 * @note   从motor1/2/3的RobStride协议缓存中，读取当前角度、速度，更新到Linzu_motor_data
 */
void Arm_Linzu_Data_update()
{
    Linzu_motor_data[0].current_angle = motor1.Pos_Info.Angle;
    Linzu_motor_data[1].current_angle = motor2.Pos_Info.Angle;
    Linzu_motor_data[2].current_angle = motor3.Pos_Info.Angle;
    Linzu_motor_data[0].current_velocity = motor1.Pos_Info.Speed;
    Linzu_motor_data[1].current_velocity = motor2.Pos_Info.Speed;
    Linzu_motor_data[2].current_velocity = motor3.Pos_Info.Speed;
}

/**
 * @brief 更新大然电机当前状态数据
 * @retval 无
 * @note   1. 从daran_motor_state缓存中读取角度、速度（扭矩注释未使用）；
 *         2. 更新到Daran_motor_data的current_angle/current_velocity成员；
 *         3. 调试打印代码已注释，如需查看可取消注释。
 */
void Arm_Daran_Data_update()
{

    Daran_motor_data[0].current_angle = daran_motor_state[0].angle;
    Daran_motor_data[1].current_angle = daran_motor_state[1].angle;
    Daran_motor_data[2].current_angle = daran_motor_state[2].angle;
    Daran_motor_data[0].current_velocity = daran_motor_state[0].speed;
    Daran_motor_data[1].current_velocity = daran_motor_state[1].speed;
    Daran_motor_data[2].current_velocity = daran_motor_state[2].speed;
}
void Arm_Damiao_Data_update()
{
    Damiao_motor_data[0].current_angle = arm_motor[3].para.pos;
    Damiao_motor_data[1].current_angle = arm_motor[4].para.pos;
    Damiao_motor_data[2].current_angle = arm_motor[5].para.pos;
    Damiao_motor_data[0].current_velocity = arm_motor[3].para.vel;
    Damiao_motor_data[1].current_velocity = arm_motor[4].para.vel;
    Damiao_motor_data[2].current_velocity = arm_motor[5].para.vel;
}

/**
 * @brief 批量更新所有电机状态数据
 * @retval 无
 * @note   1. 先更新灵足电机数据；
 *         2. 延时1ms后更新大然电机数据；
 *         3. 达妙电机数据更新逻辑暂未实现。
 */
void Arm_All_Data_update()
{
    Arm_Linzu_Data_update();
    osDelay(1);
    Arm_Daran_Data_update();
    osDelay(1);
    Arm_Damiao_Data_update();
}

/**
 * @brief 机械臂电机控制指令发送函数
 * @retval 无
 * @note   1. 根据g_ShoulderType选择发送灵足/大然电机控制指令；
 *         2. 灵足电机：依次发送1/2/3号指令，间隔1ms FreeRTOS延时；
 *         3. 大然电机：依次发送1/2/3号指令，间隔1ms FreeRTOS延时；
 *         4. 达妙电机控制指令已注释，如需启用可取消注释。
 */
void Arm_all_tx()
{
    Arm_UpdatePcTargetPlanner(HAL_GetTick());

    if (g_ShoulderType == SHOULDER_TYPE_LINGZU)
    {
        if (Arm_MotorTxDisabledByIndex(0U) == 0U)
        {
            Arm_Linzu_motor1();
        }
        if (Arm_MotorTxDisabledByIndex(1U) == 0U)
        {
            Arm_Linzu_motor2();
        }
        if (Arm_MotorTxDisabledByIndex(2U) == 0U)
        {
            Arm_Linzu_motor3();
        }
    }
    else if (g_ShoulderType == SHOULDER_TYPE_DARAN)
    {
        if (Arm_MotorTxDisabledByIndex(0U) == 0U)
        {
            Arm_Daran_motor1();
        }
        if (Arm_MotorTxDisabledByIndex(1U) == 0U)
        {
            Arm_Daran_motor2();
        }
        if (Arm_MotorTxDisabledByIndex(2U) == 0U)
        {
            Arm_Daran_motor3();
        }
    }

    if (Arm_MotorTxDisabledByIndex(3U) == 0U)
    {
        Arm_Damiao_motor4();
    }
    if (Arm_MotorTxDisabledByIndex(4U) == 0U)
    {
        Arm_Damiao_motor5();
    }
    if (Arm_MotorTxDisabledByIndex(5U) == 0U)
    {
        Arm_Damiao_motor6();
    }
}

static ArmMotorData_t *Arm_GetMotorDataByIndex(uint8_t logical_motor)
{
    if (logical_motor < 3U)
    {
        if (g_ShoulderType == SHOULDER_TYPE_DARAN)
        {
            return &Daran_motor_data[logical_motor];
        }

        return &Linzu_motor_data[logical_motor];
    }

    if (logical_motor < ARM_LOGICAL_MOTOR_COUNT)
    {
        return &Damiao_motor_data[logical_motor - 3U];
    }

    return NULL;
}

static RobStride_Motor_t *Arm_GetLinzuMotorByIndex(uint8_t logical_motor)
{
    RobStride_Motor_t *linzu_motors[3] = {&motor1, &motor2, &motor3};

    if (logical_motor >= 3U)
    {
        return NULL;
    }

    return linzu_motors[logical_motor];
}

static uint8_t Arm_GetDaranMotorIdByIndex(uint8_t logical_motor, uint8_t *motor_id)
{
    static const uint8_t daran_motor_ids[3] = {
        MOTOR_DARAN_1_ID,
        MOTOR_DARAN_2_ID,
        MOTOR_DARAN_3_ID,
    };

    if (logical_motor >= 3U)
    {
        return 0U;
    }

    *motor_id = daran_motor_ids[logical_motor];
    return 1U;
}

static uint8_t Arm_GetDamiaoMotorInfoByIndex(uint8_t logical_motor, uint16_t *motor_id, uint16_t *motor_index)
{
    static const uint16_t damiao_motor_ids[3] = {
        MOTOR_DAMIAO_4_ID,
        MOTOR_DAMIAO_5_ID,
        MOTOR_DAMIAO_6_ID,
    };
    static const uint16_t damiao_motor_indices[3] = {
        Motor4,
        Motor5,
        Motor6,
    };
    uint8_t damiao_index;

    if ((logical_motor < 3U) || (logical_motor >= ARM_LOGICAL_MOTOR_COUNT))
    {
        return 0U;
    }

    damiao_index = logical_motor - 3U;
    *motor_id = damiao_motor_ids[damiao_index];
    *motor_index = damiao_motor_indices[damiao_index];
    return 1U;
}

static void Arm_ReenableLinzuMotor(uint8_t logical_motor, RobStride_Motor_t *motor, ArmMotorData_t *motor_data)
{
    if (motor->Pos_Info.pattern == 2U)
    {
        return;
    }

    Set_RobStride_Motor_parameter(motor, CAN_HANDLE_2, 0X7005, CSP_control_mode, 'j');
    Enable_Motor(motor, CAN_HANDLE_2);
    Arm_SyncTargetAngleRamp(logical_motor, motor_data);
    (void)Arm_SendLinzuTarget(motor, motor_data);
}

static void Arm_ReenableDamiaoMotor(uint8_t logical_motor, uint16_t motor_id, uint16_t motor_index, ArmMotorData_t *motor_data)
{
    if (arm_motor[motor_index].para.state == ARM_DAMIAO_ENABLE_STATE)
    {
        return;
    }

    enable_motor_mode(CAN_HANDLE_2, motor_id, POS_MODE);
    set_DM_mode(motor_index, POS_MODE);
    Arm_SyncTargetAngleRamp(logical_motor, motor_data);
    (void)Arm_SendDamiaoTarget(motor_id, motor_data);
}

void Arm_CheckAndReenableDisabledMotors(void)
{
    static uint32_t last_check_tick = 0U;
    uint32_t now_tick = HAL_GetTick();

    if ((now_tick - last_check_tick) < ARM_REENABLE_CHECK_PERIOD_MS)
    {
        return;
    }

    last_check_tick = now_tick;

    if (Arm_MotorTxDisabledByIndex(0U) == 0U)
    {
        Arm_ReenableLinzuMotor(0U, &motor1, &Linzu_motor_data[0]);
    }
    if (Arm_MotorTxDisabledByIndex(1U) == 0U)
    {
        Arm_ReenableLinzuMotor(1U, &motor2, &Linzu_motor_data[1]);
    }
    if (Arm_MotorTxDisabledByIndex(2U) == 0U)
    {
        Arm_ReenableLinzuMotor(2U, &motor3, &Linzu_motor_data[2]);
    }

    if (Arm_MotorTxDisabledByIndex(3U) == 0U)
    {
        Arm_ReenableDamiaoMotor(3U, MOTOR_DAMIAO_4_ID, Motor4, &Damiao_motor_data[0]);
    }
    if (Arm_MotorTxDisabledByIndex(4U) == 0U)
    {
        Arm_ReenableDamiaoMotor(4U, MOTOR_DAMIAO_5_ID, Motor5, &Damiao_motor_data[1]);
    }
    if (Arm_MotorTxDisabledByIndex(5U) == 0U)
    {
        Arm_ReenableDamiaoMotor(5U, MOTOR_DAMIAO_6_ID, Motor6, &Damiao_motor_data[2]);
    }
}

void Arm_RequestDisabledFeedback(void)
{
    static uint32_t last_request_tick = 0U;
    uint32_t now_tick = HAL_GetTick();
    uint8_t logical_motor;

    if ((now_tick - last_request_tick) < ARM_DISABLED_FEEDBACK_PERIOD_MS)
    {
        return;
    }

    last_request_tick = now_tick;

    if (g_ShoulderType == SHOULDER_TYPE_LINGZU)
    {
        for (logical_motor = 0U; logical_motor < 3U; logical_motor++)
        {
            RobStride_Motor_t *motor;

            if (Arm_MotorTxDisabledByIndex(logical_motor) == 0U)
            {
                continue;
            }

            motor = Arm_GetLinzuMotorByIndex(logical_motor);
            if (motor == NULL)
            {
                continue;
            }

            Get_RobStride_Motor_parameter(motor, CAN_HANDLE_2, 0x7019U);
            osDelay(1U);
        }
    }

    for (logical_motor = 3U; logical_motor < ARM_LOGICAL_MOTOR_COUNT; logical_motor++)
    {
        uint16_t motor_id;
        uint16_t motor_index;

        if (Arm_MotorTxDisabledByIndex(logical_motor) == 0U)
        {
            continue;
        }

        if (Arm_GetDamiaoMotorInfoByIndex(logical_motor, &motor_id, &motor_index) == 0U)
        {
            continue;
        }

        (void)motor_index;
        disable_motor_mode(CAN_HANDLE_2, motor_id, POS_MODE);
        osDelay(1U);
    }
}

#define ARM_LINZU_ZERO_STOP_DELAY_MS 20U
#define ARM_LINZU_ZERO_APPLY_DELAY_MS 50U
#define ARM_LINZU_ZERO_REENABLE_DELAY_MS 20U

static void Arm_Linzu_SendStop(RobStride_Motor_t *motor)
{
    uint8_t txdata[8] = {0};
    uint32_t ext_id = ((uint32_t)Communication_Type_MotorStop << 24) |
                      ((uint32_t)motor->Master_CAN_ID << 8) |
                      motor->CAN_ID;

    canx_send_ext_data(CAN_HANDLE_2, ext_id, txdata, 8);
}

static void Arm_Linzu_SendSetZero(RobStride_Motor_t *motor)
{
    uint8_t txdata[8] = {0};
    uint32_t ext_id = ((uint32_t)Communication_Type_SetPosZero << 24) |
                      ((uint32_t)motor->Master_CAN_ID << 8) |
                      motor->CAN_ID;

    txdata[0] = 1U;
    canx_send_ext_data(CAN_HANDLE_2, ext_id, txdata, 8);
}

static void Arm_Linzu_SaveZeroAndHold(RobStride_Motor_t *motor, ArmMotorData_t *motor_data)
{
    motor_data->target_angle = 0.0f;
    motor_data->target_velocity = 1.0f;

    Arm_Linzu_SendStop(motor);
    osDelay(ARM_LINZU_ZERO_STOP_DELAY_MS);
    Arm_Linzu_SendSetZero(motor);
    osDelay(ARM_LINZU_ZERO_APPLY_DELAY_MS);

    Set_RobStride_Motor_parameter(motor, CAN_HANDLE_2, 0X7005, CSP_control_mode, 'j');
    osDelay(ARM_LINZU_ZERO_REENABLE_DELAY_MS);
    Enable_Motor(motor, CAN_HANDLE_2);
    osDelay(ARM_LINZU_ZERO_REENABLE_DELAY_MS);
    Set_RobStride_Motor_parameter(motor, CAN_HANDLE_2, 0X7017, motor_data->target_velocity, 'p');
    osDelay(ARM_LINZU_ZERO_REENABLE_DELAY_MS);
    (void)Arm_SendLinzuTarget(motor, motor_data);
}

uint8_t Arm_AdjustMotorTargetByIndex(uint8_t logical_motor, float delta_angle)
{
    ArmMotorData_t *motor_data = Arm_GetMotorDataByIndex(logical_motor);

    if (motor_data == NULL)
    {
        return 0U;
    }

    motor_data->target_angle += delta_angle;
    return 1U;
}

uint8_t Arm_EnableMotorByIndex(uint8_t logical_motor)
{
    ArmMotorData_t *motor_data = Arm_GetMotorDataByIndex(logical_motor);

    if (motor_data == NULL)
    {
        return 0U;
    }

    Arm_SyncTargetAngleRamp(logical_motor, motor_data);

    if (logical_motor < 3U)
    {
        if (g_ShoulderType == SHOULDER_TYPE_LINGZU)
        {
            RobStride_Motor_t *motor = Arm_GetLinzuMotorByIndex(logical_motor);

            if (motor == NULL)
            {
                return 0U;
            }

            Set_RobStride_Motor_parameter(motor, CAN_HANDLE_2, 0X7005, CSP_control_mode, 'j');
            Enable_Motor(motor, CAN_HANDLE_2);
            Arm_SetMotorTxDisabledByIndex(logical_motor, 0U);
            return Arm_SendLinzuTarget(motor, motor_data);
        }

        if (g_ShoulderType == SHOULDER_TYPE_DARAN)
        {
            uint8_t motor_id;

            if (Arm_GetDaranMotorIdByIndex(logical_motor, &motor_id) == 0U)
            {
                return 0U;
            }

            set_mode(CAN_HANDLE_2, motor_id, 2);
            osDelay(1);
            Arm_SetMotorTxDisabledByIndex(logical_motor, 0U);
            return Arm_SendDaranTarget(motor_id, motor_data);
        }

        return 0U;
    }

    {
        uint16_t motor_id;
        uint16_t motor_index;

        if (Arm_GetDamiaoMotorInfoByIndex(logical_motor, &motor_id, &motor_index) == 0U)
        {
            return 0U;
        }

        enable_motor_mode(CAN_HANDLE_2, motor_id, POS_MODE);
        set_DM_mode(motor_index, POS_MODE);
        Arm_SetMotorTxDisabledByIndex(logical_motor, 0U);
        return Arm_SendDamiaoTarget(motor_id, motor_data);
    }
}

uint8_t Arm_EnableAllMotors(void)
{
    uint8_t logical_motor;
    uint8_t all_enabled = 1U;

    for (logical_motor = 0U; logical_motor < ARM_LOGICAL_MOTOR_COUNT; logical_motor++)
    {
        if (Arm_EnableMotorByIndex(logical_motor) == 0U)
        {
            all_enabled = 0U;
        }

        osDelay(1);
    }

    return all_enabled;
}

uint8_t Arm_DisableMotorByIndex(uint8_t logical_motor)
{
    if (logical_motor < 3U)
    {
        if (g_ShoulderType == SHOULDER_TYPE_LINGZU)
        {
            RobStride_Motor_t *motor = Arm_GetLinzuMotorByIndex(logical_motor);

            if (motor == NULL)
            {
                return 0U;
            }

            Disenable_Motor(motor, CAN_HANDLE_2, 0U);
            Arm_SetMotorTxDisabledByIndex(logical_motor, 1U);
            return 1U;
        }

        if (g_ShoulderType == SHOULDER_TYPE_DARAN)
        {
            uint8_t motor_id;

            if (Arm_GetDaranMotorIdByIndex(logical_motor, &motor_id) == 0U)
            {
                return 0U;
            }

            set_mode(CAN_HANDLE_2, motor_id, 1);
            Arm_SetMotorTxDisabledByIndex(logical_motor, 1U);
            return 1U;
        }

        return 0U;
    }

    {
        uint16_t motor_id;
        uint16_t motor_index;

        if (Arm_GetDamiaoMotorInfoByIndex(logical_motor, &motor_id, &motor_index) == 0U)
        {
            return 0U;
        }

        disable_motor_mode(CAN_HANDLE_2, motor_id, POS_MODE);
        Arm_SetMotorTxDisabledByIndex(logical_motor, 1U);
        return 1U;
    }
}

uint8_t Arm_SaveMotorZeroByIndex(uint8_t logical_motor)
{
    ArmMotorData_t *motor_data = Arm_GetMotorDataByIndex(logical_motor);

    if (motor_data == NULL)
    {
        return 0U;
    }

    if (logical_motor < 3U)
    {
        if (g_ShoulderType == SHOULDER_TYPE_LINGZU)
        {
            RobStride_Motor_t *motor = Arm_GetLinzuMotorByIndex(logical_motor);

            if (motor == NULL)
            {
                return 0U;
            }

            Arm_Linzu_SaveZeroAndHold(motor, motor_data);
            Arm_SyncTargetAngleRamp(logical_motor, motor_data);
            return 1U;
        }

        if (g_ShoulderType == SHOULDER_TYPE_DARAN)
        {
            uint8_t motor_id;

            if (Arm_GetDaranMotorIdByIndex(logical_motor, &motor_id) == 0U)
            {
                return 0U;
            }

            motor_data->target_angle = 0.0f;
            Arm_SyncTargetAngleRamp(logical_motor, motor_data);
            set_zero_position(CAN_HANDLE_2, motor_id);
            osDelay(1);
            set_mode(CAN_HANDLE_2, motor_id, 2);
            osDelay(1);
            return Arm_SendDaranTarget(motor_id, motor_data);
        }

        return 0U;
    }

    {
        uint16_t motor_id;
        uint16_t motor_index;

        if (Arm_GetDamiaoMotorInfoByIndex(logical_motor, &motor_id, &motor_index) == 0U)
        {
            return 0U;
        }

        motor_data->target_angle = 0.0f;
        Arm_SyncTargetAngleRamp(logical_motor, motor_data);
        CAN_Send_Save_Zero(CAN_HANDLE_2, motor_id);
        osDelay(1);
        CAN_Send_Enter(CAN_HANDLE_2, motor_id);
        osDelay(1);
        return Arm_SendDamiaoTarget(motor_id, motor_data);
    }
}

void Arm_save_position(void)
{
    Damiao_motor_data[0].target_angle = 0.0f;
    Damiao_motor_data[1].target_angle = 0.0f;
    Damiao_motor_data[2].target_angle = 0.0f;

    Arm_Linzu_SaveZeroAndHold(&motor1, &Linzu_motor_data[0]);
    osDelay(1);
    Arm_Linzu_SaveZeroAndHold(&motor2, &Linzu_motor_data[1]);
    osDelay(1);
    Arm_Linzu_SaveZeroAndHold(&motor3, &Linzu_motor_data[2]);
    osDelay(1);

    Arm_SyncTargetAngleRamp(0U, &Linzu_motor_data[0]);
    Arm_SyncTargetAngleRamp(1U, &Linzu_motor_data[1]);
    Arm_SyncTargetAngleRamp(2U, &Linzu_motor_data[2]);
    Arm_SyncTargetAngleRamp(3U, &Damiao_motor_data[0]);
    Arm_SyncTargetAngleRamp(4U, &Damiao_motor_data[1]);
    Arm_SyncTargetAngleRamp(5U, &Damiao_motor_data[2]);

    CAN_Send_Save_Zero(CAN_HANDLE_2, MOTOR_DAMIAO_4_ID);
    osDelay(1);
    CAN_Send_Save_Zero(CAN_HANDLE_2, MOTOR_DAMIAO_5_ID);
    osDelay(1);
    CAN_Send_Save_Zero(CAN_HANDLE_2, MOTOR_DAMIAO_6_ID);
    osDelay(1);

    CAN_Send_Enter(CAN_HANDLE_2, MOTOR_DAMIAO_4_ID);
    osDelay(1);
    (void)Arm_SendDamiaoTarget(MOTOR_DAMIAO_4_ID, &Damiao_motor_data[0]);
    osDelay(1);
    CAN_Send_Enter(CAN_HANDLE_2, MOTOR_DAMIAO_5_ID);
    osDelay(1);
    (void)Arm_SendDamiaoTarget(MOTOR_DAMIAO_5_ID, &Damiao_motor_data[1]);
    osDelay(1);
    CAN_Send_Enter(CAN_HANDLE_2, MOTOR_DAMIAO_6_ID);
    osDelay(1);
    (void)Arm_SendDamiaoTarget(MOTOR_DAMIAO_6_ID, &Damiao_motor_data[2]);
}
// void Arm_Motor_Disable_All(void)
// {
//     Disenable_Motor(&motor1, CAN_HANDLE_2, 0U);
//     osDelay(1);
//     Disenable_Motor(&motor2, CAN_HANDLE_2, 0U);
//     osDelay(1);
//     Disenable_Motor(&motor3, CAN_HANDLE_2, 0U);
//     osDelay(1);
//     disable_motor_mode(CAN_HANDLE_2, MOTOR_DAMIAO_4_ID, POS_MODE);
//     osDelay(1);
//     disable_motor_mode(CAN_HANDLE_2, MOTOR_DAMIAO_5_ID, POS_MODE);
//     osDelay(1);
//     disable_motor_mode(CAN_HANDLE_2, MOTOR_DAMIAO_6_ID, POS_MODE);
// }
//  void Arm_Motor_Enable_All(void)
// {
//     Disenable_Motor(&motor1, CAN_HANDLE_2, 0U);
//     osDelay(1);
//     Disenable_Motor(&motor2, CAN_HANDLE_2, 0U);
//     osDelay(1);
//     Disenable_Motor(&motor3, CAN_HANDLE_2, 0U);
//     osDelay(1);
//     disable_motor_mode(CAN_HANDLE_2, MOTOR_DAMIAO_4_ID, POS_MODE);
//     osDelay(1);
//     disable_motor_mode(CAN_HANDLE_2, MOTOR_DAMIAO_5_ID, POS_MODE);
//     osDelay(1);
//     disable_motor_mode(CAN_HANDLE_2, MOTOR_DAMIAO_6_ID, POS_MODE);

// }
