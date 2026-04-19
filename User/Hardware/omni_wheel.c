#include "omni_wheel.h"

#include <math.h>

#include "Robstride04.h"
#include "fdcan.h"
#include "main.h"

#define OMNI_CAN_HANDLE (&hfdcan3)

#define OMNI_MOTOR1_ID 0x01U
#define OMNI_MOTOR2_ID 0x02U
#define OMNI_MOTOR3_ID 0x03U

#define OMNI_CHASSIS_RADIUS_M 0.195f
#define OMNI_WHEEL_RADIUS_M 0.0635f
#define OMNI_SQRT3_OVER_2 0.8660254f

#define OMNI_DIR1 1.0f
#define OMNI_DIR2 1.0f
#define OMNI_DIR3 1.0f

#define OMNI_CURRENT_LIMIT 10.0f
#define OMNI_ACCEL_LIMIT 10.0f
#define OMNI_SPEED_LIMIT 20.0f

volatile float x = 0.0f;
volatile float y = 0.0f;
volatile float w = 0.0f;

static RobStride_Motor_t omni_motor1;
static RobStride_Motor_t omni_motor2;
static RobStride_Motor_t omni_motor3;
static uint8_t omni_initialized = 0U;

static void Omni_Wheel_Init_Motor(RobStride_Motor_t *motor, uint8_t can_id)
{
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

    scale = OMNI_SPEED_LIMIT / max_abs;
    *cmd1 *= scale;
    *cmd2 *= scale;
    *cmd3 *= scale;
}

void Omni_Wheel_Init(void)
{
    x = 0.0f;
    y = 0.0f;
    w = 0.0f;

    Omni_Wheel_Init_Motor(&omni_motor1, OMNI_MOTOR1_ID);
    Omni_Wheel_Init_Motor(&omni_motor2, OMNI_MOTOR2_ID);
    Omni_Wheel_Init_Motor(&omni_motor3, OMNI_MOTOR3_ID);

    omni_initialized = 1U;
}

void Omni_Wheel_Update(void)
{
    float x_cmd;
    float y_cmd;
    float w_cmd;
    float cmd1;
    float cmd2;
    float cmd3;

    if (omni_initialized == 0U)
    {
        return;
    }

    x_cmd = x;
    y_cmd = y;
    w_cmd = w;

    cmd1 = (y_cmd + OMNI_CHASSIS_RADIUS_M * w_cmd) / OMNI_WHEEL_RADIUS_M;
    cmd2 = (-OMNI_SQRT3_OVER_2 * x_cmd - 0.5f * y_cmd + OMNI_CHASSIS_RADIUS_M * w_cmd) / OMNI_WHEEL_RADIUS_M;
    cmd3 = (OMNI_SQRT3_OVER_2 * x_cmd - 0.5f * y_cmd + OMNI_CHASSIS_RADIUS_M * w_cmd) / OMNI_WHEEL_RADIUS_M;

    cmd1 *= OMNI_DIR1;
    cmd2 *= OMNI_DIR2;
    cmd3 *= OMNI_DIR3;

    Omni_Wheel_Limit(&cmd1, &cmd2, &cmd3);

    RobStride_Motor_Speed_control(&omni_motor1, OMNI_CAN_HANDLE, cmd1, OMNI_CURRENT_LIMIT);
    RobStride_Motor_Speed_control(&omni_motor2, OMNI_CAN_HANDLE, cmd2, OMNI_CURRENT_LIMIT);
    RobStride_Motor_Speed_control(&omni_motor3, OMNI_CAN_HANDLE, cmd3, OMNI_CURRENT_LIMIT);
}

void Omni_Wheel_RxCallback(uint32_t ext_id, uint8_t *data)
{
    uint8_t target_id;

    if (data == 0)
    {
        return;
    }

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
