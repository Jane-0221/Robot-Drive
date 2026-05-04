#include "uart_protocol.h"

#include "FreeRTOS.h"
#include "task.h"
#include <string.h>

uint8_t uart_protocol_raw_data[256] = {0};
static uint8_t uart_protocol_parse_data[256] = {0};
static volatile uint16_t uart_protocol_raw_size = 0U;
static volatile uint8_t uart_protocol_data_pending = 0U;
static PcArmMotorCtrl_t pc_arm_motor_ctrl_pending = {0U, 0U};
static volatile uint8_t pc_arm_motor_ctrl_pending_valid = 0U;

DnData_t pc_dn_data = {
    .pc_target_servo_angles = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
    .pc_target_motor_angles = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
    .pc_pump_state = 0,
    .pc_target_lift_height = 0,
};

UpData_t up_tx_data = {
    .air_path_state = 0.0f,
    .suck_state = 0.0f,
    .head_motor_angle_1 = 0.0f,
    .head_motor_angle_2 = 0.0f,
    .arm_motor_angle_1 = 0.0f,
    .arm_motor_angle_2 = 0.0f,
    .arm_motor_angle_3 = 0.0f,
    .arm_motor_angle_4 = 0.0f,
    .arm_motor_angle_5 = 0.0f,
    .arm_motor_angle_6 = 0.0f,
    .lift_height = 0.0f,
    .chassis_vx = 0.0f,
    .chassis_vy = 0.0f,
    .chassis_yaw = 0.0f,
};

static void write_float_le(uint8_t *buffer, uint16_t *idx, float value)
{
    memcpy(&buffer[*idx], &value, sizeof(float));
    *idx += (uint16_t)sizeof(float);
}

uint16_t crc16_ccitt(uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFF;

    for (uint16_t i = 0; i < len; i++)
    {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t j = 0; j < 8; j++)
        {
            if (crc & 0x8000)
            {
                crc = (crc << 1) ^ 0x1021;
            }
            else
            {
                crc <<= 1;
            }
        }
    }

    return crc;
}

void pack_up_frame(UpData_t *data, uint8_t *frame_buf)
{
    uint16_t idx = 0;

    frame_buf[idx++] = FRAME_HEADER1;
    frame_buf[idx++] = FRAME_HEADER2;
    frame_buf[idx++] = UP_FRAME_TYPE;
    frame_buf[idx++] = UP_DATA_LEN;

    write_float_le(frame_buf, &idx, data->air_path_state);
    write_float_le(frame_buf, &idx, data->suck_state);
    write_float_le(frame_buf, &idx, data->head_motor_angle_1);
    write_float_le(frame_buf, &idx, data->head_motor_angle_2);
    write_float_le(frame_buf, &idx, data->arm_motor_angle_1);
    write_float_le(frame_buf, &idx, data->arm_motor_angle_2);
    write_float_le(frame_buf, &idx, data->arm_motor_angle_3);
    write_float_le(frame_buf, &idx, data->arm_motor_angle_4);
    write_float_le(frame_buf, &idx, data->arm_motor_angle_5);
    write_float_le(frame_buf, &idx, data->arm_motor_angle_6);
    write_float_le(frame_buf, &idx, data->lift_height);
    write_float_le(frame_buf, &idx, data->chassis_vx);
    write_float_le(frame_buf, &idx, data->chassis_vy);
    write_float_le(frame_buf, &idx, data->chassis_yaw);

    {
        uint16_t crc = crc16_ccitt(&frame_buf[2], UP_DATA_LEN + 2);
        frame_buf[idx++] = (uint8_t)(crc & 0xFF);
        frame_buf[idx++] = (uint8_t)((crc >> 8) & 0xFF);
    }

    frame_buf[idx++] = FRAME_TAIL1;
    frame_buf[idx++] = FRAME_TAIL2;
}

void unpack_dn_frame(uint8_t *frame_buf, DnData_t *data)
{
    uint8_t idx = 4;

    for (int i = 0; i < 6; i++)
    {
        float value;
        uint8_t *p = (uint8_t *)&value;
        p[0] = frame_buf[idx];
        p[1] = frame_buf[idx + 1];
        p[2] = frame_buf[idx + 2];
        p[3] = frame_buf[idx + 3];
        data->pc_target_servo_angles[i] = value;
        idx += 4;
    }

    for (int i = 0; i < 6; i++)
    {
        float value;
        uint8_t *p = (uint8_t *)&value;
        p[0] = frame_buf[idx];
        p[1] = frame_buf[idx + 1];
        p[2] = frame_buf[idx + 2];
        p[3] = frame_buf[idx + 3];
        data->pc_target_motor_angles[i] = value;
        idx += 4;
    }

    data->pc_pump_state = frame_buf[idx++];
    data->pc_target_lift_height = (uint16_t)((frame_buf[idx] | (frame_buf[idx + 1] << 8)) / 10);
}

static uint8_t is_valid_dn_frame(const uint8_t *frame_buf)
{
    uint16_t crc_rx;
    uint16_t crc_calc;

    if (frame_buf == NULL)
    {
        return 0U;
    }

    if ((frame_buf[0] != FRAME_HEADER1) ||
        (frame_buf[1] != FRAME_HEADER2) ||
        (frame_buf[2] != DN_FRAME_TYPE) ||
        (frame_buf[3] != DN_DATA_LEN) ||
        (frame_buf[DN_FRAME_LEN - 2U] != FRAME_TAIL1) ||
        (frame_buf[DN_FRAME_LEN - 1U] != FRAME_TAIL2))
    {
        return 0U;
    }

    crc_rx = (uint16_t)frame_buf[DN_DATA_LEN + 4U] |
             ((uint16_t)frame_buf[DN_DATA_LEN + 5U] << 8);
    crc_calc = crc16_ccitt((uint8_t *)&frame_buf[2], DN_DATA_LEN + 2U);

    return (crc_rx == crc_calc) ? 1U : 0U;
}

static uint8_t is_valid_pc_arm_motor_ctrl_frame(const uint8_t *frame_buf)
{
    uint16_t crc_rx;
    uint16_t crc_calc;

    if (frame_buf == NULL)
    {
        return 0U;
    }

    if ((frame_buf[0] != FRAME_HEADER1) ||
        (frame_buf[1] != FRAME_HEADER2) ||
        (frame_buf[2] != PC_ARM_MOTOR_CTRL_FRAME_TYPE) ||
        (frame_buf[3] != PC_ARM_MOTOR_CTRL_DATA_LEN) ||
        (frame_buf[PC_ARM_MOTOR_CTRL_FRAME_LEN - 2U] != FRAME_TAIL1) ||
        (frame_buf[PC_ARM_MOTOR_CTRL_FRAME_LEN - 1U] != FRAME_TAIL2))
    {
        return 0U;
    }

    crc_rx = (uint16_t)frame_buf[PC_ARM_MOTOR_CTRL_DATA_LEN + 4U] |
             ((uint16_t)frame_buf[PC_ARM_MOTOR_CTRL_DATA_LEN + 5U] << 8);
    crc_calc = crc16_ccitt((uint8_t *)&frame_buf[2], PC_ARM_MOTOR_CTRL_DATA_LEN + 2U);

    return (crc_rx == crc_calc) ? 1U : 0U;
}

static void unpack_pc_arm_motor_ctrl_frame(const uint8_t *frame_buf, PcArmMotorCtrl_t *command)
{
    if ((frame_buf == NULL) || (command == NULL))
    {
        return;
    }

    command->motor_index = frame_buf[4];
    command->enable_state = frame_buf[5];
}

uint8_t UART_Protocol_UnpackLatest(DnData_t *data)
{
    uint16_t size;
    uint16_t frame_pos = 0U;
    PcArmMotorCtrl_t arm_motor_ctrl_command = {0U, 0U};
    uint8_t dn_found = 0U;
    uint8_t arm_motor_ctrl_found = 0U;

    if ((data == NULL) || (uart_protocol_data_pending == 0U))
    {
        return 0U;
    }

    taskENTER_CRITICAL();
    size = uart_protocol_raw_size;
    memcpy(uart_protocol_parse_data, uart_protocol_raw_data, size);
    uart_protocol_data_pending = 0U;
    taskEXIT_CRITICAL();

    if (size < PC_ARM_MOTOR_CTRL_FRAME_LEN)
    {
        return 0U;
    }

    for (uint16_t pos = 0U; pos < size; pos++)
    {
        uint16_t remaining = (uint16_t)(size - pos);

        if ((remaining >= DN_FRAME_LEN) &&
            (is_valid_dn_frame(&uart_protocol_parse_data[pos]) != 0U))
        {
            frame_pos = pos;
            dn_found = 1U;
        }

        if ((remaining >= PC_ARM_MOTOR_CTRL_FRAME_LEN) &&
            (is_valid_pc_arm_motor_ctrl_frame(&uart_protocol_parse_data[pos]) != 0U))
        {
            unpack_pc_arm_motor_ctrl_frame(&uart_protocol_parse_data[pos], &arm_motor_ctrl_command);
            arm_motor_ctrl_found = 1U;
        }
    }

    if (arm_motor_ctrl_found != 0U)
    {
        taskENTER_CRITICAL();
        pc_arm_motor_ctrl_pending = arm_motor_ctrl_command;
        pc_arm_motor_ctrl_pending_valid = 1U;
        taskEXIT_CRITICAL();
    }

    if (dn_found != 0U)
    {
        unpack_dn_frame(&uart_protocol_parse_data[frame_pos], data);
    }

    return ((dn_found != 0U) || (arm_motor_ctrl_found != 0U)) ? 1U : 0U;
}

uint8_t UART_Protocol_GetArmMotorCtrlCommand(PcArmMotorCtrl_t *command)
{
    if (command == NULL)
    {
        return 0U;
    }

    taskENTER_CRITICAL();
    if (pc_arm_motor_ctrl_pending_valid == 0U)
    {
        taskEXIT_CRITICAL();
        return 0U;
    }

    *command = pc_arm_motor_ctrl_pending;
    pc_arm_motor_ctrl_pending_valid = 0U;
    taskEXIT_CRITICAL();

    return 1U;
}

HAL_StatusTypeDef send_frame(UART_HandleTypeDef *huart, uint8_t *frame_buf, uint16_t len)
{
    return HAL_UART_Transmit(huart, frame_buf, len, 100);
}

HAL_StatusTypeDef send_up_frame(UART_HandleTypeDef *huart)
{
    static uint8_t frame_buf[UP_FRAME_LEN];

    if (huart == NULL)
    {
        return HAL_ERROR;
    }

    if (huart->gState != HAL_UART_STATE_READY)
    {
        return HAL_BUSY;
    }

    pack_up_frame(&up_tx_data, frame_buf);

    return HAL_UART_Transmit_DMA(huart, frame_buf, UP_FRAME_LEN);
}

void store_uart_protocol_data(const uint8_t *data, uint16_t size)
{
    if (data == NULL || size == 0)
    {
        return;
    }

    uint16_t copy_size = (size > 256U) ? 256U : size;

    memcpy(uart_protocol_raw_data, data, copy_size);
    uart_protocol_raw_size = copy_size;
    uart_protocol_data_pending = 1U;
}
