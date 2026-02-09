#include "LZ_motor_driver.h"
#include "CAN_receive_send.h"
#include "string.h"
#include "User_math.h"

/**
 * @brief 灵足电机先更改模式再使能，注意发送位置和速度时要加上3位id和原来id构成11位id
 */

/**
 * @brief 发送 MIT 协议指令（通用）
 */
void lz_send_command(uint8_t can_bus, uint16_t motor_id, uint8_t *data) {
    FDCAN_HandleTypeDef *hfdcan = Get_CanHandle(can_bus);
    Fdcanx_SendData(hfdcan, motor_id, data, 8);
}

/**
 * @brief 使能电机
 */
void lz_enable_motor(uint8_t can_bus, uint8_t motor_id) {
    uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
    lz_send_command(can_bus, motor_id, data);
}

/** 
 * @brief 停止电机
 */
void lz_disable_motor(uint8_t can_bus, uint8_t motor_id) {
    uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};
    lz_send_command(can_bus, motor_id, data);
}

/**
 * @brief 发送 MIT 动态参数（指令3）
 */
void lz_send_mit_params(uint8_t can_bus, uint8_t motor_id, float angle, float speed, float kp, float kd, float torque) {
    uint16_t angle_uint = float_to_uint(angle, P_MIN, P_MAX, 16);
    uint16_t speed_uint = float_to_uint(speed, V_MIN, V_MAX, 12);
    uint16_t kp_uint = float_to_uint(kp, KP_MIN, KP_MAX, 12);
    uint16_t kd_uint = float_to_uint(kd, KD_MIN, KD_MAX, 12);
    uint16_t torque_uint = float_to_uint(torque, T_MIN, T_MAX, 12);

    uint8_t data[8] = {
        (uint8_t)(angle_uint >> 8), 
        (uint8_t)(angle_uint & 0xFF),
        (uint8_t)((speed_uint >> 4) & 0xFF),
        (uint8_t)(((speed_uint & 0x0F) << 4) | ((kp_uint >> 8) & 0x0F)),
        (uint8_t)(kp_uint & 0xFF),
        (uint8_t)((kd_uint >> 4) & 0xFF),
        (uint8_t)(((kd_uint & 0x0F) << 4) | ((torque_uint >> 8) & 0x0F)),
        (uint8_t)(torque_uint & 0xFF)
    };
    lz_send_command(can_bus, motor_id, data);
}

/**
 * @brief 设置零点（指令4）
 */
void lz_set_zero(uint8_t can_bus, uint8_t motor_id) {
    uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE};
    lz_send_command(can_bus, motor_id, data);
}

/**
 * @brief 清除错误（指令5）
 */
void lz_clear_fault(uint8_t can_bus, uint8_t motor_id) {
    uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFB};
    lz_send_command(can_bus, motor_id, data);
}

/**
 * @brief 设置运行模式（指令6）
 */
void lz_set_mode(uint8_t can_bus, uint8_t motor_id, uint8_t mode) {
    uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, mode, 0xFC};
    lz_send_command(can_bus, motor_id, data);
}

/**
 * @brief 修改电机ID（指令7）
 */
void lz_set_id(uint8_t can_bus, uint8_t motor_id, uint8_t new_id) {
    uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, new_id, 0xFA};
    lz_send_command(can_bus, motor_id, data);
}

/**
 * @brief 修改电机协议（指令8）
 */
void lz_set_protocol(uint8_t can_bus, uint8_t motor_id, uint8_t protocol) {
    uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, protocol, 0xFD};
    lz_send_command(can_bus, motor_id, data);
}

/**
 * @brief 修改主机ID（指令9）
 */
void lz_set_master_id(uint8_t can_bus, uint8_t motor_id, uint8_t master_id) {
    uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, master_id, 0x01};
    lz_send_command(can_bus, motor_id, data);
}

/**
 * @brief 位置模式控制指令（指令10）
 */
void lz_set_position(uint8_t can_bus, uint8_t motor_id, float target_pos, float pos_speed) {
    uint8_t data[8];
    memcpy(&data[0], &target_pos, 4);
    memcpy(&data[4], &pos_speed, 4);
    lz_send_command(can_bus, motor_id, data);
}

/**
 * @brief 速度模式控制指令（指令11）
 */
void lz_set_velocity(uint8_t can_bus, uint16_t motor_id, float target_vel, float current_limit) {
    uint8_t data[8];
    memcpy(&data[0], &target_vel, 4);
    memcpy(&data[4], &current_limit, 4);
    lz_send_command(can_bus, motor_id, data);
}