#ifndef __UART_PROTOCOL_H
#define __UART_PROTOCOL_H

#include "stm32h7xx_hal.h"

#define FRAME_HEADER1 0xAA
#define FRAME_HEADER2 0x55
#define FRAME_TAIL1   0xEE
#define FRAME_TAIL2   0xFF
#define UP_FRAME_TYPE 0x01
#define DN_FRAME_TYPE 0x02
#define PC_ARM_MOTOR_CTRL_FRAME_TYPE 0x03

#define UP_DATA_LEN   56
#define DN_DATA_LEN   51
#define PC_ARM_MOTOR_CTRL_DATA_LEN 2
#define UP_FRAME_LEN  64
#define DN_FRAME_LEN  59
#define PC_ARM_MOTOR_CTRL_FRAME_LEN 10

typedef struct {
    float air_path_state;
    float suck_state;
    float head_motor_angle_1;
    float head_motor_angle_2;
    float arm_motor_angle_1;
    float arm_motor_angle_2;
    float arm_motor_angle_3;
    float arm_motor_angle_4;
    float arm_motor_angle_5;
    float arm_motor_angle_6;
    float lift_height;
    float chassis_vx;
    float chassis_vy;
    float chassis_yaw;
} UpData_t;

typedef struct {
    float    pc_target_servo_angles[6];
    float    pc_target_motor_angles[6];
    uint8_t  pc_pump_state;
    uint16_t pc_target_lift_height;
} DnData_t;

typedef struct {
    uint8_t motor_index;
    uint8_t enable_state;
} PcArmMotorCtrl_t;

extern DnData_t pc_dn_data;
extern uint8_t uart_protocol_raw_data[256];
extern UpData_t up_tx_data;

uint16_t crc16_ccitt(uint8_t *data, uint16_t len);
void pack_up_frame(UpData_t *data, uint8_t *frame_buf);
void unpack_dn_frame(uint8_t *frame_buf, DnData_t *data);
uint8_t UART_Protocol_UnpackLatest(DnData_t *data);
uint8_t UART_Protocol_GetArmMotorCtrlCommand(PcArmMotorCtrl_t *command);
HAL_StatusTypeDef send_frame(UART_HandleTypeDef *huart, uint8_t *frame_buf, uint16_t len);
HAL_StatusTypeDef send_up_frame(UART_HandleTypeDef *huart);
void store_uart_protocol_data(const uint8_t *data, uint16_t size);

#endif // __UART_PROTOCOL_H
