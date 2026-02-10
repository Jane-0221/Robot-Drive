#ifndef __CAN_RECEIVE_SEND_H__
#define __CAN_RECEIVE_SEND_H__

// #include "cover_headerfile_h.h"
#include "fdcan.h"

// 马达基础参数
#define ECD_MAX 8192.0f    // 编码器最大值
#define M3508_P 19.0f      // 13.7f      // M3508电机减速比
#define M2006_P 36.0f      // M2006电机减速比
#define MAX_CURRENT 16384  // M2006+M3508最大电流 20A / MAX_CURRENT
#define MAX_6020_VOL 30000 // 6020最大电压 24V / MAX_6020_VOL

// rm motor data
// notice:默认为3508电机
// 3508/2006用的是电流，6020用的是电压
typedef struct
{
  int16_t set; // 设定的电流 / 电压

  // 原始数据
  uint16_t ecd;          // 编码器数值
  int16_t speed_rpm;     // 转速
  int16_t given_current; // 电调给的电流
  uint8_t temperate;     // 温度（获取不到）
  int16_t last_ecd;      // 上一次编码器的数值

  // 计算数据
  long long ecd_cnt;  // 编码器计数器
  double angle_cnt;   // 转过的总角度 degree
  double angle_zero;  // 编码器0点角度 degree
  double angle;       // -180~180 degree
  double round_speed; // 出轴旋转速度 degree/s

} motor_measure_t;

typedef FDCAN_HandleTypeDef hcan_t; //
// 当前时刻的电机接收flag
extern uint32_t Flag_damiao[6];
extern uint32_t Pre_Flag_damiao[6];
extern FDCAN_HandleTypeDef *Get_CanHandle(uint8_t can_bus);
extern motor_measure_t motor_data[33];
extern void can_init(void);
extern uint8_t canx_send_data(FDCAN_HandleTypeDef *hcan, uint16_t id, uint8_t *data, uint32_t len);
extern uint8_t fdcanx_receive(FDCAN_HandleTypeDef *hfdcan, uint32_t RXFIFO, FDCAN_RxHeaderTypeDef *fdcan_RxHeader, uint8_t *buf);
extern void get_motor_measure(motor_measure_t *ptr, uint8_t data[]);
extern void process_motor_data(motor_measure_t *motor_data);
extern void CAN1_send_current(void); // 发送电机控制电流
extern void CAN2_send_current(void); // 发送电机控制电流
extern void CAN3_send_current(void); // 发送电机控制电流
extern void HAL_FDCAN_ErrorCallback(FDCAN_HandleTypeDef *hfdcan);

#endif /* __CAN_RECEIVE_SEND_H__ */
