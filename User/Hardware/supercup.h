#ifndef __SUPERCUP_H__
#define __SUPERCUO_H__

#include "stdint.h"
#include "fdcan.h"
#include "CAN_receive_send.h"

typedef PACKED_STRUCT()
{
    uint8_t set_max_power;     // 最大功率
    uint8_t cache_energy;      // 缓冲电量
    uint16_t cacheEnergylimit; // 缓冲能量剩余限制
    uint8_t state;             // 状态
    float remain_vol;          // 剩余电压
    float prediect_energy;     // 预测容量 0 - 100%
} Supercap_t;


typedef PACKED_STRUCT()
{
    uint8_t setPower;          // 设置底盘功率
    uint16_t cacheEnergy;      // 缓冲能量
    uint16_t cacheEnergylimit; // 缓冲能量剩余限制
} Supercap_send_t;

typedef enum
{
    Supercap_send_id = 0x306,
    Supercap_receive_id = 0x307,
} Supercup_canid;

extern Supercap_t cap;
extern Supercap_send_t send_data_cap;

void Supercup_decode_candata(FDCAN_HandleTypeDef *hfdcan, uint8_t *data);
void Supercup_send_data(void);
int Supercap_set_power(uint8_t set);
float Supercap_get_remain_vol(void);
float Supercap_get_predict_energy(void);
#endif // !__SUPERCUP_H__
