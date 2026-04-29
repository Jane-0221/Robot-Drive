#include "Sbus.h"
#include <string.h>

SBUS_CH_Struct SBUS_CH;

// 定义256字节的数据缓冲区
 uint8_t sbus_data_buffer[256];

// 存储256字节数据的函数
void store_sbus_data(const uint8_t *data, uint16_t size)
{
    if (data == NULL || size == 0) {
        return;
    }
    
    // 确保不会超出缓冲区大小
    uint16_t copy_size = (size > 256) ? 256 : size;
    
    // 使用memcpy安全地复制数据
    memcpy(sbus_data_buffer, data, copy_size);
    
    // 如果数据小于256字节，剩余部分填充0
    if (copy_size < 256) {
        memset(sbus_data_buffer + copy_size, 0, 256 - copy_size);
    }
}

// 获取存储的数据缓冲区指针
uint8_t* get_sbus_data_buffer(void)
{
    return sbus_data_buffer;
}

// 获取缓冲区大小
uint16_t get_sbus_buffer_size(void)
{
    return 256;
}

void update_sbus(volatile const uint8_t *sbus_buf,SBUS_CH_Struct *SBUS_CH)
{


        SBUS_CH->ConnectState = 1;       
        SBUS_CH->CH1 =  (((uint16_t)sbus_buf[1] >> 0 | ((uint16_t)sbus_buf[2] << 8 )) & 0x07FF);//264 1024 1807
        SBUS_CH->CH2 =  (((uint16_t)sbus_buf[2] >> 3 | ((uint16_t)sbus_buf[3] << 5 )) & 0x07FF);//240 1024 1807
        SBUS_CH->CH3 =  (((uint16_t)sbus_buf[3] >> 6 | ((uint16_t)sbus_buf[4] << 2 ) | (int16_t)sbus_buf[5] << 10 ) & 0x07FF);//240 1024 1807
        SBUS_CH->CH4 =  (((uint16_t)sbus_buf[5] >> 1 | ((uint16_t)sbus_buf[6] << 7 )) & 0x07FF);//240  1024 1019 1807
        SBUS_CH->CH5 =  (((int16_t)sbus_buf[6] >> 4 | ((int16_t)sbus_buf[7] << 4 )) & 0x07FF);//240 1807
        SBUS_CH->CH6 =  (((int16_t)sbus_buf[7] >> 7 | ((int16_t)sbus_buf[8] << 1 ) | (int16_t)sbus_buf[9] << 9 ) & 0x07FF);//240 1807
        SBUS_CH->CH7 =  (((int16_t)sbus_buf[9] >> 2 | ((int16_t)sbus_buf[10] << 6 )) & 0x07FF);//240 1024 1807
        SBUS_CH->CH8 =  (((int16_t)sbus_buf[10] >> 5 | ((int16_t)sbus_buf[11] << 3 )) & 0x07FF);//240 1807
        SBUS_CH->CH9 =  (((int16_t)sbus_buf[12] << 0 | ((int16_t)sbus_buf[13] << 8 )) & 0x07FF);//240 1807
        SBUS_CH->CH10 = (((int16_t)sbus_buf[13] >> 3 | ((int16_t)sbus_buf[14] << 5 )) & 0x07FF);//240 1807
        SBUS_CH->CH11 = (((int16_t)sbus_buf[14] >> 6 | ((int16_t)sbus_buf[15] << 2 ) | (int16_t)sbus_buf[16] << 10 ) & 0x07FF);
        SBUS_CH->CH12 = (((int16_t)sbus_buf[16] >> 1 | ((int16_t)sbus_buf[17] << 7 )) & 0x07FF);
        SBUS_CH->CH13 = (((int16_t)sbus_buf[17] >> 4 | ((int16_t)sbus_buf[18] << 4 )) & 0x07FF);
        SBUS_CH->CH14 = (((int16_t)sbus_buf[18] >> 7 | ((int16_t)sbus_buf[19] << 1 ) | (int16_t)sbus_buf[20] << 9 ) & 0x07FF);
        SBUS_CH->CH15 = (((int16_t)sbus_buf[20] >> 2 | ((int16_t)sbus_buf[21] << 6 )) & 0x07FF);
        SBUS_CH->CH16 = (((int16_t)sbus_buf[21] >> 5 | ((int16_t)sbus_buf[22] << 3 )) & 0x07FF);

}