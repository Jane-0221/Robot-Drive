#include "DT7.h"

DT7_data_s DT7_data;
SBUS_CH_Struct SBUS_CH;
void DT7_decode_data(uint8_t *sbus_buf)
{
    DT7_data.rc.ch[0] = (sbus_buf[0] | (sbus_buf[1] << 8)) & 0x07ff;        //!< Channel 0
    DT7_data.rc.ch[1] = ((sbus_buf[1] >> 3) | (sbus_buf[2] << 5)) & 0x07ff; //!< Channel 1
    DT7_data.rc.ch[2] = ((sbus_buf[2] >> 6) | (sbus_buf[3] << 2) |          //!< Channel 2
                         (sbus_buf[4] << 10)) &
                        0x07ff;
    DT7_data.rc.ch[3] = ((sbus_buf[4] >> 1) | (sbus_buf[5] << 7)) & 0x07ff; //!< Channel 3
    DT7_data.rc.s[0] = ((sbus_buf[5] >> 4) & 0x0003);                       //!< Switch left
    DT7_data.rc.s[1] = ((sbus_buf[5] >> 4) & 0x000C) >> 2;                  //!< Switch right
    DT7_data.mouse.x = sbus_buf[6] | (sbus_buf[7] << 8);                    //!< Mouse X axis
    DT7_data.mouse.y = sbus_buf[8] | (sbus_buf[9] << 8);                    //!< Mouse Y axis
    DT7_data.mouse.z = sbus_buf[10] | (sbus_buf[11] << 8);                  //!< Mouse Z axis
    DT7_data.mouse.press_l = sbus_buf[12];                                  //!< Mouse Left Is Press ?
    DT7_data.mouse.press_r = sbus_buf[13];                                  //!< Mouse Right Is Press ?
    DT7_data.key.v = sbus_buf[14] | (sbus_buf[15] << 8);                    //!< KeyBoard value
    DT7_data.rc.ch[4] = sbus_buf[16] | (sbus_buf[17] << 8);                 // NULL

    DT7_data.rc.ch[0] -= RC_CH_VALUE_OFFSET;
    DT7_data.rc.ch[1] -= RC_CH_VALUE_OFFSET;
    DT7_data.rc.ch[2] -= RC_CH_VALUE_OFFSET;
    DT7_data.rc.ch[3] -= RC_CH_VALUE_OFFSET;
    DT7_data.rc.ch[4] -= RC_CH_VALUE_OFFSET;
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