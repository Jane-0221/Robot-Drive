#ifndef __DT7_H__
#define __DT7_H__

#include "stdint.h"

#define SBUS_RX_BUF_NUM 36u
#define RC_FRAME_LENGTH 18u

#define RC_R_SW 0
#define RC_L_SW 1

#define RC_CH_VALUE_MIN         ((uint16_t)364)
#define RC_CH_VALUE_OFFSET      ((uint16_t)1024)
#define RC_CH_VALUE_MAX         ((uint16_t)1684)

/* ----------------------- RC Switch Definition----------------------------- */
#define RC_SW_UP                ((uint16_t)1)
#define RC_SW_MID               ((uint16_t)3)
#define RC_SW_DOWN              ((uint16_t)2)
#define switch_is_down(a)       (RC_data.rc.s[a] == RC_SW_DOWN)
#define switch_is_mid(a)        (RC_data.rc.s[a] == RC_SW_MID)
#define switch_is_up(a)         (RC_data.rc.s[a] == RC_SW_UP)
/* ----------------------- PC Key Definition-------------------------------- */
#define KEY_PRESSED_OFFSET_W            ((uint16_t)1 << 0)
#define KEY_PRESSED_OFFSET_S            ((uint16_t)1 << 1)
#define KEY_PRESSED_OFFSET_A            ((uint16_t)1 << 2)
#define KEY_PRESSED_OFFSET_D            ((uint16_t)1 << 3)
#define KEY_PRESSED_OFFSET_SHIFT        ((uint16_t)1 << 4)
#define KEY_PRESSED_OFFSET_CTRL         ((uint16_t)1 << 5)
#define KEY_PRESSED_OFFSET_Q            ((uint16_t)1 << 6)
#define KEY_PRESSED_OFFSET_E            ((uint16_t)1 << 7)
#define KEY_PRESSED_OFFSET_R            ((uint16_t)1 << 8)
#define KEY_PRESSED_OFFSET_F            ((uint16_t)1 << 9)
#define KEY_PRESSED_OFFSET_G            ((uint16_t)1 << 10)
#define KEY_PRESSED_OFFSET_Z            ((uint16_t)1 << 11)
#define KEY_PRESSED_OFFSET_X            ((uint16_t)1 << 12)
#define KEY_PRESSED_OFFSET_C            ((uint16_t)1 << 13)
#define KEY_PRESSED_OFFSET_V            ((uint16_t)1 << 14)
#define KEY_PRESSED_OFFSET_B            ((uint16_t)1 << 15)

#define    IF_KEY_PRESSED         (  remote_control.keyboard_value )
#define    IF_KEY_PRESSED_W       ( (remote_control.keyboard_value & KEY_PRESSED_OFFSET_W)    != 0 )
#define    IF_KEY_PRESSED_S       ( (remote_control.keyboard_value & KEY_PRESSED_OFFSET_S)    != 0 )
#define    IF_KEY_PRESSED_A       ( (remote_control.keyboard_value & KEY_PRESSED_OFFSET_A)    != 0 )
#define    IF_KEY_PRESSED_D       ( (remote_control.keyboard_value & KEY_PRESSED_OFFSET_D)    != 0 )
#define    IF_KEY_PRESSED_Q       ( (remote_control.keyboard_value & KEY_PRESSED_OFFSET_Q)    != 0 )
#define    IF_KEY_PRESSED_E       ( (remote_control.keyboard_value & KEY_PRESSED_OFFSET_E)    != 0 )
#define    IF_KEY_PRESSED_G       ( (remote_control.keyboard_value & KEY_PRESSED_OFFSET_G)    != 0 )
#define    IF_KEY_PRESSED_X       ( (remote_control.keyboard_value & KEY_PRESSED_OFFSET_X)    != 0 )
#define    IF_KEY_PRESSED_Z       ( (remote_control.keyboard_value & KEY_PRESSED_OFFSET_Z)    != 0 )
#define    IF_KEY_PRESSED_C       ( (remote_control.keyboard_value & KEY_PRESSED_OFFSET_C)    != 0 )
#define    IF_KEY_PRESSED_B       ( (remote_control.keyboard_value & KEY_PRESSED_OFFSET_B)    != 0 )
#define    IF_KEY_PRESSED_V       ( (remote_control.keyboard_value & KEY_PRESSED_OFFSET_V)    != 0 )
#define    IF_KEY_PRESSED_F       ( (remote_control.keyboard_value & KEY_PRESSED_OFFSET_F)    != 0 )
#define    IF_KEY_PRESSED_R       ( (remote_control.keyboard_value & KEY_PRESSED_OFFSET_R)    != 0 )
#define    IF_KEY_PRESSED_CTRL    ( (remote_control.keyboard_value & KEY_PRESSED_OFFSET_CTRL) != 0 )
#define    IF_KEY_PRESSED_SHIFT   ( (remote_control.keyboard_value & KEY_PRESSED_OFFSET_SHIFT) != 0 )

/* 获取鼠标三轴的移动速度 */
#define    MOUSE_X_MOVE_SPEED    (remote_control.mouse_x)
#define    MOUSE_Y_MOVE_SPEED    (remote_control.mouse_y)
#define    MOUSE_Z_MOVE_SPEED    (remote_control.mouse_z)

/* 检测鼠标按键状态 
   按下为1，没按下为0*/
#define    IF_MOUSE_PRESSED_LEFT    (remote_control.mouse.press_l == 1)
#define    IF_MOUSE_PRESSED_RIGH    (remote_control.press_r == 1)

// 拨钮状态宏定义
#define UP 1
#define MIDDLE 3
#define DOWN 2

#pragma pack(1)
typedef struct
{
    /*遥控器数据*/
    struct
    {
        int16_t ch[5]; // 摇杆数据 +_660
        char s[2];     // 拨杆数据
    } rc;
    /*鼠标数据*/
    struct
    {
        int16_t x;       // x轴移动速度
        int16_t y;       // y轴移动速度
        int16_t z;       // z轴移动速度
        uint8_t press_l; // 左键是否按下
        uint8_t press_r; // 右键是否按下
    } mouse;
    /*键盘数据*/
    struct
    {
        uint16_t v; // 键盘按键数据
    } key;
}DT7_data_s;

#pragma pack(4)
typedef struct
{
	uint16_t CH1;//通道1数值
	uint16_t CH2;//通道2数值
	uint16_t CH3;//通道3数值
	uint16_t CH4;//通道4数值
	uint16_t CH5;//通道5数值
	uint16_t CH6;//通道6数值
	uint16_t CH7;//通道7数值
	uint16_t CH8;//通道8数值
	uint16_t CH9;//通道9数值
	uint16_t CH10;//通道10数值
	uint16_t CH11;//通道11数值
	uint16_t CH12;//通道12数值
	uint16_t CH13;//通道13数值
	uint16_t CH14;//通道14数值
	uint16_t CH15;//通道15数值
	uint16_t CH16;//通道16数值
	uint8_t ConnectState;//遥控器与接收器连接状态 0=未连接，1=正常连接
}SBUS_CH_Struct;

extern SBUS_CH_Struct SBUS_CH;
extern DT7_data_s DT7_data;

void DT7_decode_data(uint8_t *data);
void update_sbus(volatile const uint8_t *sbus_buf,SBUS_CH_Struct *SBUS_CH);
#endif // !__DT&_H__
