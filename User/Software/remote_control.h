#ifndef __REMOTE_CONTROL__
#define __REMOTE_CONTROL__

#include "stdint.h"
#include "main.h"

#define    WAIT_W 0
#define    WAIT_S 1
#define    WAIT_A 2
#define    WAIT_D 3
#define    WAIT_Q 4
#define    WAIT_E 5
#define    WAIT_G 6
#define    WAIT_X 7
#define    WAIT_Z 8
#define    WAIT_C 9
#define    WAIT_B 10
#define    WAIT_V 11
#define    WAIT_F 12
#define    WAIT_R 13
#define    WAIT_CTRL 14
#define    WAIT_SHIFT 15

#define SIZE_OF_WAIT 20 //消抖数组大小，要大于上面那些数


typedef PACKED_STRUCT()
{
    struct
    {
        uint16_t ch[5]; // 摇杆数据
        int16_t s[9];     // 拨杆数据
    } rc;
    struct
    {
        int16_t x;       // x轴移动速度
        int16_t y;       // y轴移动速度
        int16_t z;       // z轴移动速度
        uint8_t press_l; // 左键是否按下
        uint8_t press_r; // 右键是否按下
        uint8_t press_mid;
    } mouse;
    struct
    {
        uint16_t v; // 键盘按键数据
    } key;
    struct
    {
        uint8_t custom_data[30]; // 自定义控制器数据
    } custom_robot;
} RC_ctrl_t;
 extern  RC_ctrl_t RC_data;
 typedef enum ARM_CONNECT_STATUS
 {
    ARM_CONNECT_STATUS_DISCONNECTED =0,
    ARM_CONNECT_STATUS_CONNECTED,
   
   
 } ARM_CONNECT_STATUS;
 extern ARM_CONNECT_STATUS arm_connect_status;
typedef enum BOOM_ARM_Stats{//机械臂气泵 
     ARM_BOOM_ON = 0,//机械臂气泵开 
     ARM_BOOM_OFF,//机械臂气泵关 

 }BOOM_ARM_Stats;
extern BOOM_ARM_Stats boom_arm_status; 

typedef enum BOOM_STORAGE_Stats{ //储矿气泵 
 STORAGE_OFF=0,//储矿气泵全关 
 STORAGE_ON,//储矿气泵全开 
 STORAGE1_ON,//储矿1气泵开，2关 
 STORAGE2_ON,//储矿2气泵开，1关 ? ? 
}BOOM_STORAGE_Stats;
extern BOOM_STORAGE_Stats boom_storage_status;
extern float total_angle;
extern int sum_arm;
/*外部函数调用*/
void Motor_Forward(void);
void Motor_Reverse(void);
void Motor_Stop(void);





void remote_control_init(void);
void key_mouse_control(void);
void RC_Control(void);
void ARM_CONNECT_STATUS_UPDATA(void);
void start(void);
void disable_all(void);
void YAWFLOW_POS(void);
void enable_all(void);
void GPIO_init(void);
void PWM_control_init(void);
void switch_servos(void);
#endif // !__REMOTE_CONTROL__
