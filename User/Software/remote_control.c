#include "remote_control.h"
#include "ramp_generator.h"
#include "IMU_updata.h"
#include "Stm32_time.h"
#include <tim.h>
#include "pid.h"
#include "UART_data_txrx.h"

int b = 1350;
int c = 0;
RC_ctrl_t RC_data;
// 云台跟随陀螺仪pid
float AHRS_yaw_set = 0;
pid_t yaw_pid;
float delta_angle;
int16_t wait_time[SIZE_OF_WAIT] = {0}; // 键盘消抖用时间
ARM_CONNECT_STATUS arm_connect_status;
BOOM_ARM_Stats boom_arm_status;         // 机械臂气泵状态
BOOM_STORAGE_Stats boom_storage_status; // 储矿气泵状态
#define MOVE_SENSITIVITY 10.0f          // 移动灵敏度，
#define PITCH_SENSITIVITY 0.008f        // pitch轴灵敏度
#define YAW_SENSITIVITY 0.005f          // yaw轴灵敏度
                                        // DAMIAO角度读值
float total_angle = 0.0;                // 总累积角度
float prev_angle = 0.0;                 // 前一次读取的角度
float current_angle = 0.0;
float angle_range = 360.0; // 编码器量程
int sum_arm = 0;

/**
 * @brief 统一消抖
 *
 * @param key 按键宏
 * @return uint8_t 0未到时间，1到时间
 */
uint8_t Wait(uint8_t key)
{

}
void remote_control_init()
{
    GPIO_init();
    PWM_control_init();
    boom_storage_status = STORAGE_OFF; // 气泵
    boom_arm_status = ARM_BOOM_OFF;    // 机械臂
}

void GPIO_init() // 电子开关初始化
{
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET); // PD14//气泵1

    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET); // PD15//电磁阀2

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET); // PB8

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET); // PB9

    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, GPIO_PIN_RESET); // PE14

    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_RESET); // PE0，左储矿

    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_RESET); // PE1

    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET); // PC10 气泵

    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_RESET); // PC11，右储矿
}

void PWM_control_init() // 舵机云台初始化
{

}

void VT13toRCdata()
{


}
void key_mouse_control() // 键盘鼠标模式+自定义控制器
{
}
void ARM_CONNECT_STATUS_UPDATA()
{
 

    
}
//////////遥控器控制////////////////
void RC_Control(void)
{

    
}
///////////////////////////启动函数///////////////////////////
void start()
{
    
    // }
}

void YAWFLOW_POS()
{

}

//////////////////////全部失能，防止疯车//////////////////////////////
void disable_all()
{

   
}


void enable_all()
{


}

void switch_servos()
{


}