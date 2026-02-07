#include "remote_control.h"
#include "Global_status.h"
#include "Chassis.h"
#include "Gimbal.h"
#include "ui.h"
#include "ramp_generator.h"
#include "limit_filter.h"
#include "IMU_updata.h"
#include "referee_system.h"
#include "DT7.h"
#include "dm_arm.h"
#include "Stm32_time.h"
#include <tim.h>
#include "pid.h"
#include "VT13.h"
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
    if (wait_time[key] > 0)
    {
        wait_time[key]--;
        return 0;
    }

    else
    {
        wait_time[key] = 180;
        return 1;
    }
}
void remote_control_init()
{

    pid_set(&yaw_pid, 2.0f, 0.0f, 350.0f, 5.0f, 2.0f); // 3.0.5

    GPIO_init();
    PWM_control_init();
    limit_filter_init(&custom_robot_data);
    init_smoothed_data_init();
    arm_connect_status = ARM_CONNECT_STATUS_DISCONNECTED;
    boom_storage_status = STORAGE_OFF; // 气泵
    boom_arm_status = ARM_BOOM_OFF;    // 机械臂
                                       //__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, 200);
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
    HAL_TIM_Base_Start(&htim1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2); // 蜂鸣器PWM
}

void VT13toRCdata()
{

    arm_connect_status = ARM_CONNECT_STATUS_DISCONNECTED;
    Chassis_set_x(VT13_data.rc.ch[0] / 220.0f); /// 这个可能是遥控器的遥杆上下
    Chassis_set_y(VT13_data.rc.ch[1] / 220.0f); /// 遥杆左右
                                                // if (VT13_data.rc.ch[3] != 0)
                                                // {
                                                //     AHRS_yaw_set = (IMU_data.AHRS.yaw - (VT13_data.rc.ch[3] / 600.0f));
                                                // }
    Chassis_set_r(VT13_data.rc.ch[3] / 600.0f); /// 遥杆左右
}
void key_mouse_control() // 键盘鼠标模式+自定义控制器
{
    /////////////////////////映射控制底盘移动///////////////////////////////////////
    if (arm_connect_status == ARM_CONNECT_STATUS_CONNECTED)
    {
        if (IF_KEY_PRESSED_W) // W
        {
            if (Global.Chssis.input.y < 1.0f)
                Global.Chssis.input.y += 0.05f;
            else
                Global.Chssis.input.y = 1.0f;
        }

        else if (IF_KEY_PRESSED_S) // S
        {
            if (Global.Chssis.input.y > -1.0f)
                Global.Chssis.input.y -= 0.05f;
            else
                Global.Chssis.input.y = -1.0f;
        }
        else
            Global.Chssis.input.y = 0;

        if (IF_KEY_PRESSED_A) // A
        {
            // if (Global.Chssis.input.x > -1.0f)
            //     Global.Chssis.input.x -= 0.05f;
            // else
            //     Global.Chssis.input.x = -1.0f;
            Global.Chssis.input.x = -1.0f;
        }

        else if (IF_KEY_PRESSED_D) // D
        {
            // if (Global.Chssis.input.x < 1.0f)
            //     Global.Chssis.input.x += 0.05f;
            // else
            //     Global.Chssis.input.x = 1.0f;
            Global.Chssis.input.x = 1.0f;
        }
        else
            Global.Chssis.input.x = 0;
    }
    ////////////不映射的底盘移动///////////////////
    if (arm_connect_status == ARM_CONNECT_STATUS_DISCONNECTED)
    {
        if (IF_KEY_PRESSED_W && !(IF_KEY_PRESSED_W && IF_KEY_PRESSED_SHIFT)) // W
        {
            if (Global.Chssis.input.y < 2.0f)
                Global.Chssis.input.y += 0.003f; // 0.01
            else
                Global.Chssis.input.y = 2.0f;
        }
        else if (IF_KEY_PRESSED_W && IF_KEY_PRESSED_SHIFT) // shift+W
        {
            if (Global.Chssis.input.y < 4.0f)
                Global.Chssis.input.y += 0.006f; // 0.01
            else
                Global.Chssis.input.y = 4.0f;
        }

        else if (IF_KEY_PRESSED_S && !(IF_KEY_PRESSED_S && IF_KEY_PRESSED_SHIFT)) // S
        {
            if (Global.Chssis.input.y > -2.0f)
                Global.Chssis.input.y -= 0.003f;
            else
                Global.Chssis.input.y = -2.0f;
        }
        else if (IF_KEY_PRESSED_S && IF_KEY_PRESSED_SHIFT) // shift+s
        {
            if (Global.Chssis.input.y > -4.0f)
                Global.Chssis.input.y -= 0.006f; // 0.01
            else
                Global.Chssis.input.y = -4.0f;
        }
        else
            Global.Chssis.input.y = 0;
        /////////////////////////////////////////////////////////////////////////////////
        if (IF_KEY_PRESSED_A) // A
        {
            if (Global.Chssis.input.x > -3.0f && Global.Chssis.input.x < -0.5f)
                Global.Chssis.input.x -= 0.002f;
            else
                Global.Chssis.input.x -= 0.01;

            if (Global.Chssis.input.x < -3.0f)
                Global.Chssis.input.x = -3.0f;
        }

        else if (IF_KEY_PRESSED_D) // D
        {
            if (Global.Chssis.input.x < 3.0f && Global.Chssis.input.x > 0.5f)
                Global.Chssis.input.x += 0.002f;
            else
                Global.Chssis.input.x += 0.01f;

            if (Global.Chssis.input.x > 3.0f)
                Global.Chssis.input.x = 3.0f;
        }
        else
            Global.Chssis.input.x = 0;
    }

    ///////////////////////////键位控制机械臂////////////////////////////////////////////////////
    // 一键收起机械臂移动，自定义控制器不映射
    if (IF_KEY_PRESSED_CTRL && IF_KEY_PRESSED_Z)
    { // ctrl+z
        if (arm_connect_status == ARM_CONNECT_STATUS_CONNECTED)
        {
            AHRS_yaw_set = (IMU_data.AHRS.yaw - 0.5);
        }
        if (arm_connect_status == ARM_CONNECT_STATUS_CONNECTED)
            sum_arm = 0;

        arm_connect_status = ARM_CONNECT_STATUS_DISCONNECTED;
        b = 400;
        __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, 400);
    }
    // 自定义控制器映射
    if (IF_KEY_PRESSED_CTRL && IF_KEY_PRESSED_X)
    { // ctrl+X
        if (arm_connect_status == ARM_CONNECT_STATUS_DISCONNECTED)
            sum_arm = 0;
        boom_arm_status = ARM_BOOM_ON;
        b = 1300;
        __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, 1300);
        arm_connect_status = ARM_CONNECT_STATUS_CONNECTED;
    }

    if (IF_KEY_PRESSED_V)
    {
        if (b >= 1800)
        {
            b = 1800;
        }
        else
            b += 1;
        __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, b);
    }
    if (IF_KEY_PRESSED_B)
    {
        if (b <= 400)
        {
            b = 400;
        }
        else
            b -= 1;
        __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, b);
    }

    if (IF_KEY_PRESSED_Z) // z
    {
        ui_init();
    }
    // // 自定义控制器映射
    // if (IF_KEY_PRESSED_CTRL && IF_KEY_PRESSED_R)
    // { // ctrl+X

    //     disable_all();
    // }
    // if (IF_KEY_PRESSED_CTRL && IF_KEY_PRESSED_C)
    // { // ctrl+X

    //     // enable_all();]
    //     CAN_Send_Save_Zer"]777777]]]]]]]]]]]]]]]]]]]]]]]]]7]]]]]]]]]]]]]]]]]]]]]]]]]]]"o(&hfdcan2, 0x04);
    //     pos_motor.pos_middlerow = 0.0f;
    //     CAN_Send_Save_Zero(&hfdcan2, 0x05);
    //     pos_motor.pos_endyaw = 0.0f;
    // }

    // if (IF_KEY_PRESSED_CTRL && IF_KEY_PRESSED_C)
    // { // 一键储矿 1
    //     boom_arm_status = ARM_BOOM_ON;
    //     boom_storage_status = STORAGE_ON;
    //     // 机械臂到达指定位置1的姿态.....
    // }

    // if (IF_KEY_PRESSED_CTRL && IF_KEY_PRESSED_V)
    // { // 一键储矿2
    //     boom_arm_status = ARM_BOOM_ON;
    //     boom_storage_status = STORAGE_ON;
    //     // 机械臂到达指定位置2的姿态.....
    // }

    // if (IF_KEY_PRESSED_SHIFT && IF_KEY_PRESSED_C)
    // { // 兑矿 1

    //     boom_storage_status = STORAGE2_ON;
    //     // 机械臂到达指定位置1.........
    // }

    // if (IF_KEY_PRESSED_SHIFT && IF_KEY_PRESSED_V)
    // { // 兑矿2
    //     boom_storage_status = STORAGE1_ON;
    //     // 机械臂到达指定位置2........
    // }

    if (IF_KEY_PRESSED_CTRL && IF_KEY_PRESSED_G)
    { // 机械臂气泵关
        boom_arm_status = ARM_BOOM_OFF;
    }

    if (IF_KEY_PRESSED_CTRL && IF_KEY_PRESSED_F)
    { // 机械臂气泵开
        boom_arm_status = ARM_BOOM_ON;
    }

    // if (IF_KEY_PRESSED_CTRL && IF_KEY_PRESSED_B)
    // { // 储矿全关
    //     boom_storage_status = STORAGE_OFF;
    // }

    // if (IF_KEY_PRESSED_CTRL && IF_KEY_PRESSED_V)
    // { // 储矿全开
    //     boom_storage_status = STORAGE_ON;
    // }

    // 机械臂气泵
    if (boom_arm_status == ARM_BOOM_OFF)
    {
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET); // PD14,气泵1关闭
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET); // PD15，电磁阀1关掉
    }
    if (boom_arm_status == ARM_BOOM_ON)
    {
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET); // PD14,气泵1打开
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET); // PD15，电磁阀1打开
    }

    // 储矿气泵
    if (boom_storage_status == STORAGE_OFF)
    {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET); // PC10,气泵2关闭
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_RESET);  // PE0，电磁阀2关闭
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_RESET); // PC11 ，电磁阀3关闭
    }
    if (boom_storage_status == STORAGE_ON)
    {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_SET); // PC10,气泵2打开
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_SET);  // PE0，电磁阀2打开
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_SET); // PC11 ，电磁阀3打开
    }
    if (boom_storage_status == STORAGE1_ON)
    {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_SET);   // PC10,气泵2打开
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_SET);    // PE0，储矿1打开，电磁阀2打开
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_RESET); // PC11 ，储矿2关闭，电磁阀3关闭
    }
    if (boom_storage_status == STORAGE2_ON)
    {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_SET);  // PC10,气泵2打开
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_RESET); // PE0，储矿1关闭，电磁阀2关闭
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_SET);  // PC11 ，储矿2打开，电磁阀3打开
    }
}
void ARM_CONNECT_STATUS_UPDATA()
{
    switch (arm_connect_status)
    {
    case ARM_CONNECT_STATUS_DISCONNECTED:
        pos_motor.pos_rootyaw = -0.618f;
        pos_motor.pos_rootpitch = -0.001f;
        pos_motor.pos_middleyaw = 2.07f;
        pos_motor.pos_middlerow = 0.23f;
        pos_motor.pos_endyaw = 1.7f;

        current_angle = custom_robot_data.data_5;
        // 计算差值
        delta_angle = current_angle - prev_angle;
        // 处理回绕情况
        if ((delta_angle > (angle_range / 2.0)))
        {
            delta_angle -= angle_range;
        }
        else if ((delta_angle < ((-angle_range) / 2.0)))
        {
            delta_angle += angle_range;
        }
        // 累积总角度
        total_angle += delta_angle;

        // 更新前一次角度
        prev_angle = current_angle;

        pos_motor.pos_row = total_angle;

        if (((AHRS_yaw_set - IMU_data.AHRS.yaw) > PI) || ((AHRS_yaw_set - IMU_data.AHRS.yaw) < (-PI)))
        {
            AHRS_yaw_set = IMU_data.AHRS.yaw;
        }
        // pid计算
        Global.Chssis.input.r = -(pid_cal(&yaw_pid, IMU_data.AHRS.yaw, AHRS_yaw_set) * 1.1);

        if (IF_KEY_PRESSED_Q) // Q
        {
            AHRS_yaw_set = (IMU_data.AHRS.yaw + 0.2f);
        }

        if (IF_KEY_PRESSED_E) // E
        {
            AHRS_yaw_set = (IMU_data.AHRS.yaw - 0.2f);
        }

        if (remote_control.mouse_x > 30.0f || remote_control.mouse_x < -30.0f)
        {
            AHRS_yaw_set = (IMU_data.AHRS.yaw - (remote_control.mouse_x / 700.0f));
            // Global.Chssis.input.r = (remote_control.mouse_x / 300.0f);
        }

        break;

    case ARM_CONNECT_STATUS_CONNECTED:
        // 斜坡函数后
        pos_motor.pos_rootyaw = -smoothed_data.data_0;
        pos_motor.pos_rootpitch = smoothed_data.data_1;
        pos_motor.pos_middleyaw = -smoothed_data.data_2;
        pos_motor.pos_middlerow = smoothed_data.data_3;
        pos_motor.pos_endyaw = smoothed_data.data_4;

        current_angle = custom_robot_data.data_5;
        // 计算差值
        delta_angle = current_angle - prev_angle;
        // 处理回绕情况
        if ((delta_angle > (angle_range / 2.0)))
        {
            delta_angle -= angle_range;
        }
        else if ((delta_angle < ((-angle_range) / 2.0)))
        {
            delta_angle += angle_range;
        }
        // 累积总角度
        total_angle += delta_angle;

        // 更新前一次角度
        prev_angle = current_angle;

        pos_motor.pos_row = total_angle;

        // Global.Chssis.input.r = (remote_control.mouse_x / 300.0f);
        if (IF_KEY_PRESSED_Q) // Q
        {
            Global.Chssis.input.r = (-0.5f);
        }
        else if (IF_KEY_PRESSED_E) // E
        {
            Global.Chssis.input.r = 0.5f;
        }
        else
            Global.Chssis.input.r = 0.0f;
        AHRS_yaw_set = IMU_data.AHRS.yaw;

        break;

    default:
        break;
    }
}
//////////遥控器控制////////////////
void RC_Control(void)
{

    //////////////*底盘控制*///////////////////////
    Global.Chssis.mode = FLOW;
    if (SBUS_CH.CH6 == 321 && SBUS_CH.CH7 == 321 && SBUS_CH.CH8 == 321)
    { // 底盘
        Chassis_set_x((SBUS_CH.CH1 - 992) / 40.0f);
        Chassis_set_y(-(SBUS_CH.CH2 - 992) / 40.0f);
        if ((SBUS_CH.CH4 != 992))
        {
            // AHRS_yaw_set = IMU_data.AHRS.yaw - (SBUS_CH.CH4 - 992) / 7000.0f;
            AHRS_yaw_set = IMU_data.AHRS.yaw - (SBUS_CH.CH4 - 992) / 700.0f;
        }
        if (((AHRS_yaw_set - IMU_data.AHRS.yaw) > PI) || ((AHRS_yaw_set - IMU_data.AHRS.yaw) < (-PI)))
        {
            AHRS_yaw_set = IMU_data.AHRS.yaw;
        }
        // pid计算
        Global.Chssis.input.r = -(pid_cal(&yaw_pid, IMU_data.AHRS.yaw, AHRS_yaw_set));
    }
    // 机械臂
    //  机械臂1
    if (SBUS_CH.CH5 == 321 && SBUS_CH.CH6 == 992 && SBUS_CH.CH7 == 992 && SBUS_CH.CH8 == 321)
    {
        // 大YAW
        if (SBUS_CH.CH4 > 1050)
        {
            pos_motor.pos_rootyaw -= 0.0005;
        }
        else if (SBUS_CH.CH4 < 950)
        {
            pos_motor.pos_rootyaw += 0.0005;
        }
        else if (SBUS_CH.CH4 > 950 && SBUS_CH.CH4 < 1050)
        {
            pos_motor.pos_rootyaw += 0;
        }

        // 大PITCH
        if (SBUS_CH.CH3 > 1050)
        {
            pos_motor.pos_rootpitch -= 0.0005;
        }
        else if (SBUS_CH.CH3 < 950)
        {
            pos_motor.pos_rootpitch += 0.0005;
        }
        else if (SBUS_CH.CH1 < 950 && SBUS_CH.CH1 < 1050)
        {
            pos_motor.pos_rootpitch += 0.0;
        }

        // 中yaw
        if (SBUS_CH.CH1 > 1050)
        {
            pos_motor.pos_middleyaw -= 0.0005;
        }
        else if (SBUS_CH.CH1 < 950)
        {
            pos_motor.pos_middleyaw += 0.0005;
        }
        else if (SBUS_CH.CH1 > 950 && SBUS_CH.CH1 < 1050)
        {
            pos_motor.pos_middleyaw += 0;
        }

        // 小yaw(末端pitch)
        if (SBUS_CH.CH2 > 1050)
        {

            pos_motor.pos_endyaw += 0.0005;
            // SBUS_POS*(SBUS_CH.CH1-1050)
        }
        else if (SBUS_CH.CH2 < 950)
        {
            pos_motor.pos_endyaw -= 0.0005;
            // SBUS_POS*(SBUS_CH.CH1-950)
        }
        else if (SBUS_CH.CH2 > 950 && SBUS_CH.CH2 < 1050)
        {
            pos_motor.pos_endyaw += 0;
        }
    }
    // 机械臂2
    if (SBUS_CH.CH5 == 321 && SBUS_CH.CH6 == 1663 && SBUS_CH.CH7 == 1663 && SBUS_CH.CH8 == 321)
    {
        // 中row
        if (SBUS_CH.CH1 > 1050)
        {
            pos_motor.pos_middlerow += 0.0005;
        }
        else if (SBUS_CH.CH1 < 950)
        {
            pos_motor.pos_middlerow -= 0.0005;
        }
        else if (SBUS_CH.CH1 > 950 && SBUS_CH.CH1 < 1050)
        {
            pos_motor.pos_middlerow += 0;
        }
        // 小row
        if (SBUS_CH.CH4 > 1050)
        {
            endrow_motor.set += 0.0001;
        }
        else if (SBUS_CH.CH4 < 950)
        {
            endrow_motor.set -= 0.0001;
        }
        else if (SBUS_CH.CH4 > 950 && SBUS_CH.CH4 < 1050)
        {
            endrow_motor.set += 0;
        }
    }
    // 一键收起机械臂移动
    if (SBUS_CH.CH5 == 1663 && SBUS_CH.CH6 == 321 && SBUS_CH.CH7 == 321 && SBUS_CH.CH8 == 321)
    {
        pos_motor.pos_rootyaw = -0.5255f;
        pos_motor.pos_rootpitch = 0.1675f;
        pos_motor.pos_middleyaw = 1.988f;
        pos_motor.pos_middlerow = 0.0f;
        pos_motor.pos_endyaw = 0.217f;
    }
    // 保存0点，慎用
    if (SBUS_CH.CH12 == 1663)
    {
        // CAN_Send_Save_Zero(&hfdcan1,0x05);
        // pos_motor.pos_rootyaw = 0.0f;

        // CAN_Send_Save_Zero(&hfdcan2,0x02);
        // pos_motor.pos_rootpitch = 0.0f;

        // CAN_Send_Save_Zero(&hfdcan2,0x03);
        // pos_motor.pos_middleyaw=0.0f;

        // CAN_Send_Save_Zero(&hfdcan2,0x04);
        // pos_motor.pos_middlerow=0.0f;
        // CAN_Send_Save_Zero(&hfdcan2,0x05);
        // pos_motor.pos_endyaw=0.0f;
    }
    if (SBUS_CH.CH11 == 321)
    { // 机械臂气泵关
        boom_arm_status = ARM_BOOM_OFF;
    }

    if (SBUS_CH.CH11 == 1663)
    { // 机械臂气泵开
        boom_arm_status = ARM_BOOM_ON;
    }
    // 机械臂气泵
    if (boom_arm_status == ARM_BOOM_OFF)
    {
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET); // PD14,气泵1关闭
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET); // PD15，电磁阀1关掉
    }
    if (boom_arm_status == ARM_BOOM_ON)
    {
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET); // PD14,气泵1打开
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET); // PD15，电磁阀1打开
    }
}
///////////////////////////启动函数///////////////////////////
void start()
{
    Flag_T13++;
    if (Flag_T13 > 100)
    {
        VT13_data.rc.mode_sw = 0;
    }
    switch (VT13_data.rc.mode_sw)
    {
    case 2:
        key_mouse_control(); // 键鼠控制
        ARM_CONNECT_STATUS_UPDATA();
        break;
    case 1:
        VT13toRCdata();
        break;
    case 0:
        RC_Control(); // 遥控器控制
        break;
    default:
        // mode_sw 不为 0、1、2 时执行
        break;
    }
    if (SBUS_CH.CH5 == 1663 && SBUS_CH.CH6 == 1663 && SBUS_CH.CH7 == 1663 && SBUS_CH.CH8 == 1663)
    {
        disable_all(); // 失能
    }
    if (SBUS_CH.CH5 == 1663 && SBUS_CH.CH6 == 1663 && SBUS_CH.CH7 == 1663 && SBUS_CH.CH8 == 321)
    {
        enable_all(); // 使能
    }
}

void YAWFLOW_POS()
{
    float x = 0;
    if (((AHRS_yaw_set - IMU_data.AHRS.yaw) > PI) || ((AHRS_yaw_set - IMU_data.AHRS.yaw) < (-PI)))
    {
        AHRS_yaw_set = IMU_data.AHRS.yaw;
    }

    current_angle = arm_motor[Motor1].para.pos;
    // 计算差值
    float delta_angle = current_angle - prev_angle;
    // 处理回绕情况
    if (delta_angle > angle_range / 2.0)
    {
        delta_angle -= angle_range;
    }
    else if (delta_angle < -angle_range / 2.0)
    {
        delta_angle += angle_range;
    }
    // 累积总角度
    total_angle += delta_angle;

    // 更新前一次角度
    prev_angle = current_angle;

    pos_motor.pos_rootyaw = total_angle + /*(1+2*arm_motor[Motor1].para.vel)**/ (pid_cal(&yaw_pid, IMU_data.AHRS.yaw, AHRS_yaw_set));

    if (SBUS_CH.CH5 == 321 && SBUS_CH.CH6 == 321 && SBUS_CH.CH7 == 321 && SBUS_CH.CH8 == 321)
    {
        Global.Chssis.mode = FLOW;
        return;
    }

    if (SBUS_CH.CH5 == 992 && SBUS_CH.CH6 == 321 && SBUS_CH.CH7 == 321 && SBUS_CH.CH8 == 321)
    {
        Global.Chssis.mode = SPIN_P;
        return;
    }
    if (SBUS_CH.CH5 == 1663 && SBUS_CH.CH6 == 321 && SBUS_CH.CH7 == 321 && SBUS_CH.CH8 == 321)
    {
        Global.Chssis.mode = SPIN_N;
        return;
    }
}

//////////////////////全部失能，防止疯车//////////////////////////////
void disable_all()
{

    // 机械臂失能
    CAN_Send_Exit(&hfdcan1, 0x05);
    osDelay(1);
    CAN_Send_Exit(&hfdcan2, 0x05);
    osDelay(1);
    CAN_Send_Exit(&hfdcan2, 0x04);
    osDelay(1);
    CAN_Send_Exit(&hfdcan2, 0x03);
    osDelay(1);
    CAN_Send_Exit(&hfdcan2, 0x02);
    osDelay(1);

    endrow_motor.set = endrow_motor.now;
    // 底盘失能
    Chassis_set_x(0.0f);
    Chassis_set_y(0.0f);
    Chassis_set_r(0.0f);
    return;
}
/////////////////////// 舵机控制云台///////////////////////
/////////////////////// 使能函数///////////////////////

void enable_all()
{

    // 机械臂失能
    CAN_Send_Enter(&hfdcan1, 0x05);
    osDelay(1);
    CAN_Send_Enter(&hfdcan2, 0x05);
    osDelay(1);
    CAN_Send_Enter(&hfdcan2, 0x04);
    osDelay(1);
    CAN_Send_Enter(&hfdcan2, 0x03);
    osDelay(1);
    CAN_Send_Enter(&hfdcan2, 0x02);
    osDelay(1);
    return;
}
// 舵机云台
void switch_servos()
{

    // 具体根据需求确定遥控器通道
    //     if (SBUS_CH.CH4 > 992)
    //     {
    //         __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, b = b + 1); // 通道3云台1
    //     }
    //     if (SBUS_CH.CH4 < 992)
    //     {
    //         __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, b = b - 1);
    //     }
    //     if (b >= 2100)
    //     { // 给个限位，防止跑飞
    //         b = 2100;
    //     }
    //     if (b <= 400)
    //     {
    //         b = 400;
    //     }

    //  // 具体根据需求确定遥控器通道
    //     if (SBUS_CH.CH3 > 992)
    //     {
    //         __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, c = c + 1); // 通道1云台2
    //     }
    //     if (SBUS_CH.CH3 < 992)
    //     {
    //         __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, c = c - 1);
    //     }

    // if (c >= 2100)
    // { // 给个限位，防止跑飞
    //     c = 2100;
    // }
    // if (c <= 400)
    // {
    //     c = 400;
    // }
}