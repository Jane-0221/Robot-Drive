#include "stdint.h"

typedef struct
{
    /*底盘状态*/
    struct
    {
        enum chassis_mode_e
        {
            FLOW,   // 底盘跟随
            SPIN_P, // 正小陀螺
            SPIN_N, // 逆小陀螺
        } mode;

        struct
        {
            float x; // x轴移动速度
            float y; // y轴移动速度
            float r; // 转速 顺时针正，逆时针负
            uint8_t reset;
        } input;
    } Chssis;
    /*云台状态*/
    struct
    {
        enum gimbal_mode_e
        {
            NORMAL, // 普通模式
            AUTO,   // 自瞄模式
        } mode;
        struct
        {
            float pitch;
            float yaw;
        } input;

    } Gimbal;
    /*发射机构状态*/
    struct
    {
        enum tigger_mode_e
        {
            TRIGGER_CLOSE,
            HIGH,
            MID,
            LOW,
            SINGLE,
            DEBUG_TRIGGER, // 退弹使用，低射速高射频。
        } tigger_mode;
        enum shoot_mode_e
        {
            CLOSE,
            READY,
            DEBUG_SHOOT, // 退弹使用
        } shoot_mode;
        enum shoot_status_e
        {
            NOK,
            OK,
        } shoot_status;
    } Shoot;
    /*自瞄状态*/
    struct
    {
        enum auto_mode_e
        {
            NONE,       // 不启用自瞄
            CAR,        // 车
            OUTPOST,    // 前哨站
            LOWTARGET,  // 小符
            HIGHTARGET, // 大符
        } mode;
        struct
        {
            float shoot_yaw;
            float shoot_pitch;
            int Auto_control_online; // 自瞄在线
            uint8_t fire;                // 是否启用自瞄
            uint8_t target_id;           // 目标ID
        } input;
    } Auto;
    /*电容状态*/
    struct
    {
        enum cap_mode_e
        {
            Not_FULL, // 暂时不知道放啥
            FULL,
        } mode;
    } Cap;
    /**/
    struct
    {
        enum control_mode_e
        {
            RC,   // 遥控器
            KEY,  // 键鼠
            LOCK, // 锁死
        } mode;
    } Control;
    
    enum speed_mode_e
    {
        speed_normal = 0,
        speed_accel,			
    } speed_mode;	

} GlobalStatus_t;

extern GlobalStatus_t Global;