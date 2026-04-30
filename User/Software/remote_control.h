#ifndef __REMOTE_CONTROL__
#define __REMOTE_CONTROL__

#include "stdint.h"
#include "main.h"

/**
 * @brief 遥控控制模块初始化。
 *
 * 当前实现为空函数，保留该接口用于后续初始化遥控相关状态或外设。
 */
void remote_control_init(void);

/**
 * @brief 根据 SBUS 遥控通道更新气泵状态。
 *
 * CH8 高位打开气泵，CH8 低位关闭气泵。
 */
extern void Pump_Control_Updata(void);

/**
 * @brief 根据 SBUS 遥控通道更新头部电机和舵机目标角度。
 *
 * CH8 用于切换手动微调和预设姿态模式，CH6 选择当前控制档位或预设姿态。
 */
extern void Head_Motor_Control_Updata(void);

/**
 * @brief 根据 SBUS 遥控通道微调机械臂电机目标角度。
 *
 * CH8 低位时启用机械臂电机遥控；CH7 选择电机组，CH1/CH2 调整对应关节角度。
 */
extern void Arm_Motor_Control_Updata(void);

/**
 * @brief 检查遥控失能组合键并在首次触发时关闭机械臂电机。
 *
 * 当 CH5、CH6、CH7、CH8 同时处于高位时触发失能。该函数带锁存逻辑，
 * 同一次持续触发期间只发送一次失能指令。
 *
 * @return uint8_t 1 表示当前失能组合键有效，0 表示未触发。
 */
uint8_t Arm_Motor_Disable_Updata(void);

/**
 * @brief 获取机械臂失能组合键当前是否处于有效状态。
 *
 * @return uint8_t 1 表示失能组合键当前有效，0 表示无效。
 */
uint8_t Arm_Motor_Disable_IsActive(void);

/**
 * @brief 根据 SBUS 遥控通道更新升降机构目标高度。
 *
 * CH7 高、中、低三档分别对应不同的升降目标高度。
 */
void Up_Down_Motor_Control_Updata(void);

/**
 * @brief 根据 SBUS 遥控通道更新底盘速度指令。
 *
 * 在指定拨杆组合下读取 CH1/CH2/CH3，转换为横移、前后和旋转速度。
 */
extern void Chassis_Control_Updata(void);

/* PC 控制接口声明 */

/**
 * @brief 根据 PC 下发数据更新气泵状态。
 */
extern void PC_Pump_Control_Updata(void);

/**
 * @brief 根据 PC 下发数据更新头部电机目标角度。
 */
extern void PC_Head_Motor_Control_Updata(void);

/**
 * @brief 根据 PC 下发数据更新升降机构目标高度。
 */
extern void PC_Up_Down_Motor_Control_Updata(void);

/**
 * @brief 根据 PC 下发数据更新机械臂舵机和电机目标角度。
 */
extern void PC_Arm_Motor_Control_Updata(void);

/**
 * @brief 将当前底盘速度状态写入 PC 上传数据结构。
 */
extern void pc_up_tx_data(void);
#endif // !__REMOTE_CONTROL__
