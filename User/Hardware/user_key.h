#ifndef __USER_KEY_H__
#define __USER_KEY_H__

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化 USER_KEY 按键 GPIO。
 * @note  USER_KEY 使用 PA15，内部上拉，按下时读到低电平。
 */
void USER_KEY_Init(void);

/**
 * @brief 周期轮询 USER_KEY，并在一次有效按下时使能所有手臂电机。
 * @note  需要在 FreeRTOS 任务上下文调用，因为内部触发的 Arm_EnableAllMotors() 会使用 osDelay()。
 */
void USER_KEY_Update(void);

#ifdef __cplusplus
}
#endif

#endif
