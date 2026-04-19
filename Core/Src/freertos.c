/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "remote_control.h"
#include "music.h"
#include "stdio.h"
#include "LED.h"
#include <cmsis_os2.h>
#include "iwdg.h"
#include "buzzer.h"
#include "task.h"
#include "arm.h"
#include "head.h"
#include "lift_control.h"
#include "arm_sv.h"
#include "omni_wheel.h"
#include "Sbus.h"
#include "stp23l.h"
#include "uart_protocol.h"
#include "pt_sensor.h"
#include "usart.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
uint32_t color = 0;
/* USER CODE END Variables */
/* Definitions for Remote_control */
osThreadId_t Remote_controlHandle;
const osThreadAttr_t Remote_control_attributes = {
    .name = "Remote_control",
    .stack_size = 512 * 4,
    .priority = (osPriority_t)osPriorityRealtime,
};
/* Definitions for Arm_MT */
osThreadId_t Arm_MTHandle;
const osThreadAttr_t Arm_MT_attributes = {
    .name = "Arm_MT",
    .stack_size = 512 * 4,
    .priority = (osPriority_t)osPriorityHigh1,
};
/* Definitions for Lift_control */
osThreadId_t Lift_controlHandle;
const osThreadAttr_t Lift_control_attributes = {
    .name = "Lift_control",
    .stack_size = 256 * 4,
    .priority = (osPriority_t)osPriorityHigh2,
};
/* Definitions for Motor_control */
osThreadId_t Motor_controlHandle;
const osThreadAttr_t Motor_control_attributes = {
    .name = "Motor_control",
    .stack_size = 256 * 4,
    .priority = (osPriority_t)osPriorityLow1,
};
/* Definitions for Head */
osThreadId_t HeadHandle;
const osThreadAttr_t Head_attributes = {
    .name = "Head",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityHigh3,
};
/* Definitions for Arm_update */
osThreadId_t Arm_updateHandle;
const osThreadAttr_t Arm_update_attributes = {
    .name = "Arm_update",
    .stack_size = 512 * 4,
    .priority = (osPriority_t)osPriorityHigh2,
};
/* Definitions for Log_and_debug */
osThreadId_t Log_and_debugHandle;
const osThreadAttr_t Log_and_debug_attributes = {
    .name = "Log_and_debug",
    .stack_size = 256 * 4,
    .priority = (osPriority_t)osPriorityLow3,
};
/* Definitions for Arm_SV */
osThreadId_t Arm_SVHandle;
const osThreadAttr_t Arm_SV_attributes = {
    .name = "Arm_SV",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityLow2,
};
/* Definitions for PC_Comm */
osThreadId_t PC_CommHandle;
const osThreadAttr_t PC_Comm_attributes = {
    .name = "PC_Comm",
    .stack_size = 256 * 4,
    .priority = (osPriority_t)osPriorityRealtime,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void Pump_Update(void);

/* USER CODE END FunctionPrototypes */

void Remote_control_Task(void *argument);
void Arm_MT_Task(void *argument);
void Lift_control_Task(void *argument);
void Motor_control_Task(void *argument);
void Head_Task(void *argument);
void Arm_update_Task(void *argument);
void Log_and_debug_Task(void *argument);
void Arm_SV_Task(void *argument);
void PC_Comm_Task(void *argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void vApplicationIdleHook(void);

/* USER CODE BEGIN 2 */
void vApplicationIdleHook(void)
{
  /* vApplicationIdleHook() will only be called if configUSE_IDLE_HOOK is set
  to 1 in FreeRTOSConfig.h. It will be called on each iteration of the idle
  task. It is essential that code added to this hook function never attempts
  to block in any way (for example, call xQueueReceive() with a block time
  specified, or call vTaskDelay()). If the application makes use of the
  vTaskDelete() API function (as this demo application does) then it is also
  important that vApplicationIdleHook() is permitted to return to its calling
  function, because it is the responsibility of the idle task to clean up
  memory allocated by the kernel to any task that has since been deleted. */
}
/* USER CODE END 2 */

/**
 * @brief  FreeRTOS initialization
 * @param  None
 * @retval None
 */
void MX_FREERTOS_Init(void)
{
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of Remote_control */
  Remote_controlHandle = osThreadNew(Remote_control_Task, NULL, &Remote_control_attributes);

  /* creation of Arm_MT */
  Arm_MTHandle = osThreadNew(Arm_MT_Task, NULL, &Arm_MT_attributes);

  /* creation of Lift_control */
  Lift_controlHandle = osThreadNew(Lift_control_Task, NULL, &Lift_control_attributes);

  /* creation of Motor_control */
  Motor_controlHandle = osThreadNew(Motor_control_Task, NULL, &Motor_control_attributes);

  /* creation of Head */
  HeadHandle = osThreadNew(Head_Task, NULL, &Head_attributes);

  /* creation of Arm_update */
  Arm_updateHandle = osThreadNew(Arm_update_Task, NULL, &Arm_update_attributes);

  /* creation of Log_and_debug */
  Log_and_debugHandle = osThreadNew(Log_and_debug_Task, NULL, &Log_and_debug_attributes);

  /* creation of Arm_SV */
  Arm_SVHandle = osThreadNew(Arm_SV_Task, NULL, &Arm_SV_attributes);

  /* creation of PC_Comm */
  PC_CommHandle = osThreadNew(PC_Comm_Task, NULL, &PC_Comm_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */
}

/* USER CODE BEGIN Header_Remote_control_Task */
/**
 * @brief  Function implementing the Remote_control thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_Remote_control_Task */
void Remote_control_Task(void *argument)
{
  /* init code for USB_DEVICE */

  /* USER CODE BEGIN Remote_control_Task */
  MX_USB_DEVICE_Init();
  int control_mode = 0; // 0: 遥控模式, 1: pc模式
  PT_Send_ReadTemp_Cmd(&huart1);
  /* Infinite loop */
  for (;;)
  {
    PT_Send_ReadPress_Cmd(&huart1);
    osDelay(1);
    update_sbus(sbus_data_buffer, &SBUS_CH); //
    Omni_Wheel_Update();
    if (control_mode == 0)
    {
      // 遥控模式
      osDelay(1);
      Pump_Control_Updata();
      osDelay(1);
      Head_Motor_Control_Updata();
      osDelay(1);
      Up_Down_Motor_Control_Updata();
    }
    else if (control_mode == 1)
    {
      // pc模式
      osDelay(1);
      PC_Pump_Control_Updata();
      osDelay(1);
      PC_Head_Motor_Control_Updata();
      osDelay(1);
      PC_Up_Down_Motor_Control_Updata();
      osDelay(1);
      PC_Arm_Motor_Control_Updata();
    }
    ARM_SV_Tx_Rx();

    osDelay(1);
  }

  /* USER CODE END Remote_control_Task */
}

/* USER CODE BEGIN Header_Arm_MT_Task */
/**
 * @brief Function implementing the Arm_MT thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Arm_MT_Task */
void Arm_MT_Task(void *argument)
{
  /* USER CODE BEGIN Arm_MT_Task */
  /* Infinite loop */
  for (;;)
  {
    osDelay(1);
    Arm_all_tx();

  }
  /* USER CODE END Arm_MT_Task */
}

/* USER CODE BEGIN Header_Lift_control_Task */
/**
 * @brief Function implementing the Lift_control thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Lift_control_Task */
void Lift_control_Task(void *argument)
{
  /* USER CODE BEGIN Lift_control_Task */
  /* Infinite loop */
  for (;;)
  {
    STP23L_ParseData(stp23l_raw_data, sizeof(stp23l_raw_data)); // 解析升降数据包
    PT_ParsePressureToGlobal(pt_raw_buf, sizeof(pt_raw_buf));

    Lift_RefreshHeight();           // 高度数据处理，得到当前高度
    Lift_GoToTarget(aim_tx_height); // 根据目标高度，控制电机运动
    osDelay(1);
    Pump_Update(); // 更新气泵状态
    osDelay(1);
    Lift_UpdateMotor(); // 发生升降控制信息
  }
  /* USER CODE END Lift_control_Task */
}

/* USER CODE BEGIN Header_Motor_control_Task */
/**
 * @brief Function implementing the Motor_control thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Motor_control_Task */
void Motor_control_Task(void *argument)
{
  /* USER CODE BEGIN Motor_control_Task */
  /* Infinite loop */
  for (;;)
  {

    osDelay(1);
  }
  /* USER CODE END Motor_control_Task */
}

/* USER CODE BEGIN Header_Head_Task */
/**
 * @brief Function implementing the Head thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Head_Task */
void Head_Task(void *argument)
{
  /* USER CODE BEGIN Head_Task */
  /* Infinite loop */
  for (;;)
  {
    Head_all_tx();

    Head_Lk_Data_update();

    osDelay(1);
  }
  /* USER CODE END Head_Task */
}

/* USER CODE BEGIN Header_Arm_update_Task */
/**
 * @brief Function implementing the Arm_update thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Arm_update_Task */
void Arm_update_Task(void *argument)
{
  /* USER CODE BEGIN Arm_update_Task */
  /* Infinite loop */
  for (;;)
  {
    Arm_All_Data_update();
    osDelay(1);
  }
  /* USER CODE END Arm_update_Task */
}

/* USER CODE BEGIN Header_Log_and_debug_Task */
/**
 * @brief Function implementing the Log_and_debug thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Log_and_debug_Task */
void Log_and_debug_Task(void *argument)
{
  /* USER CODE BEGIN Log_and_debug_Task */
  LEDshowcolor(RED);
  osDelay(50);
  LEDshowcolor(BLUE);
  osDelay(50);
  LEDshowcolor(GREEN);
  osDelay(50);
  /* Infinite loop */
  for (;;)
  {
    // Music_play(melody);
    // printf("hello\n");
    // Music_play_56_nations();
    osDelay(1);
  }
  /* USER CODE END Log_and_debug_Task */
}

/* USER CODE BEGIN Header_Arm_SV_Task */
/**
 * @brief Function implementing the Arm_SV thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Arm_SV_Task */
void Arm_SV_Task(void *argument)
{
  /* USER CODE BEGIN Arm_SV_Task */
  /* Infinite loop */

  for (;;)
  {
    // ARM_SV_Tx_Rx();
    osDelay(1);
  }
  /* USER CODE END Arm_SV_Task */
}

/* USER CODE BEGIN Header_PC_Comm_Task */
/**
 * @brief Function implementing the PC_Comm thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_PC_Comm_Task */
void PC_Comm_Task(void *argument)
{
  /* USER CODE BEGIN PC_Comm_Task */
  /* Infinite loop */
  for (;;)
  {
    pc_up_tx_data();
    send_up_frame(&huart10);
    unpack_dn_frame(uart_protocol_raw_data, &pc_dn_data);
    osDelay(1);
    HAL_IWDG_Refresh(&hiwdg1);
  }
  /* USER CODE END PC_Comm_Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
