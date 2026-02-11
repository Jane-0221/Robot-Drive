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
  .priority = (osPriority_t) osPriorityNormal1,
};
/* Definitions for Eng_arm */
osThreadId_t Eng_armHandle;
const osThreadAttr_t Eng_arm_attributes = {
  .name = "Eng_arm",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Chassis */
osThreadId_t ChassisHandle;
const osThreadAttr_t Chassis_attributes = {
  .name = "Chassis",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Motor_control */
osThreadId_t Motor_controlHandle;
const osThreadAttr_t Motor_control_attributes = {
  .name = "Motor_control",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Yaw */
osThreadId_t YawHandle;
const osThreadAttr_t Yaw_attributes = {
  .name = "Yaw",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Referee */
osThreadId_t RefereeHandle;
const osThreadAttr_t Referee_attributes = {
  .name = "Referee",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Log_and_debug */
osThreadId_t Log_and_debugHandle;
const osThreadAttr_t Log_and_debug_attributes = {
  .name = "Log_and_debug",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void Remote_control_Task(void *argument);
void Eng_arm_Task(void *argument);
void Chassis_Task(void *argument);
void Motor_control_Task(void *argument);
void Yaw_Task(void *argument);
void Referee_Task(void *argument);
void Log_and_debug_Task(void *argument);

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
void MX_FREERTOS_Init(void) {
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

  /* creation of Eng_arm */
  Eng_armHandle = osThreadNew(Eng_arm_Task, NULL, &Eng_arm_attributes);

  /* creation of Chassis */
  ChassisHandle = osThreadNew(Chassis_Task, NULL, &Chassis_attributes);

  /* creation of Motor_control */
  Motor_controlHandle = osThreadNew(Motor_control_Task, NULL, &Motor_control_attributes);

  /* creation of Yaw */
  YawHandle = osThreadNew(Yaw_Task, NULL, &Yaw_attributes);

  /* creation of Referee */
  RefereeHandle = osThreadNew(Referee_Task, NULL, &Referee_attributes);

  /* creation of Log_and_debug */
  Log_and_debugHandle = osThreadNew(Log_and_debug_Task, NULL, &Log_and_debug_attributes);

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
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN Remote_control_Task */
  /* Infinite loop */
  for (;;)
  {
    osDelay(1);
  }
  /* USER CODE END Remote_control_Task */
}

/* USER CODE BEGIN Header_Eng_arm_Task */
/**
 * @brief Function implementing the Eng_arm thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Eng_arm_Task */
void Eng_arm_Task(void *argument)
{
  /* USER CODE BEGIN Eng_arm_Task */
  /* Infinite loop */
  for (;;)
  {
    Arm_motor1();
    osDelay(1);
    // Arm_motor2();
    // osDelay(1);
    // Arm_motor3();
    // osDelay(1);
    // Arm_motor4();
    osDelay(1);
    Arm_motor5();
    osDelay(1);
    // Arm_motor6();
    //printf("666\n");
  }
  /* USER CODE END Eng_arm_Task */
}

/* USER CODE BEGIN Header_Chassis_Task */
/**
 * @brief Function implementing the Chassis thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Chassis_Task */
void Chassis_Task(void *argument)
{
  /* USER CODE BEGIN Chassis_Task */
  /* Infinite loop */
  for (;;)
  {

    HAL_IWDG_Refresh(&hiwdg1);
    osDelay(1);
  }
  /* USER CODE END Chassis_Task */
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
  }
  /* USER CODE END Motor_control_Task */
}

/* USER CODE BEGIN Header_Yaw_Task */
/**
 * @brief Function implementing the Yaw thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Yaw_Task */
void Yaw_Task(void *argument)
{
  /* USER CODE BEGIN Yaw_Task */
  /* Infinite loop */
  for (;;)
  {

    osDelay(1);
  }
  /* USER CODE END Yaw_Task */
}

/* USER CODE BEGIN Header_Referee_Task */
/**
 * @brief Function implementing the Referee thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Referee_Task */
void Referee_Task(void *argument)
{
  /* USER CODE BEGIN Referee_Task */
  /* Infinite loop */
  for (;;)
  {

    osDelay(1);
  }
  /* USER CODE END Referee_Task */
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

  /* Infinite loop */
  for (;;)
  {
    LEDshowcolor(RED);
    osDelay(500);
    LEDshowcolor(BLUE);
    osDelay(500);
    LEDshowcolor(GREEN);
    osDelay(500);
    // Music_play(melody);
    //  printf("hello\n");
    osDelay(1);
  }
  /* USER CODE END Log_and_debug_Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

