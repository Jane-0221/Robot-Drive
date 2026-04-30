/**
 * @file CAN_receive_send.c
 * @author Siri (lixirui2017@outlook.com)
 * @brief CAN总线底层驱动（BSP层）：实现CAN帧发送、接收、中断回调等核心功能
 * @version 0.2
 * @date 2024-10-19
 * @copyright Copyright (c) 2024
 */
#include "can_receive_send.h"
#include "dm4310_drv.h"
#include "string.h"
#include "Robstride04.h"
#include "arm.h"
#include "omni_wheel.h"
#include "stdio.h"
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "remote_control.h"
#include "music.h"
#include "LED.h"
#include <cmsis_os2.h>
#include "iwdg.h"
#include "buzzer.h"
#include "fdcan.h"
#include "DrEmpower_can.h" // 大然电机通信协议头文件（第三方电机驱动）
#include "ktech_motor.h"
#include "head.h"
#include "DrEmpower_can.h"

extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;
extern FDCAN_HandleTypeDef hfdcan3;

/**
 * @brief CAN总线初始化函数（占位函数）
 * @note  1. 实际CAN控制器初始化由HAL库自动生成的MX_FDCANx_Init函数完成（在fdcan.c中）；
 *        2. 本函数仅做声明占位，无实际初始化逻辑，可根据需求补充自定义初始化；
 *        3. 中断配置、过滤器配置等核心初始化逻辑在HAL_FDCAN_ErrorCallback中也有兜底处理。
 */
void can_init(void)
{
  // 实际初始化由 HAL 库自动生成的 MX_FDCANx_Init 完成，此处仅占位
  // 若需自定义初始化（如过滤器、中断），可在此补充
}

/**
 * @brief 发送CAN标准帧（11位ID）
 * @param  hcan   CAN控制器句柄（如hfdcan1或hfdcan2）
 * @param  id     CAN标准帧ID（11位，取值0~0x7FF）
 * @param  data   待发送的数据缓冲区指针
 * @param  len    待发送数据的字节长度
 * @retval uint8_t 发送状态：0=成功，1=不支持的长度（非8/12/16/20/24/48/64字节）
 * @note   1. 数据长度会自动映射为FDCAN标准DLC值（如len<=8时按8字节发送）；
 *         2. 发送失败会触发Error_Handler错误处理函数。
 */
uint8_t canx_send_data(FDCAN_HandleTypeDef *hcan, uint16_t id, uint8_t *data, uint32_t len)
{
  FDCAN_TxHeaderTypeDef TxHeader;
  TxHeader.Identifier = id;                  // 设置CAN标准帧ID
  TxHeader.IdType = FDCAN_STANDARD_ID;       // 帧类型：标准帧（11位ID）
  TxHeader.TxFrameType = FDCAN_DATA_FRAME;   // 帧类型：数据帧（非远程帧）

  // 数据长度映射（FDCAN仅支持固定DLC长度，不足则补零，超出则返回错误）
  if (len <= 8)
    TxHeader.DataLength = FDCAN_DLC_BYTES_8;
  else if (len == 12)
    TxHeader.DataLength = FDCAN_DLC_BYTES_12;
  else if (len == 16)
    TxHeader.DataLength = FDCAN_DLC_BYTES_16;
  else if (len == 20)
    TxHeader.DataLength = FDCAN_DLC_BYTES_20;
  else if (len == 24)
    TxHeader.DataLength = FDCAN_DLC_BYTES_24;
  else if (len == 48)
    TxHeader.DataLength = FDCAN_DLC_BYTES_48;
  else if (len == 64)
    TxHeader.DataLength = FDCAN_DLC_BYTES_64;
  else
    return 1; // 不支持的长度，返回错误

  // 固定配置：错误状态激活、关闭位速率切换、经典CAN格式、无发送事件、消息标记
  TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
  TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
  TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  TxHeader.MessageMarker = 0x00;

  // 将消息添加到发送FIFO队列，失败则触发错误处理
  if (HAL_FDCAN_AddMessageToTxFifoQ(hcan, &TxHeader, data) != HAL_OK)
  {
    Error_Handler();
  }
  return 0; // 发送成功
}

/**
 * @brief 发送CAN扩展帧（29位ID）
 * @param  hcan   CAN控制器句柄（如hfdcan1或hfdcan2）
 * @param  id     CAN扩展帧ID（29位，取值0~0x1FFFFFFF）
 * @param  data   待发送的数据缓冲区指针
 * @param  len    待发送数据的字节长度
 * @retval uint8_t 发送状态：0=成功，1=不支持的长度（非8/12/16/20/24/48/64字节）
 * @note   逻辑与标准帧发送一致，仅帧ID类型为扩展帧（29位）。
 */
uint8_t canx_send_ext_data(FDCAN_HandleTypeDef *hcan, uint32_t id, uint8_t *data, uint32_t len)
{
  FDCAN_TxHeaderTypeDef TxHeader;
  TxHeader.Identifier = id;                  // 设置CAN扩展帧ID
  TxHeader.IdType = FDCAN_EXTENDED_ID;       // 帧类型：扩展帧（29位ID）
  TxHeader.TxFrameType = FDCAN_DATA_FRAME;   // 帧类型：数据帧

  // 数据长度映射（与标准帧逻辑一致）
  if (len <= 8)
    TxHeader.DataLength = FDCAN_DLC_BYTES_8;
  else if (len == 12)
    TxHeader.DataLength = FDCAN_DLC_BYTES_12;
  else if (len == 16)
    TxHeader.DataLength = FDCAN_DLC_BYTES_16;
  else if (len == 20)
    TxHeader.DataLength = FDCAN_DLC_BYTES_20;
  else if (len == 24)
    TxHeader.DataLength = FDCAN_DLC_BYTES_24;
  else if (len == 48)
    TxHeader.DataLength = FDCAN_DLC_BYTES_48;
  else if (len == 64)
    TxHeader.DataLength = FDCAN_DLC_BYTES_64;
  else
    return 1; // 不支持的长度，返回错误

  // 固定配置（与标准帧一致）
  TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
  TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
  TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  TxHeader.MessageMarker = 0x00;

  // 添加到发送FIFO队列，失败则触发错误处理
  if (HAL_FDCAN_AddMessageToTxFifoQ(hcan, &TxHeader, data) != HAL_OK)
  {
    Error_Handler();
  }
  return 0; // 发送成功
}

/**
 * @brief CAN数据接收函数（仅占位，不建议使用）
 * @param  hfdcan    CAN控制器句柄
 * @param  RXFIFO    接收FIFO编号（FDCAN_RX_FIFO0/FDCAN_RX_FIFO1）
 * @param  fdcan_RxHeader  接收帧头信息存储结构体指针
 * @param  buf       接收数据缓冲区指针
 * @retval uint8_t 始终返回0（原逻辑无有效返回值）
 * @warning 1. 本函数逻辑不完整，仅调用HAL_FDCAN_GetRxMessage但未处理返回值；
 *          2. 原代码中“DataLength>>16”为无效逻辑（已注释），实际无数据长度解析；
 *          3. 建议优先使用中断回调函数（HAL_FDCAN_RxFifo0Callback）处理接收数据，而非本函数。
 */
uint8_t fdcanx_receive(FDCAN_HandleTypeDef *hfdcan, uint32_t RXFIFO, FDCAN_RxHeaderTypeDef *fdcan_RxHeader, uint8_t *buf)
{
  // 尝试从指定FIFO读取数据，失败则直接返回0（无错误处理）
  if (HAL_FDCAN_GetRxMessage(hfdcan, RXFIFO, fdcan_RxHeader, buf) != HAL_OK)
    return 0;
  // 原代码中“DataLength>>16”是错误逻辑（DataLength无高16位有效数据），此处注释弃用
  return 0;
}

/**
 * @brief CAN接收中断回调函数（FDCAN RX FIFO0 中断）
 * @param  hfdcan    触发中断的CAN控制器句柄
 * @param  RxFifo0ITs 中断类型标志（本函数仅处理新消息中断）
 * @note   1. 循环读取FIFO0中的所有新消息，直到FIFO为空；
 *         2. 按CAN控制器（FDCAN1/FDCAN2）和帧ID分类处理不同电机的反馈数据；
 *         3. FDCAN3未在本函数中处理，可根据需求补充。
 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
  FDCAN_RxHeaderTypeDef rx_header; // 存储接收帧头信息
  uint8_t rx_data[8];              // 接收数据缓冲区（默认8字节）

  // 仅处理“FIFO0有新消息”中断，其他中断直接返回
  if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0)
    return;

  // 循环读取FIFO0中的所有消息（直到读取失败，即FIFO为空）
  while (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header, rx_data) == HAL_OK)
  {
    // ========== FDCAN1 数据处理（科泰电机反馈） ==========
    if (hfdcan->Instance == FDCAN1)
    {
      // 帧ID 0x141：解析第0路科泰电机反馈数据
      if (rx_header.Identifier == 0x141)
      {
        ktech_parse_motor_fb(&motor_linkong[0], rx_data);
      }
      // 帧ID 0x142：解析第1路科泰电机反馈数据
      else if (rx_header.Identifier == 0x142)
      {
        ktech_parse_motor_fb(&motor_linkong[1], rx_data);
      }
    }

    // ========== FDCAN2 数据处理（达妙/大然/RobStride电机） ==========
    else if (hfdcan->Instance == FDCAN2)
    {
      // 标准帧（11位ID）处理逻辑
      if (rx_header.IdType == FDCAN_STANDARD_ID)
      {
        // 按帧ID分类处理
        switch (rx_header.Identifier)
        {
        case 4: // 帧ID=4：解析机械臂4号电机反馈
          damiao_fbdata(&arm_motor[Motor4], rx_data);
          break;
        case 5: // 帧ID=5：解析机械臂5号电机反馈
          damiao_fbdata(&arm_motor[Motor5], rx_data);
          break;
        case 6: // 帧ID=6：解析机械臂6号电机反馈
          damiao_fbdata(&arm_motor[Motor6], rx_data);
          break;
        default:
          // 非4/5/6 ID：解析大然电机ID（从帧ID高位提取）
          uint8_t motor_id = (rx_header.Identifier >> 5) & 0x3F;
          switch (motor_id)
          {
          case 11: // 电机ID=11：解析第0路大然电机反馈
            DrRobot_ParseFbData(&daran_motor_state[0], rx_data);
            break;
          case 12: // 电机ID=12：解析第1路大然电机反馈
            DrRobot_ParseFbData(&daran_motor_state[1], rx_data);
            break;
          case 13: // 电机ID=13：预留（无处理逻辑）
            break;
          default: // 未定义电机ID：无处理
            break;
          }
          break;
        }
      }
      // 扩展帧（29位ID）处理逻辑（RobStride电机）灵足电机
      else if (rx_header.IdType == FDCAN_EXTENDED_ID)
      {
        // 从扩展帧ID中提取目标电机ID（右移8位后取低8位）
        uint8_t target_id = (uint8_t)((rx_header.Identifier >> 8) & 0xFF);
        if (target_id == 0x01) // 电机ID=0x01：解析1号RobStride电机
        {
          RobStride_Motor_Analysis(&motor1, rx_data, rx_header.Identifier);
        }
        else if (target_id == 0x02) // 电机ID=0x02：解析2号RobStride电机
        {
          RobStride_Motor_Analysis(&motor2, rx_data, rx_header.Identifier);
        }
        else if (target_id == 0x03) // 电机ID=0x03：解析3号RobStride电机
        {
          RobStride_Motor_Analysis(&motor3, rx_data, rx_header.Identifier);
        }
      }
    }

    // ========== FDCAN3 数据处理（全向轮 RobStride 电机） ==========
    else if (hfdcan->Instance == FDCAN3)
    {
      if (rx_header.IdType == FDCAN_EXTENDED_ID)
      {
        Omni_Wheel_RxCallback(rx_header.Identifier, rx_data);
      }
    }
  }
}

/**
 * @brief CAN错误回调函数（CAN通信出错时触发）
 * @param  hfdcan  出错的CAN控制器句柄
 * @note   1. 功能：重启CAN控制器 + 重新配置过滤器 + 重新开启中断，实现错误自恢复；
 *         2. 过滤器配置：接收所有标准帧/扩展帧（FilterID=0，掩码=0），统一存入FIFO0；
 *         3. 全局过滤：所有未匹配过滤器的帧也存入FIFO0，避免丢帧。
 */
void HAL_FDCAN_ErrorCallback(FDCAN_HandleTypeDef *hfdcan)
{
  // 步骤1：停止CAN控制器并重新初始化（错误恢复）
  HAL_FDCAN_Stop(hfdcan);
  HAL_FDCAN_DeInit(hfdcan);
  HAL_FDCAN_Init(hfdcan);

  // 步骤2：配置过滤器（接收所有标准帧）
  FDCAN_FilterTypeDef sFilter;
  sFilter.IdType = FDCAN_STANDARD_ID;       // 过滤器类型：标准帧
  sFilter.FilterIndex = 0;                  // 过滤器索引
  sFilter.FilterType = FDCAN_FILTER_MASK;   // 过滤模式：掩码匹配
  sFilter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0; // 匹配帧存入FIFO0
  sFilter.FilterID1 = 0x000;                // 过滤ID=0（接收所有）
  sFilter.FilterID2 = 0x000;                // 掩码=0（接收所有）
  HAL_FDCAN_ConfigFilter(hfdcan, &sFilter);

  // 步骤3：配置过滤器（接收所有扩展帧）
  sFilter.IdType = FDCAN_EXTENDED_ID;       // 过滤器类型：扩展帧
  sFilter.FilterIndex = 0;                  // 复用过滤器索引（覆盖配置）
  sFilter.FilterID1 = 0x00000000;           // 过滤ID=0（接收所有）
  sFilter.FilterID2 = 0x00000000;           // 掩码=0（接收所有）
  HAL_FDCAN_ConfigFilter(hfdcan, &sFilter);

  // 步骤4：配置全局过滤规则
  // 规则：未匹配过滤器的标准帧/扩展帧都存入FIFO0，远程帧过滤
  HAL_FDCAN_ConfigGlobalFilter(hfdcan, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_ACCEPT_IN_RX_FIFO0,
                               FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);

  // 步骤5：开启FIFO0新消息中断通知
  HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
  
  // 步骤6：重启CAN控制器
  HAL_FDCAN_Start(hfdcan);
}
