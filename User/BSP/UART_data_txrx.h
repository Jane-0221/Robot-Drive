/**
 * @file UART_data_txrx.h
 * @author set
 \home
 * @brief 串口数据发送接受
 * @version 0.1
 * @date 2022-11-20
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef __UART_DATA_TXRX__
#define __UART_DATA_TXRX__

#include "stm32h7xx.h" 
#include "stm32h7xx_hal_uart.h"
#include "main.h"
#include "usart.h"
#include "struct_typedef.h"

#define UART_BUFFER_SIZE 256//串口缓冲区大小
//串口数据结构体

typedef PACKED_STRUCT()
{
  UART_HandleTypeDef *huart;
  DMA_HandleTypeDef *hdma_usart_rx;
  DMA_HandleTypeDef *hdma_usart_tx;

  uint8_t rev_data[UART_BUFFER_SIZE];
  HAL_StatusTypeDef Uart_status;
} transmit_data;

extern transmit_data UART7_data;
extern transmit_data UART10_data;
extern transmit_data UART1_data;
extern uint32_t Flag_T13;

//外部调用
void uart_init(void);                                                                                                                         //初始化
void UART_DMA_rxtx_start(transmit_data *data, UART_HandleTypeDef *huart, DMA_HandleTypeDef *hdma_usart_rx, DMA_HandleTypeDef *hdma_usart_tx); //初始化
void UART_rx_IRQHandler(transmit_data *uart);                                                                                                 //接受中断
void UART_send_data(transmit_data uart, uint8_t data[], uint16_t size);                                                                       //发送

//内部调用
void UART_DMA_rxtx_start(transmit_data *data, UART_HandleTypeDef *huart, DMA_HandleTypeDef *hdma_usart_rx, DMA_HandleTypeDef *hdma_usart_tx); //初始化
#endif
// end of file
