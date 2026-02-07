/**
 * @file CAN_receive_send.c
 * @author Siri (lixirui2017@outlook.com)
 * @brief can bsp层发送与接受
 * @version 0.1
 * @date 2024-10-19
 *
 * @copyright Copyright (c) 2024
 *
 */
// #include "cover_headerfile_h.h"
#include "can_receive_send.h"
#include "motor.h"
#include "supercup.h"
#include "dm4310_drv.h"
#include "string.h"
/**/
FDCAN_RxHeaderTypeDef RxHeader1;
uint8_t g_Can1RxData[64];
float daibaoshan[3];
motor_measure_t motor_data[33];
uint32_t Flag_damiao[6] ={ 0};
//前一时刻的电机接收flag
uint32_t Pre_Flag_damiao [6] ={ 0};
/**/
// CAN寄存器及控制器
extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;
extern FDCAN_HandleTypeDef hfdcan3; // 定义原型在fdcan.c文件

/**
 * @brief 初始化can,包含过滤器配置与使能
 *
 */
void can_init(void)
{
  FDCAN_FilterTypeDef fdcan_filter;

  fdcan_filter.IdType = FDCAN_STANDARD_ID;             // 过滤标准ID
  fdcan_filter.FilterIndex = 0;                        // 滤波器索引
  fdcan_filter.FilterType = FDCAN_FILTER_MASK;         // 掩码模式
  fdcan_filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0; // 过滤器0关联到FIFO0
  fdcan_filter.FilterID1 = 0x00000000;                 // 不去过滤任何ID
  fdcan_filter.FilterID2 = 0x00000000;                 // 同上

  HAL_FDCAN_ConfigFilter(&hfdcan1, &fdcan_filter); // 将上述配置到CAN1
  HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
  HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
  HAL_FDCAN_Start(&hfdcan1);

  HAL_FDCAN_ConfigFilter(&hfdcan2, &fdcan_filter);
  HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
  HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
  HAL_FDCAN_Start(&hfdcan2);

  HAL_FDCAN_ConfigFilter(&hfdcan3, &fdcan_filter);
  HAL_FDCAN_ConfigGlobalFilter(&hfdcan3, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
  HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
  HAL_FDCAN_Start(&hfdcan3);
}
//damiao
void get_motor_measure(motor_measure_t *ptr, uint8_t data[])
{
  (ptr)->last_ecd = (ptr)->ecd;
  (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);
  (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);
  (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);
  (ptr)->temperate = (data)[6];
}


/**
************************************************************************
* @brief:      	fdcanx_send_data(FDCAN_HandleTypeDef *hfdcan, uint16_t id, uint8_t *data, uint32_t len)
* @param:       hfdcan：FDCAN句柄
* @param:       id：CAN设备ID
* @param:       data：发送的数据
* @param:       len：发送的数据长度
* @retval:     	void
* @details:    	发送数据
************************************************************************
**/
//原来的没有用，所有改了
uint8_t canx_send_data(FDCAN_HandleTypeDef *hcan, uint16_t id, uint8_t *data, uint32_t len)
{
	FDCAN_TxHeaderTypeDef TxHeader;

	TxHeader.Identifier = id;                 // CAN ID
  TxHeader.IdType =  FDCAN_STANDARD_ID ;        
  TxHeader.TxFrameType = FDCAN_DATA_FRAME;  
  if(len<=8)	
	{
	  TxHeader.DataLength =FDCAN_DLC_BYTES_8;     // 发送长度：8byte
	}
	else  if(len==12)	
	{
	   TxHeader.DataLength =FDCAN_DLC_BYTES_12;
	}
	else  if(len==16)	
	{
	  TxHeader.DataLength =FDCAN_DLC_BYTES_16;
	
	}
  else  if(len==20)
	{
		TxHeader.DataLength =FDCAN_DLC_BYTES_20;
	}		
	else  if(len==24)	
	{
	 TxHeader.DataLength =FDCAN_DLC_BYTES_24;	
	}else  if(len==48)
	{
	 TxHeader.DataLength =FDCAN_DLC_BYTES_48;
	}else  if(len==64)
   {
		 TxHeader.DataLength =FDCAN_DLC_BYTES_64;
	 }
											
	TxHeader.ErrorStateIndicator =  FDCAN_ESI_ACTIVE;
  TxHeader.BitRateSwitch = FDCAN_BRS_OFF;//比特率切换关闭，不适用于经典CAN
  TxHeader.FDFormat =  FDCAN_CLASSIC_CAN;            // CANFD
  TxHeader.TxEventFifoControl =  FDCAN_NO_TX_EVENTS;  
  TxHeader.MessageMarker = 0x00;//消息标记

   // 发送CAN指令
  if(HAL_FDCAN_AddMessageToTxFifoQ(hcan, &TxHeader, data) != HAL_OK)
  {
        // 发送失败处理
       Error_Handler();      
  }
	 return 0;
}

/**
************************************************************************
* @brief:      	fdcanx_receive(FDCAN_HandleTypeDef *hfdcan, uint8_t *buf)
* @param:       hfdcan：FDCAN句柄
* @param:       buf：接收数据缓存
* @retval:     	接收的数据长度
* @details:    	接收数据
************************************************************************
**/
//因为源代码没用这个函数，所以直接全改了
int a;
uint8_t fdcanx_receive(FDCAN_HandleTypeDef *hfdcan,uint32_t RXFIFO,FDCAN_RxHeaderTypeDef *fdcan_RxHeader,uint8_t *buf)
{	
  if(HAL_FDCAN_GetRxMessage(hfdcan,RXFIFO,fdcan_RxHeader,buf)!=HAL_OK)
		a = 1;
		//return 0;//接收数据
  return fdcan_RxHeader->DataLength>>16;	
}

/**
 * @brief CAN接受回调函数
 * 
 * @param hfdcan 
 * @param RxFifo0ITs 
 */
/*编码器累加计算*/
void process_motor_data(motor_measure_t *motor_data)
{
  // 计算总圈数
  if (motor_data->last_ecd > 7000 && motor_data->ecd < 1000)
    motor_data->ecd_cnt += ((ECD_MAX - motor_data->last_ecd) + motor_data->ecd);
  else if (motor_data->last_ecd < 1000 && motor_data->ecd > 7000)
    motor_data->ecd_cnt -= ((ECD_MAX - motor_data->ecd) + motor_data->last_ecd);
  else
    motor_data->ecd_cnt += (motor_data->ecd - motor_data->last_ecd);
}

//can接收回调  
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{

  FDCAN_RxHeaderTypeDef rx_header; // CAN 数据指针
  uint8_t rx_data[8];              // 获取到的数据

  if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
  {
    HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header, rx_data);
    //电机帧
    DJIMotor_decode_candata(hfdcan, rx_header.Identifier, rx_data);
    if(hfdcan->Instance == FDCAN1)
    {    
        fdcanx_receive(hfdcan,FDCAN_RX_FIFO0,&RxHeader1,g_Can1RxData);
      switch(rx_header.Identifier)
      {
        case 5 :damiao_fbdata(&arm_motor[Motor1],rx_data );   Flag_damiao[1] += 1;break; 
        default:break;
      }

    }
    if(hfdcan->Instance == FDCAN2)
     	{
        fdcanx_receive(hfdcan,FDCAN_RX_FIFO0,&RxHeader1,g_Can1RxData);

    //     if( rx_header.Identifier==5)
    //  {
    // 	  damiao_fbdata(&arm_motor[Motor5],rx_data );   Flag_damiao[5] += 1;
    //  }
		    	switch(rx_header.Identifier)
				{
					case 2 :damiao_fbdata(&arm_motor[Motor2],rx_data );   Flag_damiao[2] += 1;break;  
					case 3 :damiao_fbdata(&arm_motor[Motor3],rx_data );   Flag_damiao[3] += 1;break;
					case 4 :damiao_fbdata(&arm_motor[Motor4],rx_data );   Flag_damiao[4] += 1;break;
					case 5 :damiao_fbdata(&arm_motor[Motor5],rx_data );   Flag_damiao[5] += 1;break;
					// case 0x206 :get_motor_measure(&motor_data[11+RxHeader1.Identifier - CAN_ID1], g_Can1RxData);
					// 					process_motor_data(&motor_data[11+RxHeader1.Identifier - CAN_ID1]);break;
					default:break;
				}		
    }


//达妙
//  if(hfdcan->Instance == FDCAN2)
// 		{
// 			memset(g_Can1RxData, 0, sizeof(g_Can1RxData));
// 			fdcanx_receive(hfdcan,FDCAN_RX_FIFO0,&RxHeader1,g_Can1RxData);							
			
//     	switch(RxHeader1.Identifier)
// 				{
// 					case 2 :dm10010l_fbdata(&arm_motor[Motor2], g_Can1RxData,RxHeader1.DataLength); Flag_damiao[2] += 1;break;     
// 					case 3 :dm4310_fbdata(&arm_motor[Motor3], g_Can1RxData,RxHeader1.DataLength);   Flag_damiao[3] += 1;break;
// 					case 4 :dm4310_fbdata(&arm_motor[Motor4], g_Can1RxData,RxHeader1.DataLength);   Flag_damiao[4] += 1;break;
// 					case 5 :	damiao_fbdata(&arm_motor[Motor5],rx_data );   Flag_damiao[5] += 1;break;
// 					case 0x206 :get_motor_measure(&motor_data[11+RxHeader1.Identifier - CAN_ID1], g_Can1RxData);
// 										process_motor_data(&motor_data[11+RxHeader1.Identifier - CAN_ID1]);break;
// 					default:break;
// 				}				
// 		}
// 		      else if(hfdcan->Instance == FDCAN3)
// 		{
// 			memset(g_Can1RxData, 0, sizeof(g_Can1RxData));
// 			fdcanx_receive(hfdcan,FDCAN_RX_FIFO0,&RxHeader1,g_Can1RxData);							
// 			get_motor_measure(&motor_data[22+RxHeader1.Identifier - CAN_ID1], g_Can1RxData);
// 			process_motor_data(&motor_data[22+RxHeader1.Identifier - CAN_ID1]);
// 		}
  }


}

/**
 * @brief CAN错误处理回调函数，重启相关设备
 * 
 * @param hfdcan 
 */
void HAL_FDCAN_ErrorCallback(FDCAN_HandleTypeDef *hfdcan){
  FDCAN_FilterTypeDef fdcan_filter;

  fdcan_filter.IdType = FDCAN_STANDARD_ID;             // 过滤标准ID
  fdcan_filter.FilterIndex = 0;                        // 滤波器索引
  fdcan_filter.FilterType = FDCAN_FILTER_MASK;         // 掩码模式
  fdcan_filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0; // 过滤器0关联到FIFO0
  fdcan_filter.FilterID1 = 0x00000000;                 // 不去过滤任何ID
  fdcan_filter.FilterID2 = 0x00000000;                 // 同上

  HAL_FDCAN_Stop(hfdcan);
  HAL_FDCAN_DeInit(hfdcan);
  HAL_FDCAN_Init(hfdcan);
  HAL_FDCAN_ConfigFilter(hfdcan, &fdcan_filter); // 将上述配置到CAN
  HAL_FDCAN_ConfigGlobalFilter(hfdcan, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
  HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
  HAL_FDCAN_Start(hfdcan);
}
