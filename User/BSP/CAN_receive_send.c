/**
 * @file CAN_receive_send.c
 * @author Siri (lixirui2017@outlook.com)
 * @brief CAN鎬荤嚎搴曞眰椹卞姩锛圔SP灞傦級锛氬疄鐜癈AN甯у彂閫併€佹帴鏀躲€佷腑鏂洖璋冪瓑鏍稿績鍔熻兘
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
#include "DrEmpower_can.h" // 澶х劧鐢垫満閫氫俊鍗忚澶存枃浠讹紙绗笁鏂圭數鏈洪┍鍔級
#include "ktech_motor.h"
#include "head.h"
#include "DrEmpower_can.h"

extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;
extern FDCAN_HandleTypeDef hfdcan3;

/**
 * @brief CAN鎬荤嚎鍒濆鍖栧嚱鏁帮紙鍗犱綅鍑芥暟锛?
 * @note  1. 瀹為檯CAN鎺у埗鍣ㄥ垵濮嬪寲鐢盚AL搴撹嚜鍔ㄧ敓鎴愮殑MX_FDCANx_Init鍑芥暟瀹屾垚锛堝湪fdcan.c涓級锛?
 *        2. 鏈嚱鏁颁粎鍋氬０鏄庡崰浣嶏紝鏃犲疄闄呭垵濮嬪寲閫昏緫锛屽彲鏍规嵁闇€姹傝ˉ鍏呰嚜瀹氫箟鍒濆鍖栵紱
 *        3. 涓柇閰嶇疆銆佽繃婊ゅ櫒閰嶇疆绛夋牳蹇冨垵濮嬪寲閫昏緫鍦℉AL_FDCAN_ErrorCallback涓篃鏈夊厹搴曞鐞嗐€?
 */
void can_init(void)
{
  // 瀹為檯鍒濆鍖栫敱 HAL 搴撹嚜鍔ㄧ敓鎴愮殑 MX_FDCANx_Init 瀹屾垚锛屾澶勪粎鍗犱綅
  // 鑻ラ渶鑷畾涔夊垵濮嬪寲锛堝杩囨护鍣ㄣ€佷腑鏂級锛屽彲鍦ㄦ琛ュ厖
}

/**
 * @brief 鍙戦€丆AN鏍囧噯甯э紙11浣岻D锛?
 * @param  hcan   CAN鎺у埗鍣ㄥ彞鏌勶紙濡?hfdcan1銆?hfdcan2锛?
 * @param  id     CAN鏍囧噯甯D锛?1浣嶏紝鍙栧€?~0x7FF锛?
 * @param  data   寰呭彂閫佺殑鏁版嵁缂撳啿鍖烘寚閽?
 * @param  len    寰呭彂閫佹暟鎹殑瀛楄妭闀垮害
 * @retval uint8_t 鍙戦€佺姸鎬侊細0=鎴愬姛锛?=涓嶆敮鎸佺殑闀垮害锛堥潪8/12/16/20/24/48/64瀛楄妭锛?
 * @note   1. 鏁版嵁闀垮害浼氳嚜鍔ㄦ槧灏勪负FDCAN鏍囧噯DLC鍊硷紙濡俵en鈮?鏃舵寜8瀛楄妭鍙戦€侊級锛?
 *         2. 鍙戦€佸け璐ヤ細瑙﹀彂Error_Handler閿欒澶勭悊鍑芥暟銆?
 */
uint8_t canx_send_data(FDCAN_HandleTypeDef *hcan, uint16_t id, uint8_t *data, uint32_t len)
{
  FDCAN_TxHeaderTypeDef TxHeader;
  TxHeader.Identifier = id;                  // 璁剧疆CAN鏍囧噯甯D
  TxHeader.IdType = FDCAN_STANDARD_ID;       // 甯х被鍨嬶細鏍囧噯甯э紙11浣岻D锛?
  TxHeader.TxFrameType = FDCAN_DATA_FRAME;   // 甯х被鍨嬶細鏁版嵁甯э紙闈炶繙绋嬪抚锛?

  // 鏁版嵁闀垮害鏄犲皠锛團DCAN浠呮敮鎸佸浐瀹欴LC闀垮害锛屼笉瓒冲垯琛ラ浂锛岃秴鍑哄垯杩斿洖閿欒锛?
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
    return 1; // 涓嶆敮鎸佺殑闀垮害锛岃繑鍥為敊璇?

  // 鍥哄畾閰嶇疆锛氶敊璇姸鎬佹縺娲汇€佸叧闂綅閫熺巼鍒囨崲銆佺粡鍏窩AN鏍煎紡銆佹棤鍙戦€佷簨浠躲€佹秷鎭爣璁?
  TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
  TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
  TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  TxHeader.MessageMarker = 0x00;

  // 灏嗘秷鎭坊鍔犲埌鍙戦€丗IFO闃熷垪锛屽け璐ュ垯瑙﹀彂閿欒澶勭悊
  if (HAL_FDCAN_AddMessageToTxFifoQ(hcan, &TxHeader, data) != HAL_OK)
  {
    Error_Handler();
  }
  return 0; // 鍙戦€佹垚鍔?
}

/**
 * @brief 鍙戦€丆AN鎵╁睍甯э紙29浣岻D锛?
 * @param  hcan   CAN鎺у埗鍣ㄥ彞鏌勶紙濡?hfdcan1銆?hfdcan2锛?
 * @param  id     CAN鎵╁睍甯D锛?9浣嶏紝鍙栧€?~0x1FFFFFFF锛?
 * @param  data   寰呭彂閫佺殑鏁版嵁缂撳啿鍖烘寚閽?
 * @param  len    寰呭彂閫佹暟鎹殑瀛楄妭闀垮害
 * @retval uint8_t 鍙戦€佺姸鎬侊細0=鎴愬姛锛?=涓嶆敮鎸佺殑闀垮害锛堥潪8/12/16/20/24/48/64瀛楄妭锛?
 * @note   閫昏緫涓庢爣鍑嗗抚鍙戦€佷竴鑷达紝浠呭抚ID绫诲瀷涓烘墿灞曞抚锛?9浣嶏級銆?
 */
uint8_t canx_send_ext_data(FDCAN_HandleTypeDef *hcan, uint32_t id, uint8_t *data, uint32_t len)
{
  FDCAN_TxHeaderTypeDef TxHeader;
  TxHeader.Identifier = id;                  // 璁剧疆CAN鎵╁睍甯D
  TxHeader.IdType = FDCAN_EXTENDED_ID;       // 甯х被鍨嬶細鎵╁睍甯э紙29浣岻D锛?
  TxHeader.TxFrameType = FDCAN_DATA_FRAME;   // 甯х被鍨嬶細鏁版嵁甯?

  // 鏁版嵁闀垮害鏄犲皠锛堜笌鏍囧噯甯ч€昏緫涓€鑷达級
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
    return 1; // 涓嶆敮鎸佺殑闀垮害锛岃繑鍥為敊璇?

  // 鍥哄畾閰嶇疆锛堜笌鏍囧噯甯т竴鑷达級
  TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
  TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
  TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  TxHeader.MessageMarker = 0x00;

  // 娣诲姞鍒板彂閫丗IFO闃熷垪锛屽け璐ュ垯瑙﹀彂閿欒澶勭悊
  if (HAL_FDCAN_AddMessageToTxFifoQ(hcan, &TxHeader, data) != HAL_OK)
  {
    Error_Handler();
  }
  return 0; // 鍙戦€佹垚鍔?
}

/**
 * @brief CAN鏁版嵁鎺ユ敹鍑芥暟锛堜粎鍗犱綅锛屼笉寤鸿浣跨敤锛?
 * @param  hfdcan    CAN鎺у埗鍣ㄥ彞鏌?
 * @param  RXFIFO    鎺ユ敹FIFO缂栧彿锛團DCAN_RX_FIFO0/FDCAN_RX_FIFO1锛?
 * @param  fdcan_RxHeader  鎺ユ敹甯уご淇℃伅瀛樺偍缁撴瀯浣撴寚閽?
 * @param  buf       鎺ユ敹鏁版嵁缂撳啿鍖烘寚閽?
 * @retval uint8_t 濮嬬粓杩斿洖0锛堝師閫昏緫鏃犳湁鏁堣繑鍥炲€硷級
 * @warning 1. 鏈嚱鏁伴€昏緫涓嶅畬鏁达紝浠呰皟鐢℉AL_FDCAN_GetRxMessage浣嗘湭澶勭悊杩斿洖鍊硷紱
 *          2. 鍘熶唬鐮佷腑鈥淒ataLength>>16鈥濅负鏃犳晥閫昏緫锛堝凡娉ㄩ噴锛夛紝瀹為檯鏃犳暟鎹暱搴﹁В鏋愶紱
 *          3. 寤鸿浼樺厛浣跨敤涓柇鍥炶皟鍑芥暟锛圚AL_FDCAN_RxFifo0Callback锛夊鐞嗘帴鏀舵暟鎹紝鑰岄潪鏈嚱鏁般€?
 */
uint8_t fdcanx_receive(FDCAN_HandleTypeDef *hfdcan, uint32_t RXFIFO, FDCAN_RxHeaderTypeDef *fdcan_RxHeader, uint8_t *buf)
{
  // 灏濊瘯浠庢寚瀹欶IFO璇诲彇鏁版嵁锛屽け璐ュ垯鐩存帴杩斿洖0锛堟棤閿欒澶勭悊锛?
  if (HAL_FDCAN_GetRxMessage(hfdcan, RXFIFO, fdcan_RxHeader, buf) != HAL_OK)
    return 0;
  // 鍘熶唬鐮佷腑鈥淒ataLength>>16鈥濇槸閿欒閫昏緫锛圖ataLength鏃犻珮16浣嶆湁鏁堟暟鎹級锛屾澶勬敞閲婂純鐢?
  return 0;
}

/**
 * @brief CAN鎺ユ敹涓柇鍥炶皟鍑芥暟锛團DCAN RX FIFO0 涓柇锛?
 * @param  hfdcan    瑙﹀彂涓柇鐨凜AN鎺у埗鍣ㄥ彞鏌?
 * @param  RxFifo0ITs 涓柇绫诲瀷鏍囧織锛堟湰鍑芥暟浠呭鐞嗘柊娑堟伅涓柇锛?
 * @note   1. 寰幆璇诲彇FIFO0涓殑鎵€鏈夋柊娑堟伅锛岀洿鍒癋IFO涓虹┖锛?
 *         2. 鎸塁AN鎺у埗鍣紙FDCAN1/FDCAN2锛夊拰甯D鍒嗙被澶勭悊涓嶅悓鐢垫満鐨勫弽棣堟暟鎹紱
 *         3. FDCAN3鏈湪鏈嚱鏁颁腑澶勭悊锛屽彲鏍规嵁闇€姹傝ˉ鍏呫€?
 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
  FDCAN_RxHeaderTypeDef rx_header; // 瀛樺偍鎺ユ敹甯уご淇℃伅
  uint8_t rx_data[8];              // 鎺ユ敹鏁版嵁缂撳啿鍖猴紙榛樿8瀛楄妭锛?

  // 浠呭鐞嗏€淔IFO0鏈夋柊娑堟伅鈥濅腑鏂紝鍏朵粬涓柇鐩存帴杩斿洖
  if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0)
    return;

  // 寰幆璇诲彇FIFO0涓殑鎵€鏈夋秷鎭紙鐩村埌璇诲彇澶辫触锛屽嵆FIFO涓虹┖锛?
  while (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header, rx_data) == HAL_OK)
  {
    // ========== FDCAN1 鏁版嵁澶勭悊锛堢娉扮數鏈哄弽棣堬級 ==========
    if (hfdcan->Instance == FDCAN1)
    {
      // 甯D 0x141锛氳В鏋愮0璺娉扮數鏈哄弽棣堟暟鎹?
      if (rx_header.Identifier == 0x141)
      {
        ktech_parse_motor_fb(&motor_linkong[0], rx_data);
      }
      // 甯D 0x142锛氳В鏋愮1璺娉扮數鏈哄弽棣堟暟鎹?
      else if (rx_header.Identifier == 0x142)
      {
        ktech_parse_motor_fb(&motor_linkong[1], rx_data);
      }
    }

    // ========== FDCAN2 鏁版嵁澶勭悊锛堝ぇ娣?澶х劧/RobStride鐢垫満锛?==========
    else if (hfdcan->Instance == FDCAN2)
    {
      // 鏍囧噯甯э紙11浣岻D锛夊鐞嗛€昏緫
      if (rx_header.IdType == FDCAN_STANDARD_ID)
      {
        // 鎸夊抚ID鍒嗙被澶勭悊
        switch (rx_header.Identifier)
        {
        case 4: // 甯D=4锛氳В鏋愭満姊拌噦4鍙风數鏈哄弽棣?
          damiao_fbdata(&arm_motor[Motor4], rx_data);
          break;
        case 5: // 甯D=5锛氳В鏋愭満姊拌噦5鍙风數鏈哄弽棣?
          damiao_fbdata(&arm_motor[Motor5], rx_data);
          break;
        case 6: // 甯D=6锛氳В鏋愭満姊拌噦6鍙风數鏈哄弽棣?
          damiao_fbdata(&arm_motor[Motor6], rx_data);
          break;
        default:
          // 闈?/5/6 ID锛氳В鏋愬ぇ鐒剁數鏈篒D锛堜粠甯D楂?浣嶆彁鍙栵級
          uint8_t motor_id = (rx_header.Identifier >> 5) & 0x3F;
          switch (motor_id)
          {
          case 11: // 鐢垫満ID=11锛氳В鏋愮0璺ぇ鐒剁數鏈哄弽棣?
            DrRobot_ParseFbData(&daran_motor_state[0], rx_data);
            break;
          case 12: // 鐢垫満ID=12锛氳В鏋愮1璺ぇ鐒剁數鏈哄弽棣?
            DrRobot_ParseFbData(&daran_motor_state[1], rx_data);
            break;
          case 13: // 鐢垫満ID=13锛氶鐣欙紙鏃犲鐞嗛€昏緫锛?
            break;
          default: // 鏈畾涔夌數鏈篒D锛氭棤澶勭悊
            break;
          }
          break;
        }
      }
      // 鎵╁睍甯э紙29浣岻D锛夊鐞嗛€昏緫锛圧obStride鐢垫満锛?
      else if (rx_header.IdType == FDCAN_EXTENDED_ID)
      {
        // 浠庢墿灞曞抚ID涓彁鍙栫洰鏍囩數鏈篒D锛堝彸绉?浣嶅悗鍙栦綆8浣嶏級
        uint8_t target_id = (uint8_t)((rx_header.Identifier >> 8) & 0xFF);
        if (target_id == 0x01) // 鐢垫満ID=0x01锛氳В鏋?鍙稲obStride鐢垫満
        {
          RobStride_Motor_Analysis(&motor1, rx_data, rx_header.Identifier);
        }
        else if (target_id == 0x02) // 鐢垫満ID=0x02锛氳В鏋?鍙稲obStride鐢垫満
        {
          RobStride_Motor_Analysis(&motor2, rx_data, rx_header.Identifier);
        }
        else if (target_id == 0x03) // 鐢垫満ID=0x03锛氳В鏋?鍙稲obStride鐢垫満
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
 * @brief CAN閿欒鍥炶皟鍑芥暟锛圕AN閫氫俊鍑洪敊鏃惰Е鍙戯級
 * @param  hfdcan  鍑洪敊鐨凜AN鎺у埗鍣ㄥ彞鏌?
 * @note   1. 鍔熻兘锛氶噸鍚疌AN鎺у埗鍣?+ 閲嶆柊閰嶇疆杩囨护鍣?+ 閲嶆柊寮€鍚腑鏂紝瀹炵幇閿欒鑷仮澶嶏紱
 *         2. 杩囨护鍣ㄩ厤缃細鎺ユ敹鎵€鏈夋爣鍑嗗抚/鎵╁睍甯э紙FilterID=0锛屾帺鐮?0锛夛紝缁熶竴瀛樺叆FIFO0锛?
 *         3. 鍏ㄥ眬杩囨护锛氭墍鏈夋湭鍖归厤杩囨护鍣ㄧ殑甯т篃瀛樺叆FIFO0锛岄伩鍏嶄涪甯с€?
 */
void HAL_FDCAN_ErrorCallback(FDCAN_HandleTypeDef *hfdcan)
{
  // 姝ラ1锛氬仠姝AN鎺у埗鍣ㄥ苟閲嶆柊鍒濆鍖栵紙閿欒鎭㈠锛?
  HAL_FDCAN_Stop(hfdcan);
  HAL_FDCAN_DeInit(hfdcan);
  HAL_FDCAN_Init(hfdcan);

  // 姝ラ2锛氶厤缃繃婊ゅ櫒锛堟帴鏀舵墍鏈夋爣鍑嗗抚锛?
  FDCAN_FilterTypeDef sFilter;
  sFilter.IdType = FDCAN_STANDARD_ID;       // 杩囨护鍣ㄧ被鍨嬶細鏍囧噯甯?
  sFilter.FilterIndex = 0;                  // 杩囨护鍣ㄧ储寮?
  sFilter.FilterType = FDCAN_FILTER_MASK;   // 杩囨护妯″紡锛氭帺鐮佸尮閰?
  sFilter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0; // 鍖归厤甯у瓨鍏IFO0
  sFilter.FilterID1 = 0x000;                // 杩囨护ID=0锛堟帴鏀舵墍鏈夛級
  sFilter.FilterID2 = 0x000;                // 鎺╃爜=0锛堟帴鏀舵墍鏈夛級
  HAL_FDCAN_ConfigFilter(hfdcan, &sFilter);

  // 姝ラ3锛氶厤缃繃婊ゅ櫒锛堟帴鏀舵墍鏈夋墿灞曞抚锛?
  sFilter.IdType = FDCAN_EXTENDED_ID;       // 杩囨护鍣ㄧ被鍨嬶細鎵╁睍甯?
  sFilter.FilterIndex = 0;                  // 澶嶇敤杩囨护鍣ㄧ储寮?锛堣鐩栭厤缃級
  sFilter.FilterID1 = 0x00000000;           // 杩囨护ID=0锛堟帴鏀舵墍鏈夛級
  sFilter.FilterID2 = 0x00000000;           // 鎺╃爜=0锛堟帴鏀舵墍鏈夛級
  HAL_FDCAN_ConfigFilter(hfdcan, &sFilter);

  // 姝ラ4锛氶厤缃叏灞€杩囨护瑙勫垯
  // 瑙勫垯锛氭湭鍖归厤杩囨护鍣ㄧ殑鏍囧噯甯?鎵╁睍甯ч兘瀛樺叆FIFO0锛岃繙绋嬪抚杩囨护鎺?
  HAL_FDCAN_ConfigGlobalFilter(hfdcan, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_ACCEPT_IN_RX_FIFO0,
                               FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);

  // 姝ラ5锛氬紑鍚疐IFO0鏂版秷鎭腑鏂€氱煡
  HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
  
  // 姝ラ6锛氶噸鍚疌AN鎺у埗鍣?
  HAL_FDCAN_Start(hfdcan);
}
