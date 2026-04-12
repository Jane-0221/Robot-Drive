#include "arm.h"
#include "dm4310_drv.h"
#include "Robstride04.h"
#include "pid.h"
#include "remote_control.h"
#include "gpio.h"
#include "gom_protocol.h"
#include "usart.h"
#include "LZ_motor_driver.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "DrEmpower_can.h"
#include "stdio.h"

// ===================== 鍏ㄥ眬鍙橀噺瀹氫箟 =====================
/**
 * @brief 鏈烘鑷傝偐閮ㄧ數鏈虹被鍨嬶紙鏋氫妇鍊硷細SHOULDER_TYPE_LINGZU/DAREN/DAMIAO锛?
 * @note  鐢ㄤ簬鍒囨崲涓嶅悓鍝佺墝鐢垫満鐨勬帶鍒堕€昏緫锛堝噷缁?澶х劧/澶ф芳锛?
 */
ShoulderType_t g_ShoulderType;

/**
 * @brief 澶х劧鑸垫満鐘舵€佺粨鏋勪綋鏁扮粍锛?璺級
 * @note  瀛樺偍澶х劧鑸垫満鐨勭姸鎬佷俊鎭紙瑙掑害銆侀€熷害銆佹壄鐭╃瓑锛?
 */
struct servo_state servo_state_daran[3];

/**
 * @brief 澶х劧鑸垫満鐢靛帇/鐢垫祦缁撴瀯浣撴暟缁勶紙3璺級
 * @note  瀛樺偍澶х劧鑸垫満鐨勭數姘斿弬鏁帮紙杈撳叆鐢靛帇銆佺浉鐢垫祦绛夛級
 */
struct servo_volcur servo_volcur_daran[3];

/**
 * @brief 澶ф芳鐢垫満鐘舵€佺粨鏋勪綋鏁扮粍锛?璺級
 * @note  瀛樺偍DM4310鐢垫満鐨勪綅缃€侀€熷害銆佺數娴佺瓑鐘舵€?
 */
extern Motor_DM_Status DM_Status[6];

/**
 * @brief 鍑岀粍鐢垫満瀵硅薄锛?璺級
 * @note  Robstride04鍗忚椹卞姩锛屽搴斿噷缁?/2/3鍙风數鏈?
 */
RobStride_Motor_t motor1; // 鍑岀粍1鍙风數鏈?
RobStride_Motor_t motor2; // 鍑岀粍2鍙风數鏈?
RobStride_Motor_t motor3; // 鍑岀粍3鍙风數鏈?

/**
 * @brief 鍑岀粍鐢垫満鏁版嵁缁撴瀯浣撴暟缁勶紙3璺級
 * @note  瀛樺偍鍑岀粍鐢垫満鐨勭洰鏍?褰撳墠瑙掑害銆侀€熷害绛夋帶鍒跺弬鏁?
 */
ArmMotorData_t Linzu_motor_data[3];

/**
 * @brief 澶х劧鐢垫満鏁版嵁缁撴瀯浣撴暟缁勶紙3璺級
 * @note  瀛樺偍澶х劧鐢垫満鐨勭洰鏍?褰撳墠瑙掑害銆侀€熷害绛夋帶鍒跺弬鏁?
 */
ArmMotorData_t Daran_motor_data[3];

/**
 * @brief 澶ф芳鐢垫満鏁版嵁缁撴瀯浣撴暟缁勶紙3璺級
 * @note  瀛樺偍澶ф芳鐢垫満鐨勭洰鏍?褰撳墠瑙掑害銆侀€熷害绛夋帶鍒跺弬鏁?
 */
ArmMotorData_t Damiao_motor_data[3];

/**
 * @brief 鏁版嵁鍥炲浣胯兘鏍囧織
 * @note  娴偣鍨嬶細0.0f=绂佺敤鍥炲锛岄潪0.0f=鍚敤鍥炲锛堟湭鍦ㄦ湰浠ｇ爜涓疄闄呬娇鐢級
 */
float reply_enable = 0.0f;

// ===================== 鍑芥暟瀹氫箟 =====================
/**
 * @brief 鏈烘鑷傚垵濮嬪寲鍑芥暟
 * @retval 鏃?
 * @note   1. 鏍稿績鍔熻兘锛氶€夋嫨鑲╅儴鐢垫満绫诲瀷锛屽垵濮嬪寲鍑岀粍/澶х劧/澶ф芳鐢垫満锛岄厤缃瓹AN2閫氫俊锛?
 *         2. 鍑岀粍鐢垫満锛氬垵濮嬪寲RobStride鍗忚锛岃缃瓹SP浣嶇疆妯″紡锛屼娇鑳界數鏈猴紝寮€鍚富鍔ㄤ笂鎶ワ紱
 *         3. 澶х劧鐢垫満锛氭竻闄ら敊璇紝璁剧疆妯″紡2锛堜綅缃ā寮忥級锛岄厤缃弬鏁?2001锛堟瘮渚嬬郴鏁帮級锛?
 *         4. 澶ф芳鐢垫満锛氬垵濮嬪寲浣嶇疆妯″紡锛屾殏鏈厤缃叿浣撳弬鏁帮紱
 *         5. 鎵€鏈夌數鏈哄潎鎸傝浇鍦–AN2鎬荤嚎涓娿€?
 */
void Arm_Init()
{
    // 閫夋嫨榛樿鑲╅儴鐢垫満绫诲瀷锛堟敞閲婁负鍑岀粍锛屽疄闄呭惎鐢ㄥぇ鐒讹級
    // g_ShoulderType = SHOULDER_TYPE_LINGZU;
    g_ShoulderType = SHOULDER_TYPE_DARAN;

    /* 鍑岀粍鐢垫満鍒濆鍖栵紙浣跨敤CAN2鎬荤嚎锛?*/
    // 1鍙峰噷缁勭數鏈哄垵濮嬪寲
    RobStride_Motor_Init(&motor1, MOTOR_LINGZU_1_ID, false);          // 鍒濆鍖栫數鏈哄璞★紙ID涓哄噷缁?鍙凤級
    Get_RobStride_Motor_parameter(&motor1, CAN_HANDLE_2, 0X7005);     // 璇诲彇鐢垫満鍙傛暟锛?X7005涓哄弬鏁板湴鍧€锛?
    HAL_Delay(10);                                                    // 寤舵椂纭繚閫氫俊绋冲畾
    Set_RobStride_Motor_parameter(&motor1, CAN_HANDLE_2, 0X7005, CSP_control_mode, 'j'); // 璁剧疆CSP浣嶇疆鎺у埗妯″紡
    Enable_Motor(&motor1, (hcan_t *)CAN_HANDLE_2);                    // 浣胯兘鐢垫満
    Set_RobStride_Motor_parameter(&motor1, CAN_HANDLE_2, 0X7017, 1.0f, 'p'); // 璁剧疆鍙傛暟锛?X7017锛屾瘮渚?.0锛?
    HAL_Delay(10);
    // 寮€鍚富鍔ㄤ笂鎶ワ紙0x00=鍏抽棴锛?x01=寮€鍚級
    RobStride_Motor_ProactiveEscalationSet(&motor1, CAN_HANDLE_2, 0x01);

    // 2鍙峰噷缁勭數鏈哄垵濮嬪寲锛堥€昏緫鍚?鍙凤級
    RobStride_Motor_Init(&motor2, MOTOR_LINGZU_2_ID, false);
    Get_RobStride_Motor_parameter(&motor2, CAN_HANDLE_2, 0X7005);
    HAL_Delay(10);
    Set_RobStride_Motor_parameter(&motor2, CAN_HANDLE_2, 0X7005, CSP_control_mode, 'j');
    Enable_Motor(&motor2, (hcan_t *)CAN_HANDLE_2);
    Set_RobStride_Motor_parameter(&motor2, CAN_HANDLE_2, 0X7017, 1.0f, 'p');
    HAL_Delay(10);
    RobStride_Motor_ProactiveEscalationSet(&motor2, CAN_HANDLE_2, 0x01);

    // 3鍙峰噷缁勭數鏈哄垵濮嬪寲锛堥€昏緫鍚?鍙凤級
    RobStride_Motor_Init(&motor3, MOTOR_LINGZU_3_ID, false);
    Get_RobStride_Motor_parameter(&motor3, CAN_HANDLE_2, 0X7005);
    HAL_Delay(10);
    Set_RobStride_Motor_parameter(&motor3, CAN_HANDLE_2, 0X7005, CSP_control_mode, 'j');
    Enable_Motor(&motor3, (hcan_t *)CAN_HANDLE_2);
    Set_RobStride_Motor_parameter(&motor3, CAN_HANDLE_2, 0X7017, 1.0f, 'p');
    HAL_Delay(10);
    RobStride_Motor_ProactiveEscalationSet(&motor3, CAN_HANDLE_2, 0x01);

    /* 澶х劧鐢垫満鍒濆鍖栵紙浣跨敤CAN2鎬荤嚎锛?*/
    clear_error(CAN_HANDLE_2, MOTOR_DARAN_1_ID);                     // 娓呴櫎1鍙峰ぇ鐒剁數鏈洪敊璇?
    set_mode(CAN_HANDLE_2, MOTOR_DARAN_1_ID, 2);                     // 璁剧疆妯″紡2锛堜綅缃ā寮忥級
    write_property(CAN_HANDLE_2, MOTOR_DARAN_1_ID, 22001, 3, 1.0f); // 鍐欏叆鍙傛暟22001锛堟瘮渚嬬郴鏁?.0锛?

    clear_error(CAN_HANDLE_2, MOTOR_DARAN_2_ID);                     // 娓呴櫎2鍙峰ぇ鐒剁數鏈洪敊璇?
    set_mode(CAN_HANDLE_2, MOTOR_DARAN_2_ID, 2);                     // 璁剧疆浣嶇疆妯″紡
    write_property(CAN_HANDLE_2, MOTOR_DARAN_2_ID, 22001, 3, 1.0f); // 閰嶇疆姣斾緥绯绘暟

    clear_error(CAN_HANDLE_2, MOTOR_DARAN_3_ID);                     // 娓呴櫎3鍙峰ぇ鐒剁數鏈洪敊璇?
    set_mode(CAN_HANDLE_2, MOTOR_DARAN_3_ID, 2);                     // 璁剧疆浣嶇疆妯″紡
    write_property(CAN_HANDLE_2, MOTOR_DARAN_3_ID, 22001, 3, 1.0f); // 閰嶇疆姣斾緥绯绘暟

    /* 澶ф芳鐢垫満鍒濆鍖栵紙浣跨敤CAN2鎬荤嚎锛?*/
    arm_motor_init(&arm_motor[Motor4], MOTOR_DAMIAO_4_ID, POS_MODE); // 4鍙峰ぇ娣肩數鏈哄垵濮嬪寲锛堜綅缃ā寮忥級
    arm_motor_init(&arm_motor[Motor5], MOTOR_DAMIAO_5_ID, POS_MODE); // 5鍙峰ぇ娣肩數鏈哄垵濮嬪寲锛堜綅缃ā寮忥級
    arm_motor_init(&arm_motor[Motor6], MOTOR_DAMIAO_6_ID, POS_MODE); // 6鍙峰ぇ娣肩數鏈哄垵濮嬪寲锛堜綅缃ā寮忥級

    enable_motor_mode(CAN_HANDLE_2, MOTOR_DAMIAO_4_ID, POS_MODE);    // 浣胯兘4鍙峰ぇ娣肩數鏈轰綅缃ā寮?
    enable_motor_mode(CAN_HANDLE_2, MOTOR_DAMIAO_5_ID, POS_MODE);    // 浣胯兘5鍙峰ぇ娣肩數鏈轰綅缃ā寮?
    enable_motor_mode(CAN_HANDLE_2, MOTOR_DAMIAO_6_ID, POS_MODE);    // 浣胯兘6鍙峰ぇ娣肩數鏈轰綅缃ā寮?

    // 鍒濆鍖栧噷缁勭數鏈虹洰鏍囧弬鏁帮紙瑙掑害10掳锛岄€熷害1.0r/min锛?
    Linzu_motor_data[0].target_angle = 10.0f;
    Linzu_motor_data[1].target_angle = 10.0f;
    Linzu_motor_data[2].target_angle = 10.0f;
    Linzu_motor_data[0].target_velocity = 1.0f;
    Linzu_motor_data[1].target_velocity = 1.0f;
    Linzu_motor_data[2].target_velocity = 1.0f;

    // 鍒濆鍖栧ぇ鐒剁數鏈虹洰鏍囧弬鏁帮紙瑙掑害10掳锛岄€熷害鍒嗗埆涓?0/20/20r/min锛?
    Daran_motor_data[0].target_angle = 10.0f;
    Daran_motor_data[1].target_angle = 10.0f;
    Daran_motor_data[2].target_angle = 10.0f;
    Daran_motor_data[0].target_velocity = 90.0f;
    Daran_motor_data[1].target_velocity = 20.0f;
    Daran_motor_data[2].target_velocity = 20.0f;
}

/**
 * @brief 鎺у埗1鍙峰噷缁勭數鏈猴紙CSP浣嶇疆妯″紡锛?
 * @retval 鏃?
 * @note   鏍规嵁Linzu_motor_data[0]鐨勭洰鏍囪搴?閫熷害锛岄€氳繃CAN2鍙戦€佹帶鍒舵寚浠?
 */
void Arm_Linzu_motor1()
{
    RobStride_Motor_CSP_control(&motor1, CAN_HANDLE_2, Linzu_motor_data[0].target_angle, Linzu_motor_data[0].target_velocity);
}

/**
 * @brief 鎺у埗2鍙峰噷缁勭數鏈猴紙CSP浣嶇疆妯″紡锛?
 * @retval 鏃?
 * @note   鏍规嵁Linzu_motor_data[1]鐨勭洰鏍囪搴?閫熷害锛岄€氳繃CAN2鍙戦€佹帶鍒舵寚浠?
 */
void Arm_Linzu_motor2()
{
    RobStride_Motor_CSP_control(&motor2, CAN_HANDLE_2, Linzu_motor_data[1].target_angle, Linzu_motor_data[1].target_velocity);
}

/**
 * @brief 鎺у埗3鍙峰噷缁勭數鏈猴紙CSP浣嶇疆妯″紡锛?
 * @retval 鏃?
 * @note   鏍规嵁Linzu_motor_data[2]鐨勭洰鏍囪搴?閫熷害锛岄€氳繃CAN2鍙戦€佹帶鍒舵寚浠?
 */
void Arm_Linzu_motor3()
{
    RobStride_Motor_CSP_control(&motor3, CAN_HANDLE_2, Linzu_motor_data[2].target_angle, Linzu_motor_data[2].target_velocity);
}

/**
 * @brief 鎺у埗1鍙峰ぇ鐒剁數鏈猴紙浣嶇疆妯″紡锛?
 * @retval 鏃?
 * @note   1. 寤舵椂1ms纭繚閫氫俊绋冲畾锛?
 *         2. 璁剧疆鐩爣瑙掑害銆侀€熷害銆佸姞閫熷害锛?0.0f锛夈€佺珛鍗崇敓鏁堬紙1锛夛紱
 *         3. 鎸囦护閫氳繃CAN2鍙戦€佽嚦1鍙峰ぇ鐒剁數鏈恒€?
 */
void Arm_Daran_motor1()
{
    HAL_Delay(1);
    set_angle(CAN_HANDLE_2, MOTOR_DARAN_1_ID, Daran_motor_data[0].target_angle, Daran_motor_data[0].target_velocity, 10.0f, 1);
}

/**
 * @brief 鎺у埗2鍙峰ぇ鐒剁數鏈猴紙浣嶇疆妯″紡锛?
 * @retval 鏃?
 * @note   閫昏緫鍚?鍙峰ぇ鐒剁數鏈猴紝浣跨敤Daran_motor_data[1]鐨勭洰鏍囧弬鏁?
 */
void Arm_Daran_motor2()
{
    HAL_Delay(1);
    set_angle(CAN_HANDLE_2, MOTOR_DARAN_2_ID, Daran_motor_data[1].target_angle, Daran_motor_data[1].target_velocity, 10.0f, 1);
}

/**
 * @brief 鎺у埗3鍙峰ぇ鐒剁數鏈猴紙浣嶇疆妯″紡锛?
 * @retval 鏃?
 * @note   閫昏緫鍚?鍙峰ぇ鐒剁數鏈猴紝浣跨敤Daran_motor_data[2]鐨勭洰鏍囧弬鏁?
 */
void Arm_Daran_motor3()
{
    HAL_Delay(1);
    set_angle(CAN_HANDLE_2, MOTOR_DARAN_3_ID, Daran_motor_data[2].target_angle, Daran_motor_data[2].target_velocity, 10.0f, 1);
}

/**
 * @brief 鎺у埗4鍙峰ぇ娣肩數鏈猴紙浣嶇疆妯″紡锛?
 * @retval 鏃?
 * @note   1. 璁剧疆浣嶇疆妯″紡锛?
 *         2. 閰嶇疆鐩爣浣嶇疆/閫熷害锛?
 *         3. 璁剧疆浣嶇疆閫熷害鎺у埗鍙傛暟锛?=浣嶇疆鍙傛暟锛?0=閫熷害鍙傛暟锛夛紱
 *         4. 鍥哄畾鐩爣浣嶇疆涓?0锛堝彲鏍规嵁闇€姹備慨鏀癸級銆?
 */
void Arm_Damiao_motor4()
{
    set_DM_mode(Motor4, POS_MODE);
    set_DM_pos_vel(pos_motor.MT04, vel_motor.MT04, Motor4);
    pos_speed_ctrl(CAN_HANDLE_2, MOTOR_DAMIAO_4_ID, 5, 10);
    pos_motor.MT04 = 20;
}

/**
 * @brief 鎺у埗5鍙峰ぇ娣肩數鏈猴紙浣嶇疆妯″紡锛?
 * @retval 鏃?
 * @note   閫昏緫鍚?鍙峰ぇ娣肩數鏈猴紝浣嶇疆鍙傛暟10锛岄€熷害鍙傛暟1
 */
void Arm_Damiao_motor5()
{
    set_DM_mode(Motor5, POS_MODE);
    set_DM_pos_vel(pos_motor.MT05, vel_motor.MT05, Motor5);
    pos_speed_ctrl(CAN_HANDLE_2, MOTOR_DAMIAO_5_ID, 10, 1);
}

/**
 * @brief 鎺у埗6鍙峰ぇ娣肩數鏈猴紙浣嶇疆妯″紡锛?
 * @retval 鏃?
 * @note   1. 璁剧疆浣嶇疆妯″紡锛?
 *         2. 閰嶇疆鐩爣浣嶇疆/閫熷害锛?
 *         3. 鍥哄畾鐩爣浣嶇疆涓?0锛屼娇鐢ㄧ數鏈?鐨勪綅缃瀹氬€间綔涓烘帶鍒跺弬鏁帮紱
 *         4. 閫熷害鍙傛暟鍥哄畾涓?銆?
 */
void Arm_Damiao_motor6()
{
    set_DM_mode(Motor6, POS_MODE);
    set_DM_pos_vel(pos_motor.MT06, vel_motor.MT06, Motor6);
    pos_motor.MT06 = 10;
    pos_speed_ctrl(CAN_HANDLE_2, MOTOR_DAMIAO_6_ID, arm_motor[Motor6].ctrl.pos_set, 1);
}

/**
 * @brief 鏇存柊鍑岀粍鐢垫満褰撳墠鐘舵€佹暟鎹?
 * @retval 鏃?
 * @note   浠巑otor1/2/3鐨凴obStride鍗忚缂撳瓨涓紝璇诲彇褰撳墠瑙掑害銆侀€熷害锛屾洿鏂板埌Linzu_motor_data
 */
void Arm_Linzu_Data_update()
{
    Linzu_motor_data[0].current_angle = motor1.Pos_Info.Angle;
    Linzu_motor_data[1].current_angle = motor2.Pos_Info.Angle;
    Linzu_motor_data[2].current_angle = motor3.Pos_Info.Angle;
    Linzu_motor_data[0].current_velocity = motor1.Pos_Info.Speed;
    Linzu_motor_data[1].current_velocity = motor2.Pos_Info.Speed;
    Linzu_motor_data[2].current_velocity = motor3.Pos_Info.Speed;
}

/**
 * @brief 鏇存柊澶х劧鐢垫満褰撳墠鐘舵€佹暟鎹?
 * @retval 鏃?
 * @note   1. 浠巇aran_motor_state缂撳瓨涓鍙栬搴︺€侀€熷害锛堟壄鐭╂敞閲婃湭浣跨敤锛夛紱
 *         2. 鏇存柊鍒癉aran_motor_data鐨刢urrent_angle/current_velocity鎴愬憳锛?
 *         3. 璋冭瘯鎵撳嵃浠ｇ爜宸叉敞閲婏紝濡傞渶鏌ョ湅鍙彇娑堟敞閲娿€?
 */
void Arm_Daran_Data_update()
{
    // printf("Angle: %.2f锟斤拷, Speed: %.2f r/min, Torque: %.2f Nm\r\n",
    //        daran_motor_state[0].angle, daran_motor_state[0].speed, daran_motor_state[0].torque);

    Daran_motor_data[0].current_angle = daran_motor_state[0].angle;
    Daran_motor_data[1].current_angle = daran_motor_state[1].angle;
    Daran_motor_data[2].current_angle = daran_motor_state[2].angle;
    Daran_motor_data[0].current_velocity = daran_motor_state[0].speed;
    Daran_motor_data[1].current_velocity = daran_motor_state[1].speed;
    Daran_motor_data[2].current_velocity = daran_motor_state[2].speed;
}

/**
 * @brief 鎵归噺鏇存柊鎵€鏈夌數鏈虹姸鎬佹暟鎹?
 * @retval 鏃?
 * @note   1. 鍏堟洿鏂板噷缁勭數鏈烘暟鎹紱
 *         2. 寤舵椂1ms鍚庢洿鏂板ぇ鐒剁數鏈烘暟鎹紱
 *         3. 澶ф芳鐢垫満鏁版嵁鏇存柊閫昏緫鏆傛湭瀹炵幇銆?
 */
void Arm_All_Data_update()
{
    Arm_Linzu_Data_update();
    HAL_Delay(1);
    Arm_Daran_Data_update();
}

/**
 * @brief 鏈烘鑷傜數鏈烘帶鍒舵寚浠ゅ彂閫佸嚱鏁?
 * @retval 鏃?
 * @note   1. 鏍规嵁g_ShoulderType閫夋嫨鍙戦€佸噷缁?澶х劧鐢垫満鎺у埗鎸囦护锛?
 *         2. 鍑岀粍鐢垫満锛氫緷娆″彂閫?/2/3鍙锋寚浠わ紝闂撮殧1ms FreeRTOS寤舵椂锛?
 *         3. 澶х劧鐢垫満锛氫緷娆″彂閫?/2/3鍙锋寚浠わ紝闂撮殧1ms FreeRTOS寤舵椂锛?
 *         4. 澶ф芳鐢垫満鎺у埗鎸囦护宸叉敞閲婏紝濡傞渶鍚敤鍙彇娑堟敞閲娿€?
 */
void Arm_all_tx()
{
    if (g_ShoulderType == SHOULDER_TYPE_LINGZU)
    {
        Arm_Linzu_motor1();
        osDelay(1);
        Arm_Linzu_motor2();
        osDelay(1);
        Arm_Linzu_motor3();
        osDelay(1);
    }
    else if (g_ShoulderType == SHOULDER_TYPE_DARAN)
    {
        Arm_Daran_motor1();
        osDelay(1);
        Arm_Daran_motor2();
        osDelay(1);
        Arm_Daran_motor3();
        osDelay(1);
    }

    // Arm_Damiao_motor4();
    // osDelay(1);
    // Arm_Damiao_motor5();
    // osDelay(1);
    // Arm_Damiao_motor6();
    // osDelay(1);
}
