#include "ui.h"
#include "FreeRTOS.h"
#include "cmsis_os2.h"
#include "Global_status.h"
#include "referee_system.h"
#include "remote_control.h"
void ui_init()
{
  ui_init_g();
  osDelay(20);
  // _ui_init_g_BOOM_0();
  // osDelay(20);
  // _ui_init_g_BOOM_1();
  // osDelay(20);
  // _ui_init_g_BOOM_2();
  // osDelay(20);
  // _ui_init_g_BOOM_3();
  // osDelay(20);
  // _ui_init_g_MODE_0();
  // osDelay(20);
  // _ui_init_g_STATE_0();
  //   osDelay(20);

}

void ui_updata()
{
  ui_update_g();
   osDelay(5);
}

void Supercapui_change(float votage)
{

}

void BOOMui_change()
{
if(boom_arm_status == ARM_BOOM_OFF){  //关闭紫色
  ui_g_1_BOOM1->color = 4;
  ui_g_2_BOOM2->color = 4;
  ui_g_3_BOOM3->color = 4;
}
else{                                //开启绿色
  ui_g_1_BOOM1->color = 2;
  ui_g_2_BOOM2->color = 2;
  ui_g_3_BOOM3->color = 2;
}
}

void Chassisui_change(float angle)
{
  // float angle_start, angle_end;
  // if (angle > 360)
  //   angle -= 360;
  // angle_start = angle - 20.0f;
  // angle_end = angle + 20.0f;
  // if (angle_start <= 0.0f)
  //   angle_start += 360.0f;
  // if (angle_end <= 0.0f)
  //   angle_end += 360.0f;
  // if (angle_end >= 360.0f)
  //   angle_end -= 360.0f;
  // if (angle_start >= 360.0f)
  //   angle_start -= 360.0f;
  // ui_Omni_Chassis_high_Chassis_arc->start_angle = angle_start;
  // ui_Omni_Chassis_high_Chassis_arc->end_angle = angle_end;
  // if (Global.Chssis.mode == FLOW)
  //   ui_Omni_Chassis_high_Chassis_arc->color = 1; // 非小陀螺黄色
  // else if (Global.Chssis.mode == SPIN_P)
  //   ui_Omni_Chassis_high_Chassis_arc->color = 2; // 小陀螺绿色
  // else
  //   ui_Omni_Chassis_high_Chassis_arc->color = 1; // 默认黄色

}

void Autoui_change(void)
{
  // static uint8_t auto_online;
  // switch (Global.Auto.input.fire) // 自瞄圈
  // {
  // case 0:
  //   ui_Omni_Shoot_Low_Auto_rect->color = 4; // 紫红色
  //   break;
  // case 1:
  //   ui_Omni_Shoot_Low_Auto_rect->color = 2; // 绿色
  //   break;
  // default:
  //   ui_Omni_Shoot_Low_Auto_rect->color = 4; // 紫红色
  //   break;
  // }
  // if (Global.Auto.input.Auto_control_online == 1)
  //   auto_online = 10;
  // if (Global.Auto.input.Auto_control_online == 0 && auto_online == 0)
  // {
  //   ui_Omni_Shoot_Low_Auto_rect->color = 7; // 掉线黑色
  // }
  // auto_online--;
}
