//
// Created by RM UI Designer
// Dynamic Edition
//

#ifndef UI_g_H
#define UI_g_H

#include "ui_interface.h"

extern ui_interface_figure_t ui_g_now_figures[3];
extern uint8_t ui_g_dirty_figure[3];

#define ui_g_1_BOOM1 ((ui_interface_round_t*)&(ui_g_now_figures[0]))
#define ui_g_2_BOOM2 ((ui_interface_round_t*)&(ui_g_now_figures[1]))
#define ui_g_3_BOOM3 ((ui_interface_ellipse_t*)&(ui_g_now_figures[2]))


#ifdef MANUAL_DIRTY
#define ui_g_1_BOOM1_dirty (ui_g_dirty_figure[0])
#define ui_g_2_BOOM2_dirty (ui_g_dirty_figure[1])
#define ui_g_3_BOOM3_dirty (ui_g_dirty_figure[2])

#endif

void ui_init_g();
void ui_update_g();

#endif // UI_g_H
