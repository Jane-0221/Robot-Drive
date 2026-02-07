//
// Created by RM UI Designer
// Dynamic Edition
//

#include "string.h"
#include "ui_interface.h"
#include "ui_g.h"

#define TOTAL_FIGURE 3
#define TOTAL_STRING 0

ui_interface_figure_t ui_g_now_figures[TOTAL_FIGURE];
uint8_t ui_g_dirty_figure[TOTAL_FIGURE];

#ifndef MANUAL_DIRTY
ui_interface_figure_t ui_g_last_figures[TOTAL_FIGURE];
#endif

#define SCAN_AND_SEND() ui_scan_and_send(ui_g_now_figures, ui_g_dirty_figure, NULL, NULL, TOTAL_FIGURE, TOTAL_STRING)

void ui_init_g() {
    ui_g_1_BOOM1->figure_type = 2;
    ui_g_1_BOOM1->operate_type = 1;
    ui_g_1_BOOM1->layer = 0;
    ui_g_1_BOOM1->color = 4;
    ui_g_1_BOOM1->start_x = 114;
    ui_g_1_BOOM1->start_y = 813;
    ui_g_1_BOOM1->width = 10;
    ui_g_1_BOOM1->r = 33;

    ui_g_2_BOOM2->figure_type = 2;
    ui_g_2_BOOM2->operate_type = 1;
    ui_g_2_BOOM2->layer = 0;
    ui_g_2_BOOM2->color = 4;
    ui_g_2_BOOM2->start_x = 114;
    ui_g_2_BOOM2->start_y = 713;
    ui_g_2_BOOM2->width = 10;
    ui_g_2_BOOM2->r = 35;

    ui_g_3_BOOM3->figure_type = 3;
    ui_g_3_BOOM3->operate_type = 1;
    ui_g_3_BOOM3->layer = 0;
    ui_g_3_BOOM3->color = 4;
    ui_g_3_BOOM3->start_x = 113;
    ui_g_3_BOOM3->start_y = 599;
    ui_g_3_BOOM3->width = 10;
    ui_g_3_BOOM3->rx = 33;
    ui_g_3_BOOM3->ry = 33;

    uint32_t idx = 0;
    for (int i = 0; i < TOTAL_FIGURE; i++) {
        ui_g_now_figures[i].figure_name[2] = idx & 0xFF;
        ui_g_now_figures[i].figure_name[1] = (idx >> 8) & 0xFF;
        ui_g_now_figures[i].figure_name[0] = (idx >> 16) & 0xFF;
        ui_g_now_figures[i].operate_type = 1;
#ifndef MANUAL_DIRTY
        ui_g_last_figures[i] = ui_g_now_figures[i];
#endif
        ui_g_dirty_figure[i] = 1;
        idx++;
    }

    SCAN_AND_SEND();

    for (int i = 0; i < TOTAL_FIGURE; i++) {
        ui_g_now_figures[i].operate_type = 2;
    }
}

void ui_update_g() {
#ifndef MANUAL_DIRTY
    for (int i = 0; i < TOTAL_FIGURE; i++) {
        if (memcmp(&ui_g_now_figures[i], &ui_g_last_figures[i], sizeof(ui_g_now_figures[i])) != 0) {
            ui_g_dirty_figure[i] = 1;
            ui_g_last_figures[i] = ui_g_now_figures[i];
        }
    }
#endif
    SCAN_AND_SEND();
}
