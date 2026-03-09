#ifndef UI_MODE_TEMPLATES_H
#define UI_MODE_TEMPLATES_H

#include <Arduino.h>

void ui_draw_cc_template();
void ui_draw_cp_template();
void ui_draw_cr_template();
void ui_draw_bc_template(float cutoffVolts, float batteryLife, const String &batteryType);

#endif
