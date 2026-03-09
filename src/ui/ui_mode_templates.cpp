#include "ui_mode_templates.h"

#include "../ui_lcd.h"

void ui_draw_cc_template() {
  clearLCD();
  printLCD(0, 0, F("CC LOAD"));
  printLCD(1, 2, F("Set->"));
  printLCD(13, 2, F("A"));
  printLCD(0, 3, F(">"));
}

void ui_draw_cp_template() {
  clearLCD();
  printLCD(0, 0, F("CP LOAD"));
  printLCD(0, 2, F("Set->"));
  printLCD(11, 2, F("W"));
  printLCD(0, 3, F(">"));
}

void ui_draw_cr_template() {
  clearLCD();
  printLCD(0, 0, F("CR LOAD"));
  printLCD(0, 2, F("Set->"));
  printLCD_S(11, 2, String((char)0xF4));
  printLCD(0, 3, F(">"));
}

void ui_draw_bc_template(float cutoffVolts, float batteryLife, const String &batteryType) {
  clearLCD();
  printLCD(0, 0, F("BC LOAD"));
  setCursorLCD(13, 1);
  printLCDRaw(F(">"));
  printLCDNumber(14, 1, cutoffVolts, 'V', 2);
  printLCD(1, 2, F("Adj->"));
  printLCD(13, 2, F("A"));
  printLCDNumber(6, 3, batteryLife, ' ', 0);
  printLCDRaw(F("mAh"));
  printLCD_S(14, 3, batteryType);
}

void ui_draw_battery_task_menu() {
  clearLCD();
  printLCD(2, 0, F("Set Task & Batt"));
  printLCD(0, 1, F("Stor. 1)LiPo 2)LiIOn"));
  printLCD(0, 2, F("Disc. 3)LiPo 4)LiIOn"));
  printLCD(2, 3, F("5)Cutoff Voltage"));
}

void ui_draw_battery_custom_cutoff_prompt(const String &batteryType) {
  clearLCD();
  printLCD_S(3, 0, batteryType + " Batt");
  printLCD(2, 1, F("Voltage Cutoff?"));
  printLCD(5, 2, F("(0.1-25)V"));
}

void ui_draw_battery_cell_count_prompt(const String &batteryType) {
  clearLCD();
  printLCD_S(3, 0, batteryType + " Batt");
  printLCD(6, 1, F("(1-6)S?"));
}
