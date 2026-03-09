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
