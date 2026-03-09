#include "../app/app_value_input_view.h"

#include "../ui_lcd.h"

void app_value_input_view_begin(int col, int row) {
  setCursorLCD(col, row);
  blinkOnLCD();
}

void app_value_input_view_render(int col, int row, int maxDigits, const char *text) {
  printLCD_S(col, row, String("     ").substring(0, maxDigits));
  printLCD_S(col, row, String(text));
}

void app_value_input_view_end() {
  noCursorLCD();
  blinkOffLCD();
}
