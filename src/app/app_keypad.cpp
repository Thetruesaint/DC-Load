#include "app_keypad.h"

#include <Keypad.h>
#include <cmath>

#include "../ui_lcd.h"
#include "app_msc.h"
#include "app_mode_context.h"
#include "app_input_buffer.h"
#include "app_loop.h"

void app_read_keypad(int col, int row) {
  int maxDigits = app_mode_is_calibration() ? 6 : 5;
  char key = app_input_read_key();

  if (key == NO_KEY) return;

  app_push_action(make_key_pressed_action(key));

  if (!app_handle_msc_keys(key)) {
    return;
  }

  if (app_mode_is_transient()) return;

  if (app_mode_is_battery()) return;

  if (app_input_append_digit(key, maxDigits)) {
    printLCD_S(col + app_input_length() - 1, row, String(key));
  }

  if (key == '.' && app_input_append_decimal(maxDigits)) {
    printLCD(col + app_input_length() - 1, 3, F("."));
  }

  if (key == 'E' && app_input_length() != 0) {
    const float parsedValue = app_input_parse_float();
    const int32_t parsedMilli = static_cast<int32_t>(lroundf(parsedValue * 1000.0f));
    app_push_action(make_value_confirm_action(parsedMilli));
    Print_Spaces(col, row, maxDigits);
    app_input_reset();
  }

  if (key == '<' && app_input_backspace()) {
    Print_Spaces(col + app_input_length(), row);
  }
}
