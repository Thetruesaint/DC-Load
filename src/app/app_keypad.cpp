#include "app_keypad.h"

#include <cmath>

#include "app_input_buffer.h"
#include "app_loop.h"
#include "app_mode_context.h"
#include "app_msc.h"

void app_read_keypad(int col, int row) {
  (void)col;
  (void)row;

  static uint8_t lastMode = app_mode_id();
  const uint8_t currentMode = app_mode_id();
  if (currentMode != lastMode) {
    app_input_reset();
    lastMode = currentMode;
  }

  int maxDigits = app_mode_is_calibration() ? 6 : 5;
  char key = app_input_read_key();

  if (app_input_is_no_key(key)) return;

  app_push_action(make_key_pressed_action(key));

  if (!app_handle_msc_keys(key)) {
    return;
  }

  if (app_mode_is_transient()) return;

  if (app_mode_is_battery()) return;

  app_input_append_digit(key, maxDigits);

  if (key == '.') {
    app_input_append_decimal(maxDigits);
  }

  if (key == 'E' && app_input_length() != 0) {
    const float parsedValue = app_input_parse_float();
    const int32_t parsedMilli = static_cast<int32_t>(lroundf(parsedValue * 1000.0f));
    app_push_action(make_value_confirm_action(parsedMilli));
    app_input_reset();
  }

  if (key == '<') {
    app_input_backspace();
  }
}
