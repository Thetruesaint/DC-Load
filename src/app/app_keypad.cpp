#include "app_keypad.h"

#include <Keypad.h>

#include "../ui_lcd.h"
#include "../legacy/legacy_hooks.h"
#include "app_msc.h"
#include "app_mode_context.h"
#include "app_input_buffer.h"
#include "app_runtime_context.h"
#include "app_setpoint_context.h"
#include "../core/core_modes.h"
#include "app_loop.h"

void app_read_keypad(int col, int row) {
  int maxDigits = app_mode_is_calibration() ? 6 : 5;
  char key = app_input_read_key();

  if (key == NO_KEY) return;

  app_push_action(ActionType::KeyPressed, 0, key);

  if (!app_handle_msc_keys(key)) {
    return;
  }

  if (app_mode_is_transient()) return;

  if (key == 'U') {
    if (core_mode_is_managed(app_mode_id())) return;
    float encoderPosition = app_runtime_encoder_position();
    encoderPosition = encoderPosition + app_runtime_encoder_step();
    encoderPosition = constrain(encoderPosition, 0.0f, static_cast<float>(app_runtime_encoder_max()));
    app_runtime_set_encoder_position(encoderPosition);
    return;
  }

  if (key == 'D') {
    if (core_mode_is_managed(app_mode_id())) return;
    float encoderPosition = app_runtime_encoder_position();
    encoderPosition = encoderPosition - app_runtime_encoder_step();
    app_runtime_set_encoder_position(encoderPosition);
    return;
  }

  if (key == 'L') {
    if (core_mode_is_managed(app_mode_id())) return;
    app_runtime_set_cursor_position(app_runtime_cursor_position() - 1);
    return;
  }

  if (key == 'R') {
    if (core_mode_is_managed(app_mode_id())) return;
    app_runtime_set_cursor_position(app_runtime_cursor_position() + 1);
    return;
  }

  if (app_mode_is_battery()) return;

  if (app_input_append_digit(key, maxDigits)) {
    printLCD_S(col + app_input_length() - 1, row, String(key));
  }

  if (key == '.' && app_input_append_decimal(maxDigits)) {
    printLCD(col + app_input_length() - 1, 3, F("."));
  }

  if (key == 'E' && app_input_length() != 0) {
    const float parsedValue = app_input_parse_float();
    if (!app_mode_is_calibration()) {
      app_setpoint_set_reading(parsedValue);
      app_runtime_set_encoder_position(parsedValue * 1000.0f);
    } else {
      legacy_calibrate(parsedValue);
    }
    Print_Spaces(col, row, maxDigits);
    app_input_reset();
  }

  if (key == '<' && app_input_backspace()) {
    Print_Spaces(col + app_input_length(), row);
  }
}
