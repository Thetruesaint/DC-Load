#include "app_keypad.h"

#include <cmath>

#include "../config/system_constants.h"
#include "../core/core_engine.h"
#include "app_input_buffer.h"
#include "app_loop.h"
#include "app_mode_context.h"
#include "app_msc.h"
#include "app_ui_context.h"

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

  const MscKeyDecision mscDecision = app_handle_msc_key_decision(key);
  if (mscDecision != MscKeyDecision::Continue) {
    if (mscDecision == MscKeyDecision::ExitMode) {
      app_ui_request_clear_cursor_blink();
      return;
    }
    if (mscDecision == MscKeyDecision::OpenConfig) {
      app_push_action(make_open_limits_config_action());
      return;
    }
    if (mscDecision == MscKeyDecision::ToggleTraceOverlay) {
      app_push_action(make_toggle_trace_overlay_action());
      return;
    }
    return;
  }

  // While UI is in a config/menu screen, route non-MSC keys only as actions.
  const bool inMenu = core_get_state().uiScreen != UiScreen::Home;
  if (inMenu) {
    app_push_action(make_key_pressed_action(key));
    return;
  }

  app_push_action(make_key_pressed_action(key));

  const uint8_t mode = app_mode_id();
  const bool transientHomePeriodInput = (mode == TC);
  if (app_mode_is_transient() && !transientHomePeriodInput) return;

  app_input_append_digit(key, maxDigits);

  if (key == '.' && !transientHomePeriodInput) {
    app_input_append_decimal(maxDigits);
  }

  if (key == 'E' && app_input_length() != 0) {
    if (transientHomePeriodInput) {
      const int32_t parsedPeriod = static_cast<int32_t>(lroundf(app_input_parse_float()));
      app_push_action(make_value_confirm_action(parsedPeriod));
    } else {
      const float parsedValue = app_input_parse_float();
      const int32_t parsedMilli = static_cast<int32_t>(lroundf(parsedValue * 1000.0f));
      app_push_action(make_value_confirm_action(parsedMilli));
    }
    app_input_reset();
  }

  if (key == '<') {
    app_input_backspace();
  }
}
