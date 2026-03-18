#include "app_setpoint_cursor.h"

#include "../config/system_constants.h"
#include "../core/core_modes.h"
#include "../ui_display.h"
#include "app_inputs.h"
#include "app_io_context.h"
#include "app_loop.h"
#include "app_mode_context.h"
#include "app_runtime_context.h"

void app_setpoint_cursor_update() {
  static uint32_t lastPressTime = 0;
  constexpr int unitPosition = 8;
  static int lastCursor = -1;

  int cursor = app_runtime_cursor_position();
  const uint8_t mode = app_mode_id();
  const bool managedMode = core_mode_is_managed(mode);
  const uint32_t nowMs = app_io_millis();

  if (app_io_encoder_button_low() && (nowMs - lastPressTime > 200)) {
    lastPressTime = nowMs;
    if (managedMode) {
      app_push_action(make_encoder_button_press_action());
    } else {
      cursor++;
      app_runtime_set_cursor_position(cursor);
    }
  }

  if (lastCursor == cursor) return;

  if (managedMode) {
    lastCursor = cursor;
    return;
  }

  if (cursor > lastCursor && cursor == unitPosition + 1) cursor++;
  if (cursor < lastCursor && cursor == unitPosition + 1) cursor--;

  if ((mode == CC || mode == BC || mode == CA) && cursor > 12) cursor = unitPosition;
  if ((mode == CC || mode == BC || mode == CA) && cursor < 8) cursor = unitPosition + 4;
  if ((mode == CP || mode == CR) && cursor > 10) cursor = unitPosition - 2;
  if ((mode == CP || mode == CR) && cursor < 6) cursor = unitPosition + 2;

  switch (cursor) {
    case 6: app_runtime_set_encoder_step(100000); break;
    case 7: app_runtime_set_encoder_step(10000); break;
    case 10: app_runtime_set_encoder_step(100); break;
    case 11: app_runtime_set_encoder_step(10); break;
    case 12: app_runtime_set_encoder_step(1); break;
    default: app_runtime_set_encoder_step(1000);
  }

  app_runtime_set_cursor_position(cursor);
  lastCursor = cursor;

  uiGridSetCursor(cursor, 2);
}

