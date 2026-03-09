#include "legacy_mode_cc.h"

#include "../config/system_constants.h"
#include "../hw/hw_objects.h"
#include "../ui_lcd.h"
#include "../funciones.h"
#include "../app/app_mode_state_context.h"
#include "../app/app_limits_context.h"
#include "../app/app_runtime_context.h"
#include "../app/app_setpoint_context.h"

void legacy_const_current_mode() {
  if (!app_mode_state_initialized()) {
    clearLCD();
    printLCD(0, 0, F("CC LOAD"));
    printLCD(1, 2, F("Set->"));
    printLCD(13, 2, F("A"));
    printLCD(0, 3, F(">"));
    Encoder_Status(true, app_limits_current_cutoff());
    app_mode_state_set_initialized(true);
  }

  float readingValue = app_runtime_encoder_position() / 1000.0f;
  readingValue = min(app_setpoint_max_reading(), max(0.0f, readingValue));
  app_setpoint_set_reading(readingValue);
  app_runtime_set_encoder_position(readingValue * 1000.0f);
  Cursor_Position();

  // Setpoint de CC lo calcula core y se aplica via legacy_apply_state.
}


