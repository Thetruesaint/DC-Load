#include "legacy_mode_cp.h"

#include "../variables.h"
#include "../ui_lcd.h"
#include "../funciones.h"
#include "../app/app_mode_state_context.h"
#include "../app/app_runtime_context.h"
#include "../app/app_setpoint_context.h"

void legacy_const_power_mode() {
  if (!app_mode_state_initialized()) {
    clearLCD();
    printLCD(0, 0, F("CP LOAD"));
    printLCD(0, 2, F("Set->"));
    printLCD(11, 2, F("W"));
    printLCD(0, 3, F(">"));
    Encoder_Status(true, PowerCutOff);
    app_mode_state_set_initialized(true);
  }

  float readingValue = app_runtime_encoder_position() / 1000.0f;
  readingValue = min(app_setpoint_max_reading(), max(0.0f, readingValue));
  app_setpoint_set_reading(readingValue);
  app_runtime_set_encoder_position(readingValue * 1000.0f);
  Cursor_Position();

  // Setpoint de CP lo calcula core y se aplica via legacy_apply_state.
}
