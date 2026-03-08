#include "legacy_mode_cr.h"

#include "../variables.h"
#include "../ui_lcd.h"
#include "../funciones.h"
#include "../app/app_runtime_context.h"
#include "../app/app_setpoint_context.h"

void legacy_const_resistance_mode() {
  if (!modeInitialized) {
    clearLCD();
    printLCD(0, 0, F("CR LOAD"));
    printLCD(0, 2, F("Set->"));
    printLCD_S(11, 2, String((char)0xF4));
    printLCD(0, 3, F(">"));
    Encoder_Status(true, MAX_RESISTOR);
    app_runtime_set_encoder_position(MAX_RESISTOR * 1000.0f);
    modeInitialized = true;
  }

  float readingValue = app_runtime_encoder_position() / 1000.0f;
  readingValue = min(app_setpoint_max_reading(), max(0.1f, readingValue));
  app_setpoint_set_reading(readingValue);
  app_runtime_set_encoder_position(readingValue * 1000.0f);
  Cursor_Position();

  // Setpoint de CR lo calcula core y se aplica via legacy_apply_state.
}
