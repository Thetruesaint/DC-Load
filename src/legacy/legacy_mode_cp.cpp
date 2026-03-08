#include "legacy_mode_cp.h"

#include "../variables.h"
#include "../ui_lcd.h"
#include "../funciones.h"

void legacy_const_power_mode() {
  if (!modeInitialized) {
    clearLCD();
    printLCD(0, 0, F("CP LOAD"));
    printLCD(0, 2, F("Set->"));
    printLCD(11, 2, F("W"));
    printLCD(0, 3, F(">"));
    Encoder_Status(true, PowerCutOff);
    modeInitialized = true;
  }

  reading = encoderPosition / 1000;
  reading = min(maxReading, max(0.0f, reading));
  encoderPosition = reading * 1000.0;
  Cursor_Position();

  // Setpoint de CP lo calcula core y se aplica via legacy_apply_state.
}
