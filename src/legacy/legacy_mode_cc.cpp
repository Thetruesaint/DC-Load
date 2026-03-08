#include "legacy_mode_cc.h"

#include "../variables.h"
#include "../ui_lcd.h"
#include "../funciones.h"

void legacy_const_current_mode() {
  if (!modeInitialized) {
    clearLCD();
    printLCD(0, 0, F("CC LOAD"));
    printLCD(1, 2, F("Set->"));
    printLCD(13, 2, F("A"));
    printLCD(0, 3, F(">"));
    Encoder_Status(true, CurrentCutOff);
    modeInitialized = true;
  }

  reading = encoderPosition / 1000;
  reading = min(maxReading, max(0.0f, reading));
  encoderPosition = reading * 1000.0;
  Cursor_Position();

  // Setpoint de CC lo calcula core y se aplica via legacy_apply_state.
}
