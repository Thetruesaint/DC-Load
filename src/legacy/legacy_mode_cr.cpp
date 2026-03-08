#include "legacy_mode_cr.h"

#include "../variables.h"
#include "../ui_lcd.h"
#include "../funciones.h"

void legacy_const_resistance_mode() {
  if (!modeInitialized) {
    clearLCD();
    printLCD(0, 0, F("CR LOAD"));
    printLCD(0, 2, F("Set->"));
    printLCD_S(11, 2, String((char)0xF4));
    printLCD(0, 3, F(">"));
    Encoder_Status(true, MAX_RESISTOR);
    encoderPosition = MAX_RESISTOR * 1000;
    modeInitialized = true;
  }

  reading = encoderPosition / 1000.0;
  reading = min(maxReading, max(0.1f, reading));
  encoderPosition = reading * 1000;
  Cursor_Position();

  // Setpoint de CR lo calcula core y se aplica via legacy_apply_state.
}
