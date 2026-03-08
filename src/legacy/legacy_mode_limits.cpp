#include "legacy_mode_limits.h"

#include "../variables.h"
#include "../ui_lcd.h"
#include "../funciones.h"
#include "../app/app_value_input.h"

void legacy_config_limits() {
  Load_OFF();
  legacy_show_limits();
  delay(2000);
  clearLCD();

  printLCD(4, 0, F("Set Limits"));
  printLCD(0, 1, F("Current(A):"));
  z = 12;
  r = 1;
  if (!Value_Input(z, r)) {
    return;
  }
  CurrentCutOff = constrain(x, 1, MAX_CURRENT);
  printLCDNumber(z, r, CurrentCutOff, ' ', 3);
  printLCDRaw(F("A"));

  printLCD(0, 2, F("Power(W):"));
  r = 2;
  z = 12;
  if (!Value_Input(z, r)) {
    return;
  }
  PowerCutOff = constrain(x, 1, MAX_POWER);
  printLCDNumber(z, r, PowerCutOff, 'W', 1);

  printLCD(0, 3, F("Temp.("));
  printLCD_S(6, 3, String((char)0xDF) + "C):");
  z = 12;
  r = 3;
  if (!Value_Input(z, r, 2)) {
    return;
  }
  tempCutOff = constrain(x, 30.0, MAX_TEMP);
  printLCD_S(z, r, String(tempCutOff));

  Save_EEPROM(ADD_CURRENT_CUT_OFF, CurrentCutOff);
  Save_EEPROM(ADD_POWER_CUT_OFF, PowerCutOff);
  Save_EEPROM(ADD_TEMP_CUT_OFF, tempCutOff);

  legacy_show_limits();
  delay(2000);
  modeInitialized = false;
}

void legacy_show_limits() {
  clearLCD();

#ifndef WOKWI_SIMULATION
  CurrentCutOff = Load_EEPROM(ADD_CURRENT_CUT_OFF);
  PowerCutOff = Load_EEPROM(ADD_POWER_CUT_OFF);
  tempCutOff = Load_EEPROM(ADD_TEMP_CUT_OFF);
#endif

  printLCD(1, 0, F("Limits"));
  printLCD(0, 1, F("Current:"));
  printLCDNumber(9, 1, CurrentCutOff, ' ', 3);
  printLCDRaw(F("A"));
  printLCD(0, 2, F("Power:"));
  printLCDNumber(9, 2, PowerCutOff, 'W', 2);
  printLCD(0, 3, F("Temp.:"));
  printLCDNumber(9, 3, tempCutOff, ' ', 0);
  printLCDRaw(char(0xDF));
  printLCDRaw("C");
}
