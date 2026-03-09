#include "legacy_mode_limits.h"

#include "../variables.h"
#include "../ui_lcd.h"
#include "../funciones.h"
#include "../app/app_mode_state_context.h"
#include "../app/app_limits_context.h"
#include "../app/app_value_input.h"
#include "../app/app_value_result_context.h"

void legacy_config_limits() {
  Load_OFF();
  legacy_show_limits();
  delay(2000);
  clearLCD();

  printLCD(4, 0, F("Set Limits"));
  printLCD(0, 1, F("Current(A):"));
  const int col = 12;
  int row = 1;
  if (!Value_Input(col, row)) {
    return;
  }
  app_limits_set_current_cutoff(constrain(app_value_result_get(), 1, MAX_CURRENT));
  printLCDNumber(col, row, app_limits_current_cutoff(), ' ', 3);
  printLCDRaw(F("A"));

  printLCD(0, 2, F("Power(W):"));
  row = 2;
  if (!Value_Input(col, row)) {
    return;
  }
  app_limits_set_power_cutoff(constrain(app_value_result_get(), 1, MAX_POWER));
  printLCDNumber(col, row, app_limits_power_cutoff(), 'W', 1);

  printLCD(0, 3, F("Temp.("));
  printLCD_S(6, 3, String((char)0xDF) + "C):");
  row = 3;
  if (!Value_Input(col, row, 2)) {
    return;
  }
  app_limits_set_temp_cutoff(constrain(app_value_result_get(), 30.0f, MAX_TEMP));
  printLCD_S(col, row, String(app_limits_temp_cutoff()));

  Save_EEPROM(ADD_CURRENT_CUT_OFF, app_limits_current_cutoff());
  Save_EEPROM(ADD_POWER_CUT_OFF, app_limits_power_cutoff());
  Save_EEPROM(ADD_TEMP_CUT_OFF, app_limits_temp_cutoff());

  legacy_show_limits();
  delay(2000);
  app_mode_state_set_initialized(false);
}

void legacy_show_limits() {
  clearLCD();

#ifndef WOKWI_SIMULATION
  app_limits_set_current_cutoff(Load_EEPROM(ADD_CURRENT_CUT_OFF));
  app_limits_set_power_cutoff(Load_EEPROM(ADD_POWER_CUT_OFF));
  app_limits_set_temp_cutoff(Load_EEPROM(ADD_TEMP_CUT_OFF));
#endif

  printLCD(1, 0, F("Limits"));
  printLCD(0, 1, F("Current:"));
  printLCDNumber(9, 1, app_limits_current_cutoff(), ' ', 3);
  printLCDRaw(F("A"));
  printLCD(0, 2, F("Power:"));
  printLCDNumber(9, 2, app_limits_power_cutoff(), 'W', 2);
  printLCD(0, 3, F("Temp.:"));
  printLCDNumber(9, 3, app_limits_temp_cutoff(), ' ', 0);
  printLCDRaw(char(0xDF));
  printLCDRaw("C");
}


