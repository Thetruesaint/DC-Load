#include "legacy_mode_limits.h"

#include "../app/app_limits_context.h"
#include "../app/app_mode_state_context.h"
#include "../app/app_value_input.h"
#include "../app/app_value_result_context.h"
#include "../config/system_constants.h"
#include "../funciones.h"
#include "../ui/ui_mode_templates.h"

void legacy_config_limits() {
  Load_OFF();
  legacy_show_limits();
  delay(2000);

  ui_draw_limits_config_template();

  const int col = 12;
  int row = 1;
  if (!Value_Input(col, row)) {
    return;
  }
  app_limits_set_current_cutoff(constrain(app_value_result_get(), 1, MAX_CURRENT));
  printLCDNumber(col, row, app_limits_current_cutoff(), ' ', 3);
  printLCDRaw(F("A"));

  row = 2;
  if (!Value_Input(col, row)) {
    return;
  }
  app_limits_set_power_cutoff(constrain(app_value_result_get(), 1, MAX_POWER));
  printLCDNumber(col, row, app_limits_power_cutoff(), 'W', 1);

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
#ifndef WOKWI_SIMULATION
  app_limits_set_current_cutoff(Load_EEPROM(ADD_CURRENT_CUT_OFF));
  app_limits_set_power_cutoff(Load_EEPROM(ADD_POWER_CUT_OFF));
  app_limits_set_temp_cutoff(Load_EEPROM(ADD_TEMP_CUT_OFF));
#endif

  ui_draw_limits_summary(
      app_limits_current_cutoff(),
      app_limits_power_cutoff(),
      app_limits_temp_cutoff());
}
