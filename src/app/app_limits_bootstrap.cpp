#include "app_limits_bootstrap.h"

#include "../config/system_constants.h"
#include "../storage_eeprom.h"
#include "../ui/ui_mode_templates.h"
#include "app_limits_context.h"

bool app_limits_values_are_valid(float currentCutoffA, float powerCutoffW, float tempCutoffC) {
  return currentCutoffA > 1.0f && currentCutoffA <= MAX_CURRENT &&
         powerCutoffW > 1.0f && powerCutoffW <= MAX_POWER &&
         tempCutoffC >= 30.0f && tempCutoffC <= 99.0f;
}

bool app_limits_current_values_are_valid() {
  return app_limits_values_are_valid(
      app_limits_current_cutoff(),
      app_limits_power_cutoff(),
      app_limits_temp_cutoff());
}

void app_limits_load_from_eeprom() {
#ifndef WOKWI_SIMULATION
  app_limits_set_current_cutoff(Load_EEPROM(ADD_CURRENT_CUT_OFF));
  app_limits_set_power_cutoff(Load_EEPROM(ADD_POWER_CUT_OFF));
  app_limits_set_temp_cutoff(Load_EEPROM(ADD_TEMP_CUT_OFF));
#endif
}

void app_limits_save_to_eeprom() {
#ifndef WOKWI_SIMULATION
  Save_EEPROM(ADD_CURRENT_CUT_OFF, app_limits_current_cutoff());
  Save_EEPROM(ADD_POWER_CUT_OFF, app_limits_power_cutoff());
  Save_EEPROM(ADD_TEMP_CUT_OFF, app_limits_temp_cutoff());
#endif
}

void app_limits_show_summary() {
  app_limits_load_from_eeprom();
  ui_draw_limits_summary(
      app_limits_current_cutoff(),
      app_limits_power_cutoff(),
      app_limits_temp_cutoff());
}
