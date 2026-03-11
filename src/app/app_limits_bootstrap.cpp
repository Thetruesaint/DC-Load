#include "app_limits_bootstrap.h"

#include "../config/system_constants.h"
#include "../storage_eeprom.h"
#include "../ui/ui_mode_templates.h"
#include "app_limits_context.h"

void app_limits_load_from_eeprom() {
#ifndef WOKWI_SIMULATION
  app_limits_set_current_cutoff(Load_EEPROM(ADD_CURRENT_CUT_OFF));
  app_limits_set_power_cutoff(Load_EEPROM(ADD_POWER_CUT_OFF));
  app_limits_set_temp_cutoff(Load_EEPROM(ADD_TEMP_CUT_OFF));
#endif
}

void app_limits_show_summary() {
  app_limits_load_from_eeprom();
  ui_draw_limits_summary(
      app_limits_current_cutoff(),
      app_limits_power_cutoff(),
      app_limits_temp_cutoff());
}
