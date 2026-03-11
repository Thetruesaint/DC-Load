#ifndef APP_LIMITS_BOOTSTRAP_H
#define APP_LIMITS_BOOTSTRAP_H

bool app_limits_values_are_valid(float currentCutoffA, float powerCutoffW, float tempCutoffC);
bool app_limits_current_values_are_valid();
void app_limits_load_from_eeprom();
void app_limits_save_to_eeprom();
void app_limits_show_summary();

#endif
