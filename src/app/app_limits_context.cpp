#include "app_limits_context.h"

#include "../variables.h"

float app_limits_current_cutoff() {
  return CurrentCutOff;
}

void app_limits_set_current_cutoff(float value) {
  CurrentCutOff = value;
}

float app_limits_power_cutoff() {
  return PowerCutOff;
}

void app_limits_set_power_cutoff(float value) {
  PowerCutOff = value;
}

float app_limits_temp_cutoff() {
  return tempCutOff;
}

void app_limits_set_temp_cutoff(float value) {
  tempCutOff = value;
}
