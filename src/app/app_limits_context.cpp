#include "app_limits_context.h"

#include "../variables.h"

namespace {
float currentCutoff = MAX_CURRENT;
float powerCutoff = MAX_POWER;
float temperatureCutoff = MAX_TEMP;
}

float app_limits_current_cutoff() {
  return currentCutoff;
}

void app_limits_set_current_cutoff(float value) {
  currentCutoff = value;
}

float app_limits_power_cutoff() {
  return powerCutoff;
}

void app_limits_set_power_cutoff(float value) {
  powerCutoff = value;
}

float app_limits_temp_cutoff() {
  return temperatureCutoff;
}

void app_limits_set_temp_cutoff(float value) {
  temperatureCutoff = value;
}

