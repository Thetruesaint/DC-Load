#include "app_mode_setpoint_context.h"

namespace {
float setPowerW = 0.0f;
float setResistanceOhm = 0.0f;
}

float app_mode_setpoint_power_w() {
  return setPowerW;
}

void app_mode_setpoint_set_power_w(float value) {
  setPowerW = value;
}

float app_mode_setpoint_resistance_ohm() {
  return setResistanceOhm;
}

void app_mode_setpoint_set_resistance_ohm(float value) {
  setResistanceOhm = value;
}
