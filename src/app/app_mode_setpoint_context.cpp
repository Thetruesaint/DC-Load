#include "app_mode_setpoint_context.h"

#include "../variables.h"

float app_mode_setpoint_power_w() {
  return setPower;
}

void app_mode_setpoint_set_power_w(float value) {
  setPower = value;
}

float app_mode_setpoint_resistance_ohm() {
  return setResistance;
}

void app_mode_setpoint_set_resistance_ohm(float value) {
  setResistance = value;
}
