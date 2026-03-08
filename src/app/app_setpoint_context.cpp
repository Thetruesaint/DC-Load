#include "app_setpoint_context.h"

#include "../variables.h"

float app_setpoint_reading() {
  return reading;
}

void app_setpoint_set_reading(float value) {
  reading = value;
}

float app_setpoint_max_reading() {
  return maxReading;
}

void app_setpoint_set_max_reading(float value) {
  maxReading = value;
}
