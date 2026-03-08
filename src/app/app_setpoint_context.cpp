#include "app_setpoint_context.h"

#include "../variables.h"

float app_setpoint_reading() {
  return reading;
}

void app_setpoint_set_reading(float value) {
  reading = value;
}
