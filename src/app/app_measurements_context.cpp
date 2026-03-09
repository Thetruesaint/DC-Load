#include "app_measurements_context.h"

#include "../variables.h"

float app_measurements_current_a() {
  return current;
}

void app_measurements_set_current_a(float value) {
  current = value;
}

float app_measurements_voltage_v() {
  return voltage;
}

void app_measurements_set_voltage_v(float value) {
  voltage = value;
}

int app_measurements_temp_c() {
  return temp;
}

void app_measurements_set_temp_c(int value) {
  temp = value;
}

float app_measurements_power_w() {
  return app_measurements_voltage_v() * app_measurements_current_a();
}
