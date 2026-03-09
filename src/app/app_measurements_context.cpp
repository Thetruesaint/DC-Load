#include "app_measurements_context.h"

namespace {
float currentA = 0.0f;
float voltageV = 0.0f;
int temperatureC = 1;
}

float app_measurements_current_a() {
  return currentA;
}

void app_measurements_set_current_a(float value) {
  currentA = value;
}

float app_measurements_voltage_v() {
  return voltageV;
}

void app_measurements_set_voltage_v(float value) {
  voltageV = value;
}

int app_measurements_temp_c() {
  return temperatureC;
}

void app_measurements_set_temp_c(int value) {
  temperatureC = value;
}

float app_measurements_power_w() {
  return app_measurements_voltage_v() * app_measurements_current_a();
}
