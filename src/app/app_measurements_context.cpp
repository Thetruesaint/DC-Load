#include "app_measurements_context.h"

#include "../config/system_constants.h"
#include "app_calibration_context.h"

namespace {
float currentA = 0.0f;
float voltageV = 0.0f;
int temperatureC = 1;

float calibrated_temp_from_raw(float rawTempC) {
  return rawTempC * app_calibration_temp_factor_ref();
}
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

float app_measurements_read_temp_raw_c() {
  return analogRead(TEMP_SNSR) * TEMP_CONVERSION_FACTOR;
}

int app_measurements_read_temp_c() {
  const float correctedTempC = calibrated_temp_from_raw(app_measurements_read_temp_raw_c());
  return static_cast<int>(constrain(correctedTempC, 0.0f, MAX_TEMP));
}

float app_measurements_power_w() {
  const float power = app_measurements_voltage_v() * app_measurements_current_a();
  return (power < 0.0f) ? 0.0f : power;
}
