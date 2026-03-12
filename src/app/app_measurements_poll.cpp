#include "app_measurements_poll.h"

#include "../config/system_constants.h"
#include "../hw/hw_objects.h"
#include "app_calibration_context.h"
#include "app_io_context.h"
#include "app_load_context.h"
#include "app_measurements_context.h"
#include "app_mode_context.h"

namespace {
float apply_voltage_calibration(float rawVoltage) {
  return rawVoltage * app_calibration_sns_volt_factor_ref() + app_calibration_sns_volt_offset_ref();
}

float apply_current_calibration(float rawCurrent) {
  return rawCurrent * app_calibration_sns_curr_factor_ref() + app_calibration_sns_curr_offset_ref();
}
}

void app_measurements_poll() {
#ifndef WOKWI_SIMULATION
  ads.setGain(GAIN_TWOTHIRDS);
  const int16_t adcv = ads.readADC_SingleEnded(VLTG_SNSR);
  const float rawVoltage = ads.computeVolts(adcv) * SNS_VOLT_FACT;
  app_measurements_set_voltage_v(apply_voltage_calibration(rawVoltage));

  ads.setGain(GAIN_ONE);
  const int16_t adci = ads.readADC_SingleEnded(CRR_SNSR);
  const float rawCurrent = ads.computeVolts(adci) * SNS_CURR_FACT;
  app_measurements_set_current_a(apply_current_calibration(rawCurrent));
#else
  const int potValue = analogRead(VSIM);
  static float simulatedVoltage = 0.0f;
  static unsigned long lastDecreaseTime = 0;
  const unsigned long currentMillis = app_io_millis();
  const bool loadEnabled = app_load_is_enabled();

  constexpr float SIM_VOLTAGE_MAX = 40.0f;
  constexpr float ESP32_ADC_MAX = 4095.0f;
  const float simulatedPotVoltage = constrain((static_cast<float>(potValue) / ESP32_ADC_MAX) * SIM_VOLTAGE_MAX, 0.0f, SIM_VOLTAGE_MAX);

  if (!app_mode_is_battery() && !app_mode_is_calibration()) {
    simulatedVoltage = simulatedPotVoltage;
    app_measurements_set_voltage_v(simulatedVoltage);
  } else if (app_mode_is_battery()) {
    if (loadEnabled && (currentMillis - lastDecreaseTime >= 2000)) {
      lastDecreaseTime = currentMillis;
      simulatedVoltage -= 0.005f;
      simulatedVoltage = max(simulatedVoltage, 0.0f);
      app_measurements_set_voltage_v(simulatedVoltage);
    } else if (!loadEnabled) {
      simulatedVoltage = simulatedPotVoltage;
      app_measurements_set_voltage_v(simulatedVoltage);
    }
  } else if (app_mode_is_calibration()) {
    simulatedVoltage = simulatedPotVoltage;
    const float errorVoltage = simulatedVoltage * 1.05f - 0.1f;
    app_measurements_set_voltage_v(apply_voltage_calibration(errorVoltage));
  }

  if (loadEnabled) {
    app_measurements_set_current_a(app_load_set_current_mA() / 1000.0f);
  } else {
    app_measurements_set_current_a(0.0f);
  }
#endif
}
