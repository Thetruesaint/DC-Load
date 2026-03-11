#include "legacy_base_io.h"

#include "../config/system_constants.h"
#include "../hw/hw_objects.h"
#include "../app/app_inputs.h"
#include "../app/app_io_context.h"
#include "../app/app_load_context.h"
#include "../app/app_calibration_context.h"
#include "../app/app_loop.h"
#include "../app/app_mode_context.h"
#include "../app/app_runtime_context.h"
#include "../app/app_setpoint_context.h"
#include "../app/app_measurements_context.h"
#include "../core/core_modes.h"
#include "../ui/ui_mode_templates.h"
#define Sns_Volt_Calib_Fact (app_calibration_sns_volt_factor_ref())
#define Sns_Volt_Calib_Offs (app_calibration_sns_volt_offset_ref())
#define Sns_Curr_Calib_Fact (app_calibration_sns_curr_factor_ref())
#define Sns_Curr_Calib_Offs (app_calibration_sns_curr_offset_ref())
#define Out_Curr_Calib_Fact (app_calibration_out_curr_factor_ref())
#define Out_Curr_Calib_Offs (app_calibration_out_curr_offset_ref())

void legacy_read_volts_current() {
#ifndef WOKWI_SIMULATION

  float raw_voltage;
  float raw_current;

  ads.setGain(GAIN_TWOTHIRDS);
  const int16_t adcv = ads.readADC_SingleEnded(VLTG_SNSR);
  raw_voltage = ads.computeVolts(adcv) * SNS_VOLT_FACT;

  app_measurements_set_voltage_v(raw_voltage * Sns_Volt_Calib_Fact + Sns_Volt_Calib_Offs);

  ads.setGain(GAIN_ONE);
  const int16_t adci = ads.readADC_SingleEnded(CRR_SNSR);
  raw_current = ads.computeVolts(adci) * SNS_CURR_FACT;

  app_measurements_set_current_a(raw_current * Sns_Curr_Calib_Fact + Sns_Curr_Calib_Offs);

#else

  const int potValue = analogRead(VSIM);
  static float simulatedVoltage = 0;
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
      simulatedVoltage -= 0.005;
      simulatedVoltage = max(simulatedVoltage, 0.0f);
      app_measurements_set_voltage_v(simulatedVoltage);
    } else if (!loadEnabled) {
      simulatedVoltage = simulatedPotVoltage;
      app_measurements_set_voltage_v(simulatedVoltage);
    }
  } else if (app_mode_is_calibration()) {
    simulatedVoltage = simulatedPotVoltage;
    float error_voltage = simulatedVoltage * 1.05f - 0.1f;
    app_measurements_set_voltage_v(error_voltage * Sns_Volt_Calib_Fact + Sns_Volt_Calib_Offs);
  }

  if (loadEnabled) {
    app_measurements_set_current_a(app_load_set_current_mA() / 1000);
  } else {
    app_measurements_set_current_a(0);
  }

#endif
}




