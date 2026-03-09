#include "legacy_base_io.h"

#include "../variables.h"
#include "../funciones.h"
#include "../app/app_inputs.h"
#include "../app/app_io_context.h"
#include "../app/app_load_context.h"
#include "../app/app_loop.h"
#include "../app/app_mode_context.h"
#include "../app/app_runtime_context.h"
#include "../app/app_setpoint_context.h"
#include "../app/app_measurements_context.h"
#include "../core/core_modes.h"

void legacy_load_off() {
#ifndef WOKWI_SIMULATION
  dac.setVoltage(0, false);
  app_load_set_enabled(false);
  app_load_set_set_current_mA(0);
#else
  app_load_set_enabled(false);
  app_load_set_set_current_mA(0);
#endif
}

void legacy_encoder_status(bool encOnOff, float limit) {
  if (encOnOff) {
    app_runtime_set_cursor_position(8);
    app_setpoint_set_reading(0);
    app_runtime_set_encoder_position(0);
    app_setpoint_set_max_reading(limit);
    app_runtime_set_encoder_max(static_cast<unsigned long>(app_setpoint_max_reading() * 1000));

    encoder.clearCount();
  } else {
    encoder.clearCount();
  }
}

void legacy_read_encoder() {
  app_read_encoder();
}

void legacy_cursor_position() {
  static uint32_t lastPressTime = 0;
  constexpr int unitPosition = 8;
  static int lastCursor = -1;

  int cursor = app_runtime_cursor_position();
  const uint8_t mode = app_mode_id();
  const bool managedMode = core_mode_is_managed(mode);
  const uint32_t nowMs = app_io_millis();

  if (app_io_encoder_button_low() && (nowMs - lastPressTime > 200)) {
    lastPressTime = nowMs;
    if (managedMode) {
      app_push_action(ActionType::EncoderButtonPress, 0, '\0');
    } else {
      cursor++;
      app_runtime_set_cursor_position(cursor);
    }
  }

  if (lastCursor == cursor) return;

  if (managedMode) {
    lastCursor = cursor;
    setCursorLCD(cursor, 2);
    return;
  }

  if (cursor > lastCursor && cursor == unitPosition + 1) cursor++;
  if (cursor < lastCursor && cursor == unitPosition + 1) cursor--;

  if ((mode == CC || mode == BC || mode == CA) && cursor > 12) cursor = unitPosition;
  if ((mode == CC || mode == BC || mode == CA) && cursor < 8) cursor = unitPosition + 4;
  if ((mode == CP || mode == CR) && cursor > 10) cursor = unitPosition - 2;
  if ((mode == CP || mode == CR) && cursor < 6) cursor = unitPosition + 2;

  switch (cursor) {
    case 6: app_runtime_set_encoder_step(100000); break;
    case 7: app_runtime_set_encoder_step(10000); break;
    case 10: app_runtime_set_encoder_step(100); break;
    case 11: app_runtime_set_encoder_step(10); break;
    case 12: app_runtime_set_encoder_step(1); break;
    default: app_runtime_set_encoder_step(1000);
  }

  app_runtime_set_cursor_position(cursor);
  lastCursor = cursor;

  setCursorLCD(cursor, 2);
}

void legacy_read_volts_current() {
#ifndef WOKWI_SIMULATION

  float raw_voltage;
  float raw_current;

  ads.setGain(GAIN_TWOTHIRDS);
  adcv = ads.readADC_SingleEnded(VLTG_SNSR);
  raw_voltage = ads.computeVolts(adcv) * SNS_VOLT_FACT;

  app_measurements_set_voltage_v(raw_voltage * Sns_Volt_Calib_Fact + Sns_Volt_Calib_Offs);

  ads.setGain(GAIN_ONE);
  adci = ads.readADC_SingleEnded(CRR_SNSR);
  raw_current = ads.computeVolts(adci) * SNS_CURR_FACT;

  app_measurements_set_current_a(raw_current * Sns_Curr_Calib_Fact + Sns_Curr_Calib_Offs);

#else

  int potValue = analogRead(VSIM);
  static float simulatedVoltage = 0;
  static unsigned long lastDecreaseTime = 0;
  unsigned long currentMillis = app_io_millis();
  const bool loadEnabled = app_load_is_enabled();

  if (!app_mode_is_battery() && !app_mode_is_calibration()) {
    simulatedVoltage = map(potValue, 0, 1023, 550, 0) / 10.0;
    app_measurements_set_voltage_v(simulatedVoltage);
  } else if (app_mode_is_battery()) {
    if (loadEnabled && (currentMillis - lastDecreaseTime >= 2000)) {
      lastDecreaseTime = currentMillis;
      simulatedVoltage -= 0.005;
      simulatedVoltage = max(simulatedVoltage, 0.0f);
      app_measurements_set_voltage_v(simulatedVoltage);
    } else if (!loadEnabled) {
      simulatedVoltage = map(potValue, 0, 1023, 550, 0) / 10.0;
      app_measurements_set_voltage_v(simulatedVoltage);
    }
  } else if (app_mode_is_calibration()) {
    simulatedVoltage = map(potValue, 0, 1023, 550, 0) / 10.0;
    float error_voltage = simulatedVoltage * 1.05 - 0.1;
    app_measurements_set_voltage_v(error_voltage * Sns_Volt_Calib_Fact + Sns_Volt_Calib_Offs);
  }

  if (loadEnabled) {
    app_measurements_set_current_a(app_load_set_current_mA() / 1000);
  } else {
    app_measurements_set_current_a(0);
  }

#endif
}
