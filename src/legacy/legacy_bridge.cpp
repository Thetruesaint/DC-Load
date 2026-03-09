#include "legacy_bridge.h"

#include "../config/system_constants.h"
#include "../hw/hw_objects.h"
#include "../app/app_load_context.h"
#include "../app/app_mode_state_context.h"
#include "../app/app_limits_context.h"
#include "../app/app_measurements_context.h"
#include "../app/app_runtime_context.h"
#include "../app/app_mode_setpoint_context.h"
#include "../app/app_setpoint_context.h"

SystemState legacy_capture_state() {
  SystemState state = {0};

  state.setCurrent_mA = app_load_set_current_mA();
  state.setPower_W = app_mode_setpoint_power_w();
  state.setResistance_Ohm = app_mode_setpoint_resistance_ohm();

  state.measuredCurrent_A = app_measurements_current_a();
  state.measuredVoltage_V = app_measurements_voltage_v();
  state.measuredPower_W = app_measurements_power_w();
  state.temp_C = static_cast<float>(app_measurements_temp_c());

  state.readingValue = app_setpoint_reading();
  state.encoderPositionRaw = app_runtime_encoder_position();
  state.encoderStep = app_runtime_encoder_step();
  state.encoderMaxRaw = static_cast<float>(app_runtime_encoder_max());
  state.currentCutOffA = app_limits_current_cutoff();
  state.cursorPosition = app_runtime_cursor_position();
  state.functionIndex = app_mode_state_function_index();

  state.lastEncoderDelta = 0;
  state.lastKeyPressed = '\0';
  state.loadToggleEvent = false;
  state.actionCounter = 0;

  state.loadEnabled = app_load_is_enabled();
  state.mode = app_mode_state_mode();
  state.modeInitialized = app_mode_state_initialized();
  state.modeConfigured = app_mode_state_configured();

  return state;
}

void legacy_apply_state(const SystemState &state) {
  static bool lastAppliedLoadEnabled = false;
  static bool initialized = false;

  const bool previous = initialized ? lastAppliedLoadEnabled : app_load_is_enabled();

  app_runtime_set_encoder_position(state.encoderPositionRaw);
  app_runtime_set_encoder_step(state.encoderStep);
  app_runtime_set_cursor_position(state.cursorPosition);
  app_mode_state_set_function_index(state.functionIndex);
  app_mode_state_set_mode(state.mode);
  app_mode_state_set_initialized(state.modeInitialized);
  app_mode_state_set_configured(state.modeConfigured);
  app_setpoint_set_reading(state.readingValue);
  app_load_set_enabled(state.loadEnabled);

  if (state.mode == CC || state.mode == CP || state.mode == CR || state.mode == CA) {
    app_load_set_set_current_mA(state.setCurrent_mA);
    app_mode_setpoint_set_power_w(state.setPower_W);
    app_mode_setpoint_set_resistance_ohm(state.setResistance_Ohm);
  }

  if (!app_load_is_enabled()) {
    app_load_set_set_current_mA(0.0f);
  }

#ifndef WOKWI_SIMULATION
  if (initialized && previous != app_load_is_enabled() && !app_load_is_enabled()) {
    dac.setVoltage(0, false);
  }
#endif

  lastAppliedLoadEnabled = app_load_is_enabled();
  initialized = true;
}
