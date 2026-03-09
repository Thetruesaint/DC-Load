#include "legacy_bridge.h"

#include "../variables.h"
#include "../app/app_load_context.h"
#include "../app/app_runtime_context.h"
#include "../app/app_setpoint_context.h"

SystemState legacy_capture_state() {
  SystemState state = {0};

  state.setCurrent_mA = app_load_set_current_mA();
  state.setPower_W = setPower;
  state.setResistance_Ohm = setResistance;

  state.measuredCurrent_A = current;
  state.measuredVoltage_V = voltage;
  state.measuredPower_W = voltage * current;
  state.temp_C = static_cast<float>(temp);

  state.readingValue = app_setpoint_reading();
  state.encoderPositionRaw = app_runtime_encoder_position();
  state.encoderStep = app_runtime_encoder_step();
  state.encoderMaxRaw = static_cast<float>(app_runtime_encoder_max());
  state.currentCutOffA = CurrentCutOff;
  state.cursorPosition = app_runtime_cursor_position();
  state.functionIndex = functionIndex;

  state.lastEncoderDelta = 0;
  state.lastKeyPressed = '\0';
  state.loadToggleEvent = false;
  state.actionCounter = 0;

  state.loadEnabled = app_load_is_enabled();
  state.mode = static_cast<uint8_t>(Mode);
  state.modeInitialized = modeInitialized;
  state.modeConfigured = modeConfigured;

  return state;
}

void legacy_apply_state(const SystemState &state) {
  static bool lastAppliedLoadEnabled = false;
  static bool initialized = false;

  const bool previous = initialized ? lastAppliedLoadEnabled : app_load_is_enabled();

  app_runtime_set_encoder_position(state.encoderPositionRaw);
  app_runtime_set_encoder_step(state.encoderStep);
  app_runtime_set_cursor_position(state.cursorPosition);
  functionIndex = state.functionIndex;
  Mode = static_cast<ModeType>(state.mode);
  modeInitialized = state.modeInitialized;
  modeConfigured = state.modeConfigured;
  app_setpoint_set_reading(state.readingValue);
  app_load_set_enabled(state.loadEnabled);

  if (state.mode == CC || state.mode == CP || state.mode == CR || state.mode == CA) {
    app_load_set_set_current_mA(state.setCurrent_mA);
    setPower = state.setPower_W;
    setResistance = state.setResistance_Ohm;
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
