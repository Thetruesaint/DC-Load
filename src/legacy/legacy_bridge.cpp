#include "legacy_bridge.h"

#include "../variables.h"

SystemState legacy_capture_state() {
  SystemState state = {0};

  state.setCurrent_mA = setCurrent;
  state.setPower_W = setPower;
  state.setResistance_Ohm = setResistance;

  state.measuredCurrent_A = current;
  state.measuredVoltage_V = voltage;
  state.measuredPower_W = voltage * current;
  state.temp_C = static_cast<float>(temp);

  state.readingValue = reading;
  state.encoderPositionRaw = encoderPosition;
  state.encoderStep = factor;
  state.encoderMaxRaw = static_cast<float>(maxEncoder);
  state.currentCutOffA = CurrentCutOff;
  state.cursorPosition = CuPo;

  state.lastEncoderDelta = 0;
  state.lastKeyPressed = '\0';
  state.loadToggleEvent = false;
  state.actionCounter = 0;

  state.loadEnabled = toggle;
  state.mode = static_cast<uint8_t>(Mode);
  state.modeInitialized = modeInitialized;
  state.modeConfigured = modeConfigured;

  return state;
}

void legacy_apply_state(const SystemState &state) {
  static bool lastAppliedLoadEnabled = false;
  static bool initialized = false;

  const bool previous = initialized ? lastAppliedLoadEnabled : toggle;

  encoderPosition = state.encoderPositionRaw;
  factor = state.encoderStep;
  CuPo = state.cursorPosition;
  reading = state.readingValue;
  toggle = state.loadEnabled;

  if (state.mode == CC || state.mode == CP || state.mode == CR || state.mode == CA) {
    setCurrent = state.setCurrent_mA;
    setPower = state.setPower_W;
    setResistance = state.setResistance_Ohm;
  }

  if (!toggle) {
    setCurrent = 0;
  }

#ifndef WOKWI_SIMULATION
  if (initialized && previous != toggle && !toggle) {
    dac.setVoltage(0, false);
  }
#endif

  lastAppliedLoadEnabled = toggle;
  initialized = true;
}