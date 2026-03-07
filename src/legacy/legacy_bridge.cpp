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

  state.loadEnabled = toggle;
  state.mode = static_cast<uint8_t>(Mode);
  state.modeInitialized = modeInitialized;
  state.modeConfigured = modeConfigured;

  return state;
}