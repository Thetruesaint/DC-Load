#ifndef CORE_STATE_H
#define CORE_STATE_H

#include <stdint.h>

enum class UiScreen : uint8_t {
  Home = 0,
  MenuRoot,
  MenuLimits,
  MenuCalibration
};

enum class ConfigSection : uint8_t {
  None = 0,
  Limits,
  Calibration
};

struct SystemState {
  float setCurrent_mA;
  float setPower_W;
  float setResistance_Ohm;

  float measuredCurrent_A;
  float measuredVoltage_V;
  float measuredPower_W;
  float temp_C;

  float readingValue;
  float encoderPositionRaw;
  float encoderStep;
  float encoderMaxRaw;
  float currentCutOffA;
  float powerCutOffW;
  float tempCutOffC;

  // Draft values for non-blocking limits menu flow.
  float limitsDraftCurrentA;
  float limitsDraftPowerW;
  float limitsDraftTempC;

  int cursorPosition;
  int functionIndex;
  uint8_t limitsMenuField;

  int32_t lastEncoderDelta;
  char lastKeyPressed;
  bool loadToggleEvent;
  bool calibrationValueConfirmEvent;
  bool openLimitsConfigEvent;
  bool openCalibrationConfigEvent;
  bool limitsMenuActive;
  bool limitsSaveEvent;
  float calibrationRealValue;
  uint32_t actionCounter;

  bool loadEnabled;
  uint8_t mode;
  bool modeInitialized;
  bool modeConfigured;

  UiScreen uiScreen;
  ConfigSection pendingConfigSection;
};

inline SystemState core_state_make_default() {
  SystemState state = {0};
  state.uiScreen = UiScreen::Home;
  state.pendingConfigSection = ConfigSection::None;
  state.limitsMenuField = 0;
  return state;
}

inline void core_state_clear_one_shot_events(SystemState *state) {
  if (state == nullptr) return;
  state->loadToggleEvent = false;
  state->calibrationValueConfirmEvent = false;
  state->openLimitsConfigEvent = false;
  state->openCalibrationConfigEvent = false;
  state->limitsSaveEvent = false;
  state->calibrationRealValue = 0.0f;
}

#endif
