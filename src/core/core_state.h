#ifndef CORE_STATE_H
#define CORE_STATE_H

#include <stdint.h>

enum class UiScreen : uint8_t {
  Home = 0,
  MenuRoot,
  MenuProtection,
  MenuFanSettings,
  MenuLimits,
  MenuCalibration
};

enum class ConfigSection : uint8_t {
  None = 0,
  Limits,
  Calibration
};

enum class ConfigMenu : uint8_t {
  None = 0,
  Root,
  Protection,
  FanSettings,
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
  float fanTempOnC;
  float fanHoldSeconds;

  float limitsDraftCurrentA;
  float limitsDraftPowerW;
  float limitsDraftTempC;
  bool limitsEditActive;
  char limitsInputText[8];
  uint8_t limitsInputLength;
  bool limitsInputHasDecimal;

  float fanDraftTempC;
  float fanDraftHoldSeconds;
  bool fanEditActive;
  char fanInputText[8];
  uint8_t fanInputLength;

  uint8_t calibrationMenuOption;
  uint8_t menuRootSelection;
  uint8_t protectionMenuSelection;
  uint8_t fanSettingsMenuSelection;

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
  bool calibrationMenuActive;
  bool calibrationMenuApplyEvent;
  bool fanSaveEvent;
  float calibrationRealValue;
  uint32_t actionCounter;

  bool loadEnabled;
  uint8_t mode;
  bool modeInitialized;
  bool modeConfigured;

  UiScreen uiScreen;
  ConfigSection pendingConfigSection;
  ConfigMenu currentConfigMenu;
  ConfigMenu parentConfigMenu;
};

inline SystemState core_state_make_default() {
  SystemState state = {0};
  state.uiScreen = UiScreen::Home;
  state.pendingConfigSection = ConfigSection::None;
  state.currentConfigMenu = ConfigMenu::None;
  state.parentConfigMenu = ConfigMenu::None;
  state.limitsMenuField = 0;
  state.calibrationMenuOption = 1;
  state.protectionMenuSelection = 0;
  state.fanSettingsMenuSelection = 0;
  return state;
}

inline void core_state_clear_one_shot_events(SystemState *state) {
  if (state == nullptr) return;
  state->loadToggleEvent = false;
  state->calibrationValueConfirmEvent = false;
  state->openLimitsConfigEvent = false;
  state->openCalibrationConfigEvent = false;
  state->limitsSaveEvent = false;
  state->calibrationMenuApplyEvent = false;
  state->fanSaveEvent = false;
  state->calibrationRealValue = 0.0f;
}

#endif
