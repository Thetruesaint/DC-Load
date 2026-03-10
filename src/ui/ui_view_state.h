#ifndef UI_VIEW_STATE_H
#define UI_VIEW_STATE_H

#include <stdint.h>

struct UiViewState {
  bool modeInitialized;
  bool loadEnabled;
  uint8_t mode;
  int cursorPosition;

  float measuredCurrent_A;
  float measuredVoltage_V;
  float measuredPower_W;
  float readingValue;

  float currentCutOffA;
  float powerCutOffW;
  float tempCutOffC;
  float fanTempOnC;
  float fanHoldSeconds;
  float batteryCutoffVolts;
  float batteryLife;
  char batteryType[8];

  float limitsDraftCurrentA;
  float limitsDraftPowerW;
  float limitsDraftTempC;
  uint8_t limitsMenuField;
  bool limitsEditActive;
  char limitsInputText[8];

  float fanDraftTempC;
  float fanDraftHoldSeconds;
  bool fanEditActive;
  char fanInputText[8];

  uint8_t calibrationMenuOption;
  uint8_t menuRootSelection;
  uint8_t protectionMenuSelection;
  uint8_t fanSettingsMenuSelection;

  uint8_t pendingConfigSection;
};

inline UiViewState ui_view_state_make_default() {
  UiViewState state = {0};
  state.calibrationMenuOption = 1;
  state.menuRootSelection = 0;
  state.protectionMenuSelection = 0;
  state.fanSettingsMenuSelection = 0;
  return state;
}

#endif
