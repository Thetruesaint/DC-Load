#ifndef UI_VIEW_STATE_H
#define UI_VIEW_STATE_H

#include <stdint.h>

struct UiViewState {
  bool modeInitialized;
  bool loadEnabled;
  bool traceOverlayActive;
  uint8_t mode;
  int cursorPosition;

  float measuredCurrent_A;
  float measuredVoltage_V;
  float measuredPower_W;
  float tempC;
  float readingValue;
  float setCurrent_mA;

  float currentCutOffA;
  float powerCutOffW;
  float tempCutOffC;
  float fanTempOnC;
  float fanHoldSeconds;
  float batteryCutoffVolts;
  float batteryLife;
  bool batteryDone;
  char batteryType[8];
  float transientLowCurrentA;
  float transientHighCurrentA;
  float transientPeriodMs;
  uint8_t transientListActiveStep;
  uint8_t transientListTotalSteps;

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
  bool fanManualOverrideActive;
  bool fanManualStateOn;

  uint8_t rtcDay;
  uint8_t rtcMonth;
  uint8_t rtcYear;
  uint8_t rtcHour;
  uint8_t rtcMinute;
  uint8_t clockDraftDay;
  uint8_t clockDraftMonth;
  uint8_t clockDraftYear;
  uint8_t clockDraftHour;
  uint8_t clockDraftMinute;
  bool clockEditActive;
  char clockInputText[3];

  uint8_t batterySetupStage;
  char batteryInputText[8];
  uint8_t transientSetupStage;
  char transientInputText[8];
  uint8_t transientListSetupStage;
  uint8_t transientListDraftStepCount;
  uint8_t transientListDraftStepIndex;
  uint8_t transientListDraftField;
  float transientListCurrentA;
  float transientListCurrentPeriodMs;
  char transientListInputText[8];

  uint8_t calibrationMenuOption;
  uint8_t menuRootSelection;
  uint8_t protectionMenuSelection;
  uint8_t updateMenuSelection;
  uint8_t fanSettingsMenuSelection;
  uint8_t clockMenuSelection;
};

inline UiViewState ui_view_state_make_default() {
  UiViewState state = {0};
  state.calibrationMenuOption = 1;
  state.menuRootSelection = 0;
  state.protectionMenuSelection = 0;
  state.updateMenuSelection = 0;
  state.fanSettingsMenuSelection = 0;
  state.clockMenuSelection = 0;
  state.batterySetupStage = 0;
  state.transientSetupStage = 0;
  state.transientListSetupStage = 0;
  state.transientListDraftField = 0;
  return state;
}

#endif
