#ifndef CORE_STATE_H
#define CORE_STATE_H

#include <stdint.h>

enum class UiScreen : uint8_t {
  Home = 0,
  BatterySetupTask,
  BatterySetupCustomCutoff,
  BatterySetupCellCount,
  TransientContSetupLow,
  TransientContSetupHigh,
  TransientContSetupPeriod,
  TransientListSetupCount,
  TransientListSetupStep,
  MenuRoot,
  MenuProtection,
  MenuUpdate,
  MenuFwUpdate,
  MenuFanSettings,
  MenuLimits,
  MenuCalibration,
  MenuClock
};

enum class ConfigMenu : uint8_t {
  None = 0,
  Root,
  Protection,
  Update,
  FwUpdate,
  FanSettings,
  Limits,
  Calibration,
  Clock
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
  bool limitsEditActive;
  char limitsInputText[8];
  uint8_t limitsInputLength;
  bool limitsInputHasDecimal;

  float fanDraftTempC;
  float fanDraftHoldSeconds;
  bool fanEditActive;
  char fanInputText[8];
  uint8_t fanInputLength;
  bool fanOutputOn;
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
  uint8_t clockInputLength;

  uint8_t batterySetupStage;
  char batteryInputText[8];
  uint8_t batteryInputLength;
  bool batteryInputHasDecimal;

  uint8_t transientSetupStage;
  char transientInputText[8];
  uint8_t transientInputLength;
  bool transientInputHasDecimal;
  uint8_t transientListSetupStage;
  uint8_t transientListDraftStepCount;
  uint8_t transientListDraftStepIndex;
  uint8_t transientListDraftField;
  float transientListDraftCurrentsA[10];
  float transientListDraftPeriodsMs[10];
  char transientListInputText[8];
  uint8_t transientListInputLength;
  bool transientListInputHasDecimal;

  uint8_t calibrationMenuOption;
  uint8_t menuRootSelection;
  uint8_t protectionMenuSelection;
  uint8_t updateMenuSelection;
  uint8_t fanSettingsMenuSelection;
  uint8_t clockMenuSelection;

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
  bool clockSaveEvent;
  float calibrationRealValue;
  uint32_t actionCounter;

  bool loadEnabled;
  bool traceOverlayActive;
  uint8_t mode;
  bool modeInitialized;
  bool modeConfigured;

  UiScreen uiScreen;
  ConfigMenu currentConfigMenu;
  ConfigMenu parentConfigMenu;
};

inline SystemState core_state_make_default() {
  SystemState state = {0};
  state.uiScreen = UiScreen::Home;
  state.currentConfigMenu = ConfigMenu::None;
  state.parentConfigMenu = ConfigMenu::None;
  state.limitsMenuField = 0;
  state.calibrationMenuOption = 1;
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

inline void core_state_clear_one_shot_events(SystemState *state) {
  if (state == nullptr) return;
  state->loadToggleEvent = false;
  state->calibrationValueConfirmEvent = false;
  state->openLimitsConfigEvent = false;
  state->openCalibrationConfigEvent = false;
  state->limitsSaveEvent = false;
  state->calibrationMenuApplyEvent = false;
  state->fanSaveEvent = false;
  state->clockSaveEvent = false;
  state->calibrationRealValue = 0.0f;
}

#endif
