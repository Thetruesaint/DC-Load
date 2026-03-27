#include "core_sync_bridge.h"

#include <cstring>

#include "../config/system_constants.h"

namespace {
struct PreservedUiState {
  uint32_t actionCounter;
  UiScreen uiScreen;
  uint8_t menuRootSelection;
  uint8_t protectionMenuSelection;
  uint8_t updateMenuSelection;
  uint8_t fanSettingsMenuSelection;
  ConfigMenu currentConfigMenu;
  ConfigMenu parentConfigMenu;
  bool traceOverlayActive;
  int32_t lastEncoderDelta;
  char lastKeyPressed;
  bool limitsMenuActive;
  uint8_t limitsMenuField;
  float limitsDraftCurrentA;
  float limitsDraftPowerW;
  float limitsDraftTempC;
  bool limitsEditActive;
  char limitsInputText[8];
  uint8_t limitsInputLength;
  bool limitsInputHasDecimal;
  bool calibrationMenuActive;
  uint8_t calibrationMenuOption;
  float fanDraftTempC;
  float fanDraftHoldSeconds;
  bool fanEditActive;
  char fanInputText[8];
  uint8_t fanInputLength;
  uint8_t clockMenuSelection;
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
  float transientLowCurrentA;
  float transientHighCurrentA;
  float transientPeriodMs;
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
};

PreservedUiState capture_preserved_ui_state(const SystemState &state) {
  PreservedUiState preserved = {};
  preserved.actionCounter = state.actionCounter;
  preserved.uiScreen = state.uiScreen;
  preserved.menuRootSelection = state.menuRootSelection;
  preserved.protectionMenuSelection = state.protectionMenuSelection;
  preserved.updateMenuSelection = state.updateMenuSelection;
  preserved.fanSettingsMenuSelection = state.fanSettingsMenuSelection;
  preserved.currentConfigMenu = state.currentConfigMenu;
  preserved.parentConfigMenu = state.parentConfigMenu;
  preserved.traceOverlayActive = state.traceOverlayActive;
  preserved.lastEncoderDelta = state.lastEncoderDelta;
  preserved.lastKeyPressed = state.lastKeyPressed;
  preserved.limitsMenuActive = state.limitsMenuActive;
  preserved.limitsMenuField = state.limitsMenuField;
  preserved.limitsDraftCurrentA = state.limitsDraftCurrentA;
  preserved.limitsDraftPowerW = state.limitsDraftPowerW;
  preserved.limitsDraftTempC = state.limitsDraftTempC;
  preserved.limitsEditActive = state.limitsEditActive;
  std::strncpy(preserved.limitsInputText, state.limitsInputText, sizeof(preserved.limitsInputText) - 1);
  preserved.limitsInputText[sizeof(preserved.limitsInputText) - 1] = '\0';
  preserved.limitsInputLength = state.limitsInputLength;
  preserved.limitsInputHasDecimal = state.limitsInputHasDecimal;
  preserved.calibrationMenuActive = state.calibrationMenuActive;
  preserved.calibrationMenuOption = state.calibrationMenuOption;
  preserved.fanDraftTempC = state.fanDraftTempC;
  preserved.fanDraftHoldSeconds = state.fanDraftHoldSeconds;
  preserved.fanEditActive = state.fanEditActive;
  std::strncpy(preserved.fanInputText, state.fanInputText, sizeof(preserved.fanInputText) - 1);
  preserved.fanInputText[sizeof(preserved.fanInputText) - 1] = '\0';
  preserved.fanInputLength = state.fanInputLength;
  preserved.clockMenuSelection = state.clockMenuSelection;
  preserved.clockDraftDay = state.clockDraftDay;
  preserved.clockDraftMonth = state.clockDraftMonth;
  preserved.clockDraftYear = state.clockDraftYear;
  preserved.clockDraftHour = state.clockDraftHour;
  preserved.clockDraftMinute = state.clockDraftMinute;
  preserved.clockEditActive = state.clockEditActive;
  std::strncpy(preserved.clockInputText, state.clockInputText, sizeof(preserved.clockInputText) - 1);
  preserved.clockInputText[sizeof(preserved.clockInputText) - 1] = '\0';
  preserved.clockInputLength = state.clockInputLength;
  preserved.batterySetupStage = state.batterySetupStage;
  std::strncpy(preserved.batteryInputText, state.batteryInputText, sizeof(preserved.batteryInputText) - 1);
  preserved.batteryInputText[sizeof(preserved.batteryInputText) - 1] = '\0';
  preserved.batteryInputLength = state.batteryInputLength;
  preserved.batteryInputHasDecimal = state.batteryInputHasDecimal;
  preserved.transientSetupStage = state.transientSetupStage;
  preserved.transientLowCurrentA = state.transientLowCurrentA;
  preserved.transientHighCurrentA = state.transientHighCurrentA;
  preserved.transientPeriodMs = state.transientPeriodMs;
  std::strncpy(preserved.transientInputText, state.transientInputText, sizeof(preserved.transientInputText) - 1);
  preserved.transientInputText[sizeof(preserved.transientInputText) - 1] = '\0';
  preserved.transientInputLength = state.transientInputLength;
  preserved.transientInputHasDecimal = state.transientInputHasDecimal;
  preserved.transientListSetupStage = state.transientListSetupStage;
  preserved.transientListDraftStepCount = state.transientListDraftStepCount;
  preserved.transientListDraftStepIndex = state.transientListDraftStepIndex;
  preserved.transientListDraftField = state.transientListDraftField;
  for (int i = 0; i < 10; ++i) {
    preserved.transientListDraftCurrentsA[i] = state.transientListDraftCurrentsA[i];
    preserved.transientListDraftPeriodsMs[i] = state.transientListDraftPeriodsMs[i];
  }
  std::strncpy(preserved.transientListInputText, state.transientListInputText, sizeof(preserved.transientListInputText) - 1);
  preserved.transientListInputText[sizeof(preserved.transientListInputText) - 1] = '\0';
  preserved.transientListInputLength = state.transientListInputLength;
  preserved.transientListInputHasDecimal = state.transientListInputHasDecimal;
  return preserved;
}

void apply_runtime_snapshot(SystemState *current, const RuntimeSnapshot &incoming) {
  current->setCurrent_mA = incoming.setCurrent_mA;
  current->setPower_W = incoming.setPower_W;
  current->setResistance_Ohm = incoming.setResistance_Ohm;

  current->measuredCurrent_A = incoming.measuredCurrent_A;
  current->measuredVoltage_V = incoming.measuredVoltage_V;
  current->measuredPower_W = incoming.measuredPower_W;
  current->temp_C = incoming.temp_C;

  current->readingValue = incoming.readingValue;
  current->encoderPositionRaw = incoming.encoderPositionRaw;
  current->encoderStep = incoming.encoderStep;
  current->encoderMaxRaw = incoming.encoderMaxRaw;
  current->currentCutOffA = incoming.currentCutOffA;
  current->powerCutOffW = incoming.powerCutOffW;
  current->tempCutOffC = incoming.tempCutOffC;
  current->fanTempOnC = incoming.fanTempOnC;
  current->fanHoldSeconds = incoming.fanHoldSeconds;
  current->fanOutputOn = incoming.fanOutputOn;
  current->fanManualOverrideActive = incoming.fanManualOverrideActive;
  current->fanManualStateOn = incoming.fanManualStateOn;
  current->rtcDay = incoming.rtcDay;
  current->rtcMonth = incoming.rtcMonth;
  current->rtcYear = incoming.rtcYear;
  current->rtcHour = incoming.rtcHour;
  current->rtcMinute = incoming.rtcMinute;
  current->batteryCutoffVolts = incoming.batteryCutoffVolts;
  current->batteryLife = incoming.batteryLife;
  current->batteryDone = incoming.batteryDone;
  std::strncpy(current->batteryType, incoming.batteryType, sizeof(current->batteryType) - 1);
  current->batteryType[sizeof(current->batteryType) - 1] = '\0';
  current->transientLowCurrentA = incoming.transientLowCurrentA;
  current->transientHighCurrentA = incoming.transientHighCurrentA;
  current->transientPeriodMs = incoming.transientPeriodMs;
  current->transientListActiveStep = incoming.transientListActiveStep;
  current->transientListTotalSteps = incoming.transientListTotalSteps;
  current->cursorPosition = incoming.cursorPosition;
  current->functionIndex = incoming.functionIndex;
  current->loadEnabled = incoming.loadEnabled;
  current->mode = incoming.mode;
  current->modeInitialized = incoming.modeInitialized;
  current->modeConfigured = incoming.modeConfigured;
}

void restore_preserved_ui_state(SystemState *current, const PreservedUiState &preserved, const RuntimeSnapshot &incoming) {
  current->actionCounter = preserved.actionCounter;
  current->uiScreen = preserved.uiScreen;
  current->menuRootSelection = preserved.menuRootSelection;
  current->protectionMenuSelection = preserved.protectionMenuSelection;
  current->updateMenuSelection = preserved.updateMenuSelection;
  current->fanSettingsMenuSelection = preserved.fanSettingsMenuSelection;
  current->currentConfigMenu = preserved.currentConfigMenu;
  current->parentConfigMenu = preserved.parentConfigMenu;
  current->traceOverlayActive = preserved.traceOverlayActive;
  current->lastEncoderDelta = preserved.lastEncoderDelta;
  current->lastKeyPressed = preserved.lastKeyPressed;
  current->limitsMenuActive = preserved.limitsMenuActive;
  current->limitsMenuField = preserved.limitsMenuField;
  current->limitsDraftCurrentA = preserved.limitsDraftCurrentA;
  current->limitsDraftPowerW = preserved.limitsDraftPowerW;
  current->limitsDraftTempC = preserved.limitsDraftTempC;
  current->limitsEditActive = preserved.limitsEditActive;
  std::strncpy(current->limitsInputText, preserved.limitsInputText, sizeof(current->limitsInputText) - 1);
  current->limitsInputText[sizeof(current->limitsInputText) - 1] = '\0';
  current->limitsInputLength = preserved.limitsInputLength;
  current->limitsInputHasDecimal = preserved.limitsInputHasDecimal;
  current->calibrationMenuActive = preserved.calibrationMenuActive;
  current->calibrationMenuOption = preserved.calibrationMenuOption;
  current->fanDraftTempC = preserved.fanDraftTempC;
  current->fanDraftHoldSeconds = preserved.fanDraftHoldSeconds;
  current->fanEditActive = preserved.fanEditActive;
  std::strncpy(current->fanInputText, preserved.fanInputText, sizeof(current->fanInputText) - 1);
  current->fanInputText[sizeof(current->fanInputText) - 1] = '\0';
  current->fanInputLength = preserved.fanInputLength;
  current->clockMenuSelection = preserved.clockMenuSelection;
  current->clockDraftDay = preserved.clockDraftDay;
  current->clockDraftMonth = preserved.clockDraftMonth;
  current->clockDraftYear = preserved.clockDraftYear;
  current->clockDraftHour = preserved.clockDraftHour;
  current->clockDraftMinute = preserved.clockDraftMinute;
  current->clockEditActive = preserved.clockEditActive;
  std::strncpy(current->clockInputText, preserved.clockInputText, sizeof(current->clockInputText) - 1);
  current->clockInputText[sizeof(current->clockInputText) - 1] = '\0';
  current->clockInputLength = preserved.clockInputLength;
  current->batterySetupStage = preserved.batterySetupStage;
  std::strncpy(current->batteryInputText, preserved.batteryInputText, sizeof(current->batteryInputText) - 1);
  current->batteryInputText[sizeof(current->batteryInputText) - 1] = '\0';
  current->batteryInputLength = preserved.batteryInputLength;
  current->batteryInputHasDecimal = preserved.batteryInputHasDecimal;
  current->transientSetupStage = preserved.transientSetupStage;
  current->transientLowCurrentA = preserved.transientLowCurrentA;
  current->transientHighCurrentA = preserved.transientHighCurrentA;
  if (incoming.mode == TL && incoming.modeConfigured && preserved.uiScreen == UiScreen::Home) {
    current->transientPeriodMs = incoming.transientPeriodMs;
  } else {
    current->transientPeriodMs = preserved.transientPeriodMs;
  }
  std::strncpy(current->transientInputText, preserved.transientInputText, sizeof(current->transientInputText) - 1);
  current->transientInputText[sizeof(current->transientInputText) - 1] = '\0';
  current->transientInputLength = preserved.transientInputLength;
  current->transientInputHasDecimal = preserved.transientInputHasDecimal;
  current->transientListSetupStage = preserved.transientListSetupStage;
  current->transientListDraftStepCount = preserved.transientListDraftStepCount;
  current->transientListDraftStepIndex = preserved.transientListDraftStepIndex;
  current->transientListDraftField = preserved.transientListDraftField;
  for (int i = 0; i < 10; ++i) {
    current->transientListDraftCurrentsA[i] = preserved.transientListDraftCurrentsA[i];
    current->transientListDraftPeriodsMs[i] = preserved.transientListDraftPeriodsMs[i];
  }
  std::strncpy(current->transientListInputText, preserved.transientListInputText, sizeof(current->transientListInputText) - 1);
  current->transientListInputText[sizeof(current->transientListInputText) - 1] = '\0';
  current->transientListInputLength = preserved.transientListInputLength;
  current->transientListInputHasDecimal = preserved.transientListInputHasDecimal;
}
}  // namespace

void core_sync_merge_runtime_state(SystemState *current, const RuntimeSnapshot &incoming) {
  if (current == nullptr) return;

  const PreservedUiState preserved = capture_preserved_ui_state(*current);
  apply_runtime_snapshot(current, incoming);
  restore_preserved_ui_state(current, preserved, incoming);
}
