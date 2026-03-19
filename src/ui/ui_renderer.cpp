#include "ui_renderer.h"

#include <cstring>

#include "ui_state_cache.h"
#include "ui_state_machine.h"

namespace {
UiViewState make_ui_view_state(const SystemState &state) {
  UiViewState view = ui_view_state_make_default();
  view.modeInitialized = state.modeInitialized;
  view.loadEnabled = state.loadEnabled;
  view.traceOverlayActive = state.traceOverlayActive;
  view.mode = state.mode;
  view.cursorPosition = state.cursorPosition;
  view.measuredCurrent_A = state.measuredCurrent_A;
  view.measuredVoltage_V = state.measuredVoltage_V;
  view.measuredPower_W = state.measuredPower_W;
  view.tempC = state.temp_C;
  view.readingValue = state.readingValue;
  view.setCurrent_mA = state.setCurrent_mA;
  view.currentCutOffA = state.currentCutOffA;
  view.powerCutOffW = state.powerCutOffW;
  view.tempCutOffC = state.tempCutOffC;
  view.fanTempOnC = state.fanTempOnC;
  view.fanHoldSeconds = state.fanHoldSeconds;
  view.batteryCutoffVolts = state.batteryCutoffVolts;
  view.batteryLife = state.batteryLife;
  view.batteryDone = state.batteryDone;
  std::strncpy(view.batteryType, state.batteryType, sizeof(view.batteryType) - 1);
  view.batteryType[sizeof(view.batteryType) - 1] = '\0';
  view.transientLowCurrentA = state.transientLowCurrentA;
  view.transientHighCurrentA = state.transientHighCurrentA;
  view.transientPeriodMs = state.transientPeriodMs;
  view.transientListActiveStep = state.transientListActiveStep;
  view.transientListTotalSteps = state.transientListTotalSteps;
  view.limitsDraftCurrentA = state.limitsDraftCurrentA;
  view.limitsDraftPowerW = state.limitsDraftPowerW;
  view.limitsDraftTempC = state.limitsDraftTempC;
  view.limitsMenuField = state.limitsMenuField;
  view.limitsEditActive = state.limitsEditActive;
  std::strncpy(view.limitsInputText, state.limitsInputText, sizeof(view.limitsInputText) - 1);
  view.limitsInputText[sizeof(view.limitsInputText) - 1] = '\0';
  view.fanDraftTempC = state.fanDraftTempC;
  view.fanDraftHoldSeconds = state.fanDraftHoldSeconds;
  view.fanEditActive = state.fanEditActive;
  std::strncpy(view.fanInputText, state.fanInputText, sizeof(view.fanInputText) - 1);
  view.fanInputText[sizeof(view.fanInputText) - 1] = '\0';
  view.fanManualOverrideActive = state.fanManualOverrideActive;
  view.fanManualStateOn = state.fanManualStateOn;
  view.rtcDay = state.rtcDay;
  view.rtcMonth = state.rtcMonth;
  view.rtcYear = state.rtcYear;
  view.rtcHour = state.rtcHour;
  view.rtcMinute = state.rtcMinute;
  view.clockDraftDay = state.clockDraftDay;
  view.clockDraftMonth = state.clockDraftMonth;
  view.clockDraftYear = state.clockDraftYear;
  view.clockDraftHour = state.clockDraftHour;
  view.clockDraftMinute = state.clockDraftMinute;
  view.clockEditActive = state.clockEditActive;
  std::strncpy(view.clockInputText, state.clockInputText, sizeof(view.clockInputText) - 1);
  view.clockInputText[sizeof(view.clockInputText) - 1] = '\0';
  view.batterySetupStage = state.batterySetupStage;
  std::strncpy(view.batteryInputText, state.batteryInputText, sizeof(view.batteryInputText) - 1);
  view.batteryInputText[sizeof(view.batteryInputText) - 1] = '\0';
  view.transientSetupStage = state.transientSetupStage;
  std::strncpy(view.transientInputText, state.transientInputText, sizeof(view.transientInputText) - 1);
  view.transientInputText[sizeof(view.transientInputText) - 1] = '\0';
  view.transientListSetupStage = state.transientListSetupStage;
  view.transientListDraftStepCount = state.transientListDraftStepCount;
  view.transientListDraftStepIndex = state.transientListDraftStepIndex;
  view.transientListDraftField = state.transientListDraftField;
  view.transientListCurrentA = state.transientListDraftCurrentsA[state.transientListDraftStepIndex];
  view.transientListCurrentPeriodMs = state.transientListDraftPeriodsMs[state.transientListDraftStepIndex];
  std::strncpy(view.transientListInputText, state.transientListInputText, sizeof(view.transientListInputText) - 1);
  view.transientListInputText[sizeof(view.transientListInputText) - 1] = '\0';
  view.calibrationMenuOption = state.calibrationMenuOption;
  view.menuRootSelection = state.menuRootSelection;
  view.protectionMenuSelection = state.protectionMenuSelection;
  view.updateMenuSelection = state.updateMenuSelection;
  view.fanSettingsMenuSelection = state.fanSettingsMenuSelection;
  view.clockMenuSelection = state.clockMenuSelection;
  return view;
}
}

void ui_render(const SystemState &state) {
  const UiViewState view = make_ui_view_state(state);
  ui_state_cache_set(view);
  ui_state_machine_tick(state.uiScreen, view);
}
