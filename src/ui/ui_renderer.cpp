#include "ui_renderer.h"

#include <cstring>

#include "ui_state_cache.h"
#include "ui_state_machine.h"

namespace {
UiViewState make_ui_view_state(const SystemState &state) {
  UiViewState view = ui_view_state_make_default();
  view.modeInitialized = state.modeInitialized;
  view.loadEnabled = state.loadEnabled;
  view.mode = state.mode;
  view.cursorPosition = state.cursorPosition;
  view.measuredCurrent_A = state.measuredCurrent_A;
  view.measuredVoltage_V = state.measuredVoltage_V;
  view.measuredPower_W = state.measuredPower_W;
  view.readingValue = state.readingValue;
  view.currentCutOffA = state.currentCutOffA;
  view.powerCutOffW = state.powerCutOffW;
  view.tempCutOffC = state.tempCutOffC;
  view.fanTempOnC = state.fanTempOnC;
  view.fanHoldSeconds = state.fanHoldSeconds;
  view.batteryCutoffVolts = state.batteryCutoffVolts;
  view.batteryLife = state.batteryLife;
  std::strncpy(view.batteryType, state.batteryType, sizeof(view.batteryType) - 1);
  view.batteryType[sizeof(view.batteryType) - 1] = '\0';
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
  view.calibrationMenuOption = state.calibrationMenuOption;
  view.menuRootSelection = state.menuRootSelection;
  view.protectionMenuSelection = state.protectionMenuSelection;
  view.fanSettingsMenuSelection = state.fanSettingsMenuSelection;
  view.pendingConfigSection = static_cast<uint8_t>(state.pendingConfigSection);
  return view;
}
}

void ui_render(const SystemState &state) {
  const UiViewState view = make_ui_view_state(state);
  ui_state_cache_set(view);
  ui_state_machine_tick(state.uiScreen, view);
}
