#include "core_config_flow.h"

bool core_config_wants_limits(const SystemState &state) {
  return state.openLimitsConfigEvent;
}

bool core_config_wants_calibration(const SystemState &state) {
  return state.openCalibrationConfigEvent;
}

UiScreen core_config_target_screen(const SystemState &state) {
  if (state.limitsMenuActive) {
    return UiScreen::MenuLimits;
  }

  if (state.pendingConfigSection == ConfigSection::Limits ||
      state.pendingConfigSection == ConfigSection::Calibration) {
    return UiScreen::MenuRoot;
  }

  return UiScreen::Home;
}

