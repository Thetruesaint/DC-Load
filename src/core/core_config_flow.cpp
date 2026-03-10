#include "core_config_flow.h"

bool core_config_wants_limits(const SystemState &state) {
  return state.openLimitsConfigEvent;
}

bool core_config_wants_calibration(const SystemState &state) {
  return state.openCalibrationConfigEvent;
}

UiScreen core_config_target_screen(const SystemState &state) {
  if (state.currentConfigMenu == ConfigMenu::Protection) {
    return UiScreen::MenuProtection;
  }

  if (state.currentConfigMenu == ConfigMenu::FanSettings) {
    return UiScreen::MenuFanSettings;
  }

  if (state.currentConfigMenu == ConfigMenu::Limits || state.limitsMenuActive) {
    return UiScreen::MenuLimits;
  }

  if (state.currentConfigMenu == ConfigMenu::Calibration || state.calibrationMenuActive) {
    return UiScreen::MenuCalibration;
  }

  if (state.currentConfigMenu == ConfigMenu::Root ||
      state.pendingConfigSection == ConfigSection::Limits ||
      state.pendingConfigSection == ConfigSection::Calibration) {
    return UiScreen::MenuRoot;
  }

  return UiScreen::Home;
}
