#include "core_config_flow.h"

bool core_config_wants_limits(const SystemState &state) {
  return state.pendingConfigSection == ConfigSection::Limits || state.openLimitsConfigEvent;
}

bool core_config_wants_calibration(const SystemState &state) {
  return state.pendingConfigSection == ConfigSection::Calibration || state.calibrationValueConfirmEvent;
}

UiScreen core_config_target_screen(const SystemState &state) {
  if (core_config_wants_limits(state)) {
    return UiScreen::MenuLimits;
  }

  if (core_config_wants_calibration(state)) {
    return UiScreen::MenuCalibration;
  }

  return UiScreen::Home;
}
