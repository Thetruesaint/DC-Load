#include "core_modes.h"

namespace {
constexpr uint8_t MODE_CC = 0;
constexpr uint8_t MODE_CP = 1;
constexpr uint8_t MODE_CR = 2;
constexpr uint8_t MODE_BC = 3;
constexpr uint8_t MODE_TC = 4;
constexpr uint8_t MODE_TL = 5;
constexpr uint8_t MODE_CA = 6;
constexpr int DECIMAL_CURSOR_COL = 9;

bool mode_uses_managed_input(uint8_t mode) {
  return mode == MODE_CC || mode == MODE_CP || mode == MODE_CR || mode == MODE_BC || mode == MODE_CA;
}

bool mode_uses_managed_setpoints(uint8_t mode) {
  return mode == MODE_CC || mode == MODE_CP || mode == MODE_CR || mode == MODE_CA;
}

int cursor_min_by_mode(uint8_t mode) {
  return (mode == MODE_CP || mode == MODE_CR) ? 6 : 8;
}

int cursor_max_by_mode(uint8_t mode) {
  return (mode == MODE_CP || mode == MODE_CR) ? 10 : 12;
}

float factor_for_cursor(int cursor) {
  switch (cursor) {
    case 6: return 100000.0f;
    case 7: return 10000.0f;
    case 10: return 100.0f;
    case 11: return 10.0f;
    case 12: return 1.0f;
    default: return 1000.0f;
  }
}

void wrap_cursor_by_mode(SystemState *state) {
  const int minPos = cursor_min_by_mode(state->mode);
  const int maxPos = cursor_max_by_mode(state->mode);

  if (state->cursorPosition > maxPos) state->cursorPosition = minPos;
  if (state->cursorPosition < minPos) state->cursorPosition = maxPos;
}
}

bool core_mode_is_managed(uint8_t mode) {
  return mode_uses_managed_input(mode);
}

void core_mode_normalize_state(SystemState *state) {
  if (state == nullptr || !mode_uses_managed_input(state->mode)) return;

  wrap_cursor_by_mode(state);
  if (state->cursorPosition == DECIMAL_CURSOR_COL) {
    state->cursorPosition = cursor_min_by_mode(state->mode);
  }
  state->encoderStep = factor_for_cursor(state->cursorPosition);
}

void core_mode_apply_encoder_delta(SystemState *state, int direction) {
  if (state == nullptr || !mode_uses_managed_input(state->mode)) return;

  if (direction > 0) {
    state->encoderPositionRaw += state->encoderStep;
  } else if (direction < 0) {
    state->encoderPositionRaw -= state->encoderStep;
  }

  if (state->encoderPositionRaw < 0.0f) state->encoderPositionRaw = 0.0f;
  if (state->encoderPositionRaw > state->encoderMaxRaw) state->encoderPositionRaw = state->encoderMaxRaw;
}

void core_mode_move_cursor(SystemState *state, int direction) {
  if (state == nullptr || !mode_uses_managed_input(state->mode)) return;

  if (direction > 0) {
    state->cursorPosition++;
    if (state->cursorPosition == DECIMAL_CURSOR_COL) state->cursorPosition++;
  } else if (direction < 0) {
    state->cursorPosition--;
    if (state->cursorPosition == DECIMAL_CURSOR_COL) state->cursorPosition--;
  }

  wrap_cursor_by_mode(state);
  state->encoderStep = factor_for_cursor(state->cursorPosition);
}

void core_mode_update_setpoints(SystemState *state) {
  if (state == nullptr || !mode_uses_managed_setpoints(state->mode)) return;

  const float minReading = (state->mode == MODE_CR) ? 0.1f : 0.0f;
  float maxReading = state->encoderMaxRaw / 1000.0f;
  if (maxReading < minReading) maxReading = minReading;

  float reading = state->encoderPositionRaw / 1000.0f;
  if (reading < minReading) reading = minReading;
  if (reading > maxReading) reading = maxReading;

  state->readingValue = reading;
  state->encoderPositionRaw = reading * 1000.0f;

  if (!state->loadEnabled) {
    state->setCurrent_mA = 0.0f;
    return;
  }

  if (state->mode == MODE_CC || state->mode == MODE_CA) {
    state->setCurrent_mA = reading * 1000.0f;
    return;
  }

  if (state->mode == MODE_CP) {
    state->setPower_W = reading * 1000.0f;
    if (state->measuredVoltage_V > 0.05f) {
      state->setCurrent_mA = state->setPower_W / state->measuredVoltage_V;
    } else {
      state->setCurrent_mA = 0.0f;
    }
    return;
  }

  state->setResistance_Ohm = reading;
  if (reading > 0.0f) {
    state->setCurrent_mA = (state->measuredVoltage_V / reading) * 1000.0f;
  } else {
    state->setCurrent_mA = 0.0f;
  }
}

void core_mode_apply_selection(SystemState *state, bool shiftPressed, char key) {
  if (state == nullptr) return;

  // Shared behavior: leaving current mode always turns load off and re-inits template.
  state->loadEnabled = false;
  state->setCurrent_mA = 0.0f;
  state->modeInitialized = false;

  if (!shiftPressed) {
    state->functionIndex = (state->functionIndex + 1) % 6;
    switch (state->functionIndex) {
      case 0: state->mode = MODE_CC; break;
      case 1: state->mode = MODE_CP; break;
      case 2: state->mode = MODE_CR; break;
      case 3: state->mode = MODE_BC; state->modeConfigured = false; break;
      case 4: state->mode = MODE_TC; state->modeConfigured = false; break;
      case 5: state->mode = MODE_TL; state->modeConfigured = false; break;
      default: break;
    }
    return;
  }

  switch (key) {
    case '1': state->functionIndex = 0; state->mode = MODE_CC; break;
    case '2': state->functionIndex = 1; state->mode = MODE_CP; break;
    case '3': state->functionIndex = 2; state->mode = MODE_CR; break;
    case '4': state->functionIndex = 3; state->mode = MODE_BC; state->modeConfigured = false; break;
    case '5': state->functionIndex = 4; state->mode = MODE_TC; state->modeConfigured = false; break;
    case '6': state->functionIndex = 5; state->mode = MODE_TL; state->modeConfigured = false; break;
    case 'C': state->mode = MODE_CA; state->modeConfigured = false; break;
    case '<': state->modeConfigured = false; break;
    default: break;
  }
}

void core_mode_update_ui_screen(SystemState *state) {
  if (state == nullptr) return;

  if (state->pendingConfigSection == ConfigSection::Limits || state->openLimitsConfigEvent) {
    state->uiScreen = UiScreen::MenuLimits;
    return;
  }

  if (state->pendingConfigSection == ConfigSection::Calibration || state->calibrationValueConfirmEvent) {
    state->uiScreen = UiScreen::MenuCalibration;
    return;
  }

  state->uiScreen = UiScreen::Home;
}

