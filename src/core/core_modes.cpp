#include "core_modes.h"

#include "core_config_flow.h"

namespace {
constexpr uint8_t MODE_CC = 0;
constexpr uint8_t MODE_CP = 1;
constexpr uint8_t MODE_CR = 2;
constexpr uint8_t MODE_BC = 3;
constexpr uint8_t MODE_TC = 4;
constexpr uint8_t MODE_TL = 5;
constexpr uint8_t MODE_CA = 6;
constexpr int DECIMAL_CURSOR_COL = 9;
constexpr float TC_MIN_PERIOD_MS = 100.0f;
constexpr float TC_MAX_PERIOD_MS = 10000.0f;

bool mode_uses_managed_input(uint8_t mode) {
  return mode == MODE_CC || mode == MODE_CP || mode == MODE_CR || mode == MODE_BC || mode == MODE_TC || mode == MODE_CA;
}

bool mode_uses_managed_setpoints(uint8_t mode) {
  return mode == MODE_CC || mode == MODE_CP || mode == MODE_CR || mode == MODE_TC || mode == MODE_CA;
}

UiScreen battery_setup_target_screen(const SystemState &state) {
  switch (state.batterySetupStage) {
    case 1: return UiScreen::BatterySetupCustomCutoff;
    case 2: return UiScreen::BatterySetupCellCount;
    default: return UiScreen::BatterySetupTask;
  }
}

UiScreen transient_cont_setup_target_screen(const SystemState &state) {
  switch (state.transientSetupStage) {
    case 1: return UiScreen::TransientContSetupHigh;
    case 2: return UiScreen::TransientContSetupPeriod;
    default: return UiScreen::TransientContSetupLow;
  }
}

UiScreen transient_list_setup_target_screen(const SystemState &state) {
  return (state.transientListSetupStage == 0) ? UiScreen::TransientListSetupCount : UiScreen::TransientListSetupStep;
}

int cursor_min_by_mode(uint8_t mode) {
  return (mode == MODE_CP || mode == MODE_CR) ? 6 : 8;
}

int cursor_max_by_mode(uint8_t mode) {
  return (mode == MODE_CP || mode == MODE_CR) ? 10 : 12;
}

float factor_for_cursor(uint8_t mode, int cursor) {
  if (mode == MODE_TC) {
    switch (cursor) {
      case 8: return 10000.0f;
      case 9: return 1000.0f;
      case 10: return 100.0f;
      case 11: return 10.0f;
      case 12: return 1.0f;
      default: return 1000.0f;
    }
  }

  switch (cursor) {
    case 6: return 100000.0f;
    case 7: return 10000.0f;
    case 9: return 1000.0f;
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
  if (state->mode != MODE_TC && state->cursorPosition == DECIMAL_CURSOR_COL) {
    state->cursorPosition = cursor_min_by_mode(state->mode);
  }
  state->encoderStep = factor_for_cursor(state->mode, state->cursorPosition);
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
    if (state->mode != MODE_TC && state->cursorPosition == DECIMAL_CURSOR_COL) state->cursorPosition++;
  } else if (direction < 0) {
    state->cursorPosition--;
    if (state->mode != MODE_TC && state->cursorPosition == DECIMAL_CURSOR_COL) state->cursorPosition--;
  }

  wrap_cursor_by_mode(state);
  state->encoderStep = factor_for_cursor(state->mode, state->cursorPosition);
}

void core_mode_update_setpoints(SystemState *state) {
  if (state == nullptr || !mode_uses_managed_setpoints(state->mode)) return;

  if (state->mode == MODE_TC) {
    if (!state->modeInitialized || state->uiScreen != UiScreen::Home) return;

    float periodMs = state->encoderPositionRaw;
    if (periodMs < TC_MIN_PERIOD_MS) periodMs = TC_MIN_PERIOD_MS;
    if (periodMs > TC_MAX_PERIOD_MS) periodMs = TC_MAX_PERIOD_MS;

    state->transientPeriodMs = periodMs;
    state->encoderPositionRaw = periodMs;
    return;
  }

  const float minReading = (state->mode == MODE_CR) ? 0.1f : 0.0f;
  float maxReading = state->encoderMaxRaw / 1000.0f;
  if (maxReading < minReading) maxReading = minReading;

  float reading = state->encoderPositionRaw / 1000.0f;
  if (reading < minReading) reading = minReading;
  if (reading > maxReading) reading = maxReading;

  state->readingValue = reading;
  state->encoderPositionRaw = reading * 1000.0f;

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

namespace {
void transient_list_setup_begin(SystemState *state) {
  if (state == nullptr) return;
  state->transientListSetupStage = 0;
  state->transientListDraftStepCount = 0;
  state->transientListDraftStepIndex = 0;
  state->transientListDraftField = 0;
  for (int i = 0; i < 10; ++i) {
    state->transientListDraftCurrentsA[i] = 0.0f;
    state->transientListDraftPeriodsMs[i] = 0.0f;
  }
  state->transientListInputText[0] = '\0';
  state->transientListInputLength = 0;
  state->transientListInputHasDecimal = false;
}
}

void core_mode_apply_selection(SystemState *state, bool shiftPressed, char key) {
  if (state == nullptr) return;

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
    if (state->mode == MODE_TC) {
      state->transientSetupStage = 0;
      state->transientLowCurrentA = state->currentCutOffA;
      state->transientHighCurrentA = state->currentCutOffA;
      state->transientPeriodMs = 1000.0f;
      state->transientInputText[0] = '\0';
      state->transientInputLength = 0;
      state->transientInputHasDecimal = false;
    }
    if (state->mode == MODE_TL) {
      transient_list_setup_begin(state);
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

  if (state->mode == MODE_TC) {
    state->transientSetupStage = 0;
    state->transientLowCurrentA = state->currentCutOffA;
    state->transientHighCurrentA = state->currentCutOffA;
    state->transientPeriodMs = 1000.0f;
    state->transientInputText[0] = '\0';
    state->transientInputLength = 0;
    state->transientInputHasDecimal = false;
  }

  if (state->mode == MODE_TL) {
    transient_list_setup_begin(state);
  }
}

void core_mode_update_ui_screen(SystemState *state) {
  if (state == nullptr) return;

  const UiScreen configScreen = core_config_target_screen(*state);
  if (configScreen != UiScreen::Home) {
    state->uiScreen = configScreen;
    return;
  }

  if (state->mode == MODE_BC && !state->modeConfigured) {
    state->uiScreen = battery_setup_target_screen(*state);
    return;
  }

  if (state->mode == MODE_TC && !state->modeConfigured) {
    state->uiScreen = transient_cont_setup_target_screen(*state);
    return;
  }

  if (state->mode == MODE_TL && !state->modeConfigured) {
    state->uiScreen = transient_list_setup_target_screen(*state);
    return;
  }

  state->uiScreen = UiScreen::Home;
}




