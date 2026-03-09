#include "core_engine.h"

#include <Arduino.h>

#include "core_modes.h"

namespace {
SystemState g_state = {0};
unsigned long g_lastTickMs = 0;
}

void core_init() {
  g_lastTickMs = millis();
}

void core_sync_from_legacy(const SystemState &state) {
  g_state = state;
  core_mode_normalize_state(&g_state);
  core_mode_update_setpoints(&g_state);
}

void core_dispatch(const UserAction &action) {
  switch (action.type) {
    case ActionType::EncoderDelta:
      g_state.lastEncoderDelta = action.value;
      core_mode_apply_encoder_delta(&g_state, (action.value > 0) ? 1 : ((action.value < 0) ? -1 : 0));
      break;

    case ActionType::EncoderButtonPress:
      core_mode_move_cursor(&g_state, 1);
      break;

    case ActionType::KeyPressed:
      g_state.lastKeyPressed = action.key;
      if (action.key == 'U') {
        core_mode_apply_encoder_delta(&g_state, 1);
      } else if (action.key == 'D') {
        core_mode_apply_encoder_delta(&g_state, -1);
      } else if (action.key == 'L') {
        core_mode_move_cursor(&g_state, -1);
      } else if (action.key == 'R') {
        core_mode_move_cursor(&g_state, 1);
      }
      break;

    case ActionType::LoadToggle:
      g_state.loadEnabled = !g_state.loadEnabled;
      g_state.loadToggleEvent = true;
      if (!g_state.loadEnabled) {
        g_state.setCurrent_mA = 0.0f;
      }
      break;

    case ActionType::ModeSelect:
      core_mode_apply_selection(&g_state, action.value != 0, action.key);
      break;

    case ActionType::ValueConfirm:
      g_state.encoderPositionRaw = static_cast<float>(action.value);
      break;

    case ActionType::CalibrationValueConfirm:
      g_state.calibrationRealValue = static_cast<float>(action.value) / 1000.0f;
      g_state.calibrationValueConfirmEvent = true;
      break;

    case ActionType::None:
    default:
      return;
  }

  core_mode_update_setpoints(&g_state);
  g_state.actionCounter++;
}

void core_tick_10ms() {
  const unsigned long now = millis();
  if ((now - g_lastTickMs) < 10UL) return;
  g_lastTickMs = now;
}

const SystemState &core_get_state() {
  return g_state;
}
