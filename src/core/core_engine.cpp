#include "core_engine.h"

#include <Arduino.h>

namespace {
SystemState g_state = {0};
unsigned long g_lastTickMs = 0;
constexpr uint8_t MODE_CC = 0;
constexpr int CC_CURSOR_MIN = 8;
constexpr int CC_CURSOR_MAX = 12;

void applyEncoderStep(int direction) {
  if (direction > 0) {
    g_state.encoderPositionRaw += g_state.encoderStep;
  } else if (direction < 0) {
    g_state.encoderPositionRaw -= g_state.encoderStep;
  }

  if (g_state.encoderPositionRaw < 0.0f) {
    g_state.encoderPositionRaw = 0.0f;
  }
  if (g_state.encoderPositionRaw > g_state.encoderMaxRaw) {
    g_state.encoderPositionRaw = g_state.encoderMaxRaw;
  }
}
}

void core_init() {
  g_lastTickMs = millis();
}

void core_sync_from_legacy(const SystemState &state) {
  g_state = state;
}

void core_dispatch(const UserAction &action) {
  switch (action.type) {
    case ActionType::EncoderDelta:
      g_state.lastEncoderDelta = action.value;
      if (g_state.mode == MODE_CC) {
        applyEncoderStep((action.value > 0) ? 1 : ((action.value < 0) ? -1 : 0));
      }
      break;

    case ActionType::KeyPressed:
      g_state.lastKeyPressed = action.key;
      if (g_state.mode == MODE_CC) {
        if (action.key == 'U') {
          applyEncoderStep(1);
        } else if (action.key == 'D') {
          applyEncoderStep(-1);
        } else if (action.key == 'L') {
          g_state.cursorPosition--;
        } else if (action.key == 'R') {
          g_state.cursorPosition++;
        }

        if (g_state.cursorPosition < CC_CURSOR_MIN) g_state.cursorPosition = CC_CURSOR_MIN;
        if (g_state.cursorPosition > CC_CURSOR_MAX) g_state.cursorPosition = CC_CURSOR_MAX;
      }
      break;

    case ActionType::LoadToggle:
      g_state.loadEnabled = !g_state.loadEnabled;
      g_state.loadToggleEvent = true;
      if (!g_state.loadEnabled) {
        g_state.setCurrent_mA = 0.0f;
      }
      break;

    case ActionType::None:
    default:
      return;
  }

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