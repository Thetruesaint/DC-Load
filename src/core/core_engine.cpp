#include "core_engine.h"

#include <Arduino.h>

namespace {
SystemState g_state = {0};
unsigned long g_lastTickMs = 0;
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
      break;
    case ActionType::KeyPressed:
      g_state.lastKeyPressed = action.key;
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