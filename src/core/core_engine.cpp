#include "core_engine.h"

#include <Arduino.h>

namespace {
SystemState g_state = {0};
unsigned long g_lastTickMs = 0;
constexpr uint8_t MODE_CC = 0;
constexpr uint8_t MODE_CP = 1;
constexpr uint8_t MODE_CR = 2;
constexpr int DECIMAL_CURSOR_COL = 9;

bool modeUsesSetpointCursor(uint8_t mode) {
  return mode == MODE_CC || mode == MODE_CP || mode == MODE_CR;
}

int cursorMinByMode(uint8_t mode) {
  return (mode == MODE_CC) ? 8 : 6;
}

int cursorMaxByMode(uint8_t mode) {
  return (mode == MODE_CC) ? 12 : 10;
}

float factorForCursor(int cursor) {
  switch (cursor) {
    case 6: return 100000.0f;
    case 7: return 10000.0f;
    case 10: return 100.0f;
    case 11: return 10.0f;
    case 12: return 1.0f;
    default: return 1000.0f;
  }
}

void wrapCursorByMode() {
  const int minPos = cursorMinByMode(g_state.mode);
  const int maxPos = cursorMaxByMode(g_state.mode);

  if (g_state.cursorPosition > maxPos) g_state.cursorPosition = minPos;
  if (g_state.cursorPosition < minPos) g_state.cursorPosition = maxPos;
}

void moveCursor(int direction) {
  if (direction > 0) {
    g_state.cursorPosition++;
    if (g_state.cursorPosition == DECIMAL_CURSOR_COL) g_state.cursorPosition++;
  } else if (direction < 0) {
    g_state.cursorPosition--;
    if (g_state.cursorPosition == DECIMAL_CURSOR_COL) g_state.cursorPosition--;
  }

  wrapCursorByMode();
  g_state.encoderStep = factorForCursor(g_state.cursorPosition);
}

void applyEncoderStep(int direction) {
  if (direction > 0) {
    g_state.encoderPositionRaw += g_state.encoderStep;
  } else if (direction < 0) {
    g_state.encoderPositionRaw -= g_state.encoderStep;
  }

  if (g_state.encoderPositionRaw < 0.0f) g_state.encoderPositionRaw = 0.0f;
  if (g_state.encoderPositionRaw > g_state.encoderMaxRaw) g_state.encoderPositionRaw = g_state.encoderMaxRaw;
}

void normalizeManagedModeState() {
  if (!modeUsesSetpointCursor(g_state.mode)) return;

  wrapCursorByMode();
  if (g_state.cursorPosition == DECIMAL_CURSOR_COL) {
    g_state.cursorPosition = cursorMinByMode(g_state.mode);
  }
  g_state.encoderStep = factorForCursor(g_state.cursorPosition);
}

void updateManagedSetpoints() {
  if (!modeUsesSetpointCursor(g_state.mode)) return;

  const float minReading = (g_state.mode == MODE_CR) ? 0.1f : 0.0f;
  float maxReading = g_state.encoderMaxRaw / 1000.0f;
  if (maxReading < minReading) maxReading = minReading;

  float reading = g_state.encoderPositionRaw / 1000.0f;
  if (reading < minReading) reading = minReading;
  if (reading > maxReading) reading = maxReading;

  g_state.readingValue = reading;
  g_state.encoderPositionRaw = reading * 1000.0f;

  if (!g_state.loadEnabled) {
    g_state.setCurrent_mA = 0.0f;
    return;
  }

  if (g_state.mode == MODE_CC) {
    g_state.setCurrent_mA = reading * 1000.0f;
    return;
  }

  if (g_state.mode == MODE_CP) {
    g_state.setPower_W = reading * 1000.0f;
    if (g_state.measuredVoltage_V > 0.05f) {
      g_state.setCurrent_mA = g_state.setPower_W / g_state.measuredVoltage_V;
    } else {
      g_state.setCurrent_mA = 0.0f;
    }
    return;
  }

  g_state.setResistance_Ohm = reading;
  if (reading > 0.0f) {
    g_state.setCurrent_mA = (g_state.measuredVoltage_V / reading) * 1000.0f;
  } else {
    g_state.setCurrent_mA = 0.0f;
  }
}
}

void core_init() {
  g_lastTickMs = millis();
}

void core_sync_from_legacy(const SystemState &state) {
  g_state = state;
  normalizeManagedModeState();
  updateManagedSetpoints();
}

void core_dispatch(const UserAction &action) {
  switch (action.type) {
    case ActionType::EncoderDelta:
      g_state.lastEncoderDelta = action.value;
      if (modeUsesSetpointCursor(g_state.mode)) {
        applyEncoderStep((action.value > 0) ? 1 : ((action.value < 0) ? -1 : 0));
      }
      break;

    case ActionType::EncoderButtonPress:
      if (modeUsesSetpointCursor(g_state.mode)) {
        moveCursor(1);
      }
      break;

    case ActionType::KeyPressed:
      g_state.lastKeyPressed = action.key;
      if (modeUsesSetpointCursor(g_state.mode)) {
        if (action.key == 'U') {
          applyEncoderStep(1);
        } else if (action.key == 'D') {
          applyEncoderStep(-1);
        } else if (action.key == 'L') {
          moveCursor(-1);
        } else if (action.key == 'R') {
          moveCursor(1);
        }
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

  updateManagedSetpoints();
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