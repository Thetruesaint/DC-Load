#include "legacy_bridge.h"

#include <math.h>

#include "../variables.h"

namespace {
float g_prevEncoderPosition = 0.0f;
bool g_prevToggle = false;
}

SystemState legacy_capture_state() {
  SystemState state = {0};

  state.setCurrent_mA = setCurrent;
  state.setPower_W = setPower;
  state.setResistance_Ohm = setResistance;

  state.measuredCurrent_A = current;
  state.measuredVoltage_V = voltage;
  state.measuredPower_W = voltage * current;
  state.temp_C = static_cast<float>(temp);

  state.encoderPositionRaw = encoderPosition;
  state.lastEncoderDelta = 0;
  state.lastKeyPressed = '\0';
  state.loadToggleEvent = false;
  state.actionCounter = 0;

  state.loadEnabled = toggle;
  state.mode = static_cast<uint8_t>(Mode);
  state.modeInitialized = modeInitialized;
  state.modeConfigured = modeConfigured;

  return state;
}

void legacy_reset_action_poll() {
  g_prevEncoderPosition = encoderPosition;
  g_prevToggle = toggle;
}

size_t legacy_poll_actions(UserAction *actions, size_t maxActions) {
  if (actions == nullptr || maxActions == 0) return 0;

  size_t count = 0;

  const float encNow = encoderPosition;
  const float encDiff = encNow - g_prevEncoderPosition;
  if (encDiff != 0.0f && count < maxActions) {
    actions[count++] = {ActionType::EncoderDelta, static_cast<int32_t>(lroundf(encDiff)), '\0'};
    g_prevEncoderPosition = encNow;
  }

  if (customKey != NO_KEY && count < maxActions) {
    actions[count++] = {ActionType::KeyPressed, 0, customKey};
  }

  if (toggle != g_prevToggle && count < maxActions) {
    actions[count++] = {ActionType::LoadToggle, 0, '\0'};
    g_prevToggle = toggle;
  }

  return count;
}