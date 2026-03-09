#ifndef CORE_STATE_H
#define CORE_STATE_H

#include <stdint.h>

struct SystemState {
  float setCurrent_mA;
  float setPower_W;
  float setResistance_Ohm;

  float measuredCurrent_A;
  float measuredVoltage_V;
  float measuredPower_W;
  float temp_C;

  float readingValue;
  float encoderPositionRaw;
  float encoderStep;
  float encoderMaxRaw;
  float currentCutOffA;

  int cursorPosition;
  int functionIndex;

  int32_t lastEncoderDelta;
  char lastKeyPressed;
  bool loadToggleEvent;
  bool calibrationValueConfirmEvent;
  bool openLimitsConfigEvent;
  float calibrationRealValue;
  uint32_t actionCounter;

  bool loadEnabled;
  uint8_t mode;
  bool modeInitialized;
  bool modeConfigured;
};

inline SystemState core_state_make_default() {
  SystemState state = {0};
  return state;
}

inline void core_state_clear_one_shot_events(SystemState *state) {
  if (state == nullptr) return;
  state->loadToggleEvent = false;
  state->calibrationValueConfirmEvent = false;
  state->openLimitsConfigEvent = false;
  state->calibrationRealValue = 0.0f;
}

#endif
