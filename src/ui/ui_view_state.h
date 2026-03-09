#ifndef UI_VIEW_STATE_H
#define UI_VIEW_STATE_H

#include <stdint.h>

struct UiViewState {
  bool modeInitialized;
  bool loadEnabled;
  uint8_t mode;
  int cursorPosition;

  float measuredCurrent_A;
  float measuredVoltage_V;
  float measuredPower_W;
  float readingValue;
};

inline UiViewState ui_view_state_make_default() {
  UiViewState state = {0};
  return state;
}

#endif
