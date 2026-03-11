#include "app_mode_keys.h"

#include <Arduino.h>

#include "app_loop.h"

namespace {
constexpr unsigned long SHIFT_TIMEOUT_MS = 700UL;

bool is_shift_mode_key(char key) {
  return key >= '1' && key <= '6';
}
}

MscKeyDecision app_route_msc_key(char key, bool configAllowed) {
  static bool shiftPressed = false;
  static unsigned long shiftPressedAt = 0;

  const unsigned long now = millis();
  if (shiftPressed && (now - shiftPressedAt) > SHIFT_TIMEOUT_MS) {
    shiftPressed = false;
  }

  if (key == 'M') {
    shiftPressed = false;
    app_push_action(make_mode_select_action(false));
    return MscKeyDecision::ExitMode;
  }

  if (shiftPressed) {
    shiftPressed = false;
    if (is_shift_mode_key(key)) {
      app_push_action(make_mode_select_action(true, key));
      return MscKeyDecision::ExitMode;
    }
    return MscKeyDecision::Consumed;
  }

  if (key == 'S') {
    shiftPressed = true;
    shiftPressedAt = now;
    return MscKeyDecision::Consumed;
  }

  if (configAllowed && key == 'C') {
    return MscKeyDecision::OpenConfig;
  }

  return MscKeyDecision::Continue;
}
