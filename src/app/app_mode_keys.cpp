#include "app_mode_keys.h"

#include <Arduino.h>

#include "app_loop.h"

namespace {
bool shiftPressed = false;

bool is_shift_mode_key(char key) {
  return key >= '1' && key <= '6';
}
}

MscKeyDecision app_route_msc_key(char key, bool configAllowed) {
  if (key == 'M') {
    shiftPressed = false;
    app_push_action(make_mode_select_action(false));
    return MscKeyDecision::ExitMode;
  }

  if (shiftPressed) {
    if (key == 'S') {
      shiftPressed = false;
      return MscKeyDecision::Consumed;
    }
    if (is_shift_mode_key(key)) {
      shiftPressed = false;
      app_push_action(make_mode_select_action(true, key));
      return MscKeyDecision::ExitMode;
    }
    return MscKeyDecision::Consumed;
  }

  if (key == 'S') {
    shiftPressed = true;
    return MscKeyDecision::Consumed;
  }

  if (configAllowed && key == 'C') {
    return MscKeyDecision::OpenConfig;
  }

  return MscKeyDecision::Continue;
}

bool app_mode_shift_active() {
  return shiftPressed;
}
