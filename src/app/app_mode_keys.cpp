#include "app_mode_keys.h"

#include "app_loop.h"

namespace {
bool handle_mode_hotkeys(char key) {
  static bool shiftPressed = false;

  if (key == 'M') {
    app_push_action(ActionType::ModeSelect, 0, '\0');
    return false;
  }

  if (shiftPressed) {
    shiftPressed = false;
    app_push_action(ActionType::ModeSelect, 1, key);
    return false;
  }

  if (key == 'S') {
    shiftPressed = true;
    return true;
  }

  return true;
}
}

MscKeyDecision app_route_msc_key(char key, bool configAllowed) {
  if (!handle_mode_hotkeys(key)) {
    return MscKeyDecision::ExitMode;
  }

  if (configAllowed && key == 'C') {
    return MscKeyDecision::OpenConfig;
  }

  return MscKeyDecision::Continue;
}
