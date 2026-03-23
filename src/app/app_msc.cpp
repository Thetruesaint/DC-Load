#include "app_msc.h"

#include "app_loop.h"
#include "app_mode_context.h"
#include "app_mode_keys.h"
#include "app_ui_context.h"

MscKeyDecision app_handle_msc_key_decision(char key) {
  const bool configAllowed = app_mode_config_allowed();
  return app_route_msc_key(key, configAllowed);
}

bool app_handle_msc_keys(char key) {
  switch (app_handle_msc_key_decision(key)) {
    case MscKeyDecision::ExitMode:
      app_ui_request_clear_cursor_blink();
      return false;

    case MscKeyDecision::OpenConfig:
      app_push_action(make_open_limits_config_action());
      return true;

    case MscKeyDecision::ToggleTraceOverlay:
      app_push_action(make_toggle_trace_overlay_action());
      return true;

    case MscKeyDecision::Continue:
    default:
      return true;
  }
}

bool app_msc_shift_active() {
  return app_mode_shift_active();
}


