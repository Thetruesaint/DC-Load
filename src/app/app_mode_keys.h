#ifndef APP_MODE_KEYS_H
#define APP_MODE_KEYS_H

enum class MscKeyDecision {
  Continue = 0,
  Consumed,
  ExitMode,
  OpenConfig,
  ToggleTraceOverlay
};

MscKeyDecision app_route_msc_key(char key, bool configAllowed);
bool app_mode_shift_active();

#endif
