#ifndef APP_MODE_KEYS_H
#define APP_MODE_KEYS_H

enum class MscKeyDecision {
  Continue = 0,
  ExitMode,
  OpenConfig
};

MscKeyDecision app_route_msc_key(char key, bool configAllowed);

#endif
