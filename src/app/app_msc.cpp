#include "app_msc.h"

#include "../ui_lcd.h"
#include "../legacy/legacy_hooks.h"
#include "app_mode_keys.h"
#include "app_mode_context.h"

bool app_handle_msc_keys(char key) {
  const bool configAllowed = app_mode_config_allowed();

  switch (app_route_msc_key(key, configAllowed)) {
    case MscKeyDecision::ExitMode:
      noCursorLCD();
      blinkOffLCD();
      return false;

    case MscKeyDecision::OpenConfig:
      legacy_open_config_limits();
      return true;

    case MscKeyDecision::Continue:
    default:
      return true;
  }
}

