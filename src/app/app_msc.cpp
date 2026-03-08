#include "app_msc.h"

#include "../variables.h"
#include "../ui_lcd.h"
#include "../legacy/legacy_hooks.h"
#include "app_mode_keys.h"

bool app_handle_msc_keys(char key) {
  const bool configAllowed = (Mode != TC && Mode != TL);

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
