#include "app_msc.h"

#include "../ui_lcd.h"
#include "app_mode_keys.h"
#include "app_mode_context.h"
#include "app_loop.h"

bool app_handle_msc_keys(char key) {
  const bool configAllowed = app_mode_config_allowed();

  switch (app_route_msc_key(key, configAllowed)) {
    case MscKeyDecision::ExitMode:
      noCursorLCD();
      blinkOffLCD();
      return false;

    case MscKeyDecision::OpenConfig:
      app_push_action(make_open_limits_config_action());
      return true;

    case MscKeyDecision::Continue:
    default:
      return true;
  }
}
