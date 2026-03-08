#include "app_msc.h"

#include "../funciones.h"
#include "app_mode_keys.h"

bool app_handle_msc_keys(char key) {
  const bool configAllowed = (Mode != TC && Mode != TL);

  switch (app_route_msc_key(key, configAllowed)) {
    case MscKeyDecision::ExitMode:
      noCursorLCD();
      blinkOffLCD();
      return false;

    case MscKeyDecision::OpenConfig:
      Config_Limits();
      return true;

    case MscKeyDecision::Continue:
    default:
      return true;
  }
}
