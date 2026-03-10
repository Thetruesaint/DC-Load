#include "legacy_mode_dispatch.h"

#include "../app/app_mode_state_context.h"
#include "../config/system_constants.h"
#include "legacy_mode_bc.h"
#include "legacy_mode_ca.h"
#include "legacy_mode_transient.h"

void legacy_run_mode_logic() {
  switch (app_mode_state_mode()) {
    case CC:
    case CP:
    case CR:
      break;
    case BC: legacy_battery_mode(); break;
    case TC: legacy_transient_cont_mode(); break;
    case TL: legacy_transient_list_mode(); break;
    case CA: legacy_calibration_mode(); break;
    case UNKNOWN:
    default: break;
  }
}
