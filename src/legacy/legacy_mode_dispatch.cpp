#include "legacy_mode_dispatch.h"

#include "../app/app_mode_state_context.h"
#include "../config/system_constants.h"
#include "legacy_mode_bc.h"
#include "legacy_mode_ca.h"
#include "legacy_mode_cc.h"
#include "legacy_mode_cp.h"
#include "legacy_mode_cr.h"
#include "legacy_mode_transient.h"

void legacy_run_mode_logic() {
  switch (app_mode_state_mode()) {
    case CC: legacy_const_current_mode(); break;
    case CP: legacy_const_power_mode(); break;
    case CR: legacy_const_resistance_mode(); break;
    case BC: legacy_battery_mode(); break;
    case TC: legacy_transient_cont_mode(); break;
    case TL: legacy_transient_list_mode(); break;
    case CA: legacy_calibration_mode(); break;
    case UNKNOWN:
    default: break;
  }
}
