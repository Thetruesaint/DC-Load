#include "legacy_hooks.h"

#include "../funciones.h"
#include "../config/system_constants.h"
#include "../hw/hw_objects.h"
#include "../app/app_mode_state_context.h"

void legacy_open_config_limits() {
  Config_Limits();
}

void legacy_reset_input_pointers() {
  Reset_Input_Pointers();
}

void legacy_run_mode_logic() {
  switch (app_mode_state_mode()) {
    case CC:  Const_Current_Mode(); break;
    case CP:  Const_Power_Mode(); break;
    case CR:  Const_Resistance_Mode(); break;
    case BC:  Battery_Mode(); break;
    case TC:  Transient_Cont_Mode(); break;
    case TL:  Transient_List_Mode(); break;
    case CA:  Calibration_Mode(); break;
    case UNKNOWN:
    default: break;
  }
}
