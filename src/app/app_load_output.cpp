#include "app_load_output.h"

#include "../config/system_constants.h"
#include "../hw/hw_objects.h"
#include "app_calibration_context.h"
#include "app_load_context.h"

#define Out_Curr_Calib_Fact (app_calibration_out_curr_factor_ref())
#define Out_Curr_Calib_Offs (app_calibration_out_curr_offset_ref())

void app_load_output_apply() {
#ifndef WOKWI_SIMULATION
  if (app_load_is_enabled()) {
    const float targetCurrent = app_load_set_current_mA();
    const unsigned long setDAC = (unsigned long)(constrain(targetCurrent * Out_Curr_Calib_Fact + Out_Curr_Calib_Offs, 0.0f, 12000.0f) * OUT_CURR_FACT);
    dac.setVoltage(setDAC, false);
  } else {
    dac.setVoltage(0, false);
  }
#else
  (void)app_load_is_enabled();
#endif
}
void app_load_output_off() {
#ifndef WOKWI_SIMULATION
  dac.setVoltage(0, false);
#endif
  app_load_set_enabled(false);
  app_load_set_set_current_mA(0.0f);
}

