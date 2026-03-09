#include "legacy_dac_control.h"

#include "../config/system_constants.h"
#include "../hw/hw_objects.h"
#include "../app/app_load_context.h"
#include "../app/app_calibration_context.h"
#define Sns_Volt_Calib_Fact (app_calibration_sns_volt_factor_ref())
#define Sns_Volt_Calib_Offs (app_calibration_sns_volt_offset_ref())
#define Sns_Curr_Calib_Fact (app_calibration_sns_curr_factor_ref())
#define Sns_Curr_Calib_Offs (app_calibration_sns_curr_offset_ref())
#define Out_Curr_Calib_Fact (app_calibration_out_curr_factor_ref())
#define Out_Curr_Calib_Offs (app_calibration_out_curr_offset_ref())


void legacy_dac_control() {
#ifndef WOKWI_SIMULATION
  if (app_load_is_enabled()) {
    const float targetCurrent = app_load_set_current_mA();
    const unsigned long setDAC = (unsigned long)(constrain(targetCurrent * Out_Curr_Calib_Fact + Out_Curr_Calib_Offs, 0.0f, 12000.0f) * OUT_CURR_FACT);
    dac.setVoltage(setDAC, false);
  } else {
    dac.setVoltage(0, false);
    app_load_set_set_current_mA(0.0f);
  }
#else
  if (!app_load_is_enabled()) {
    app_load_set_set_current_mA(0.0f);
  }
#endif
}

