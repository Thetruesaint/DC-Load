#include "app_load_output.h"

#include "../config/system_constants.h"
#include "../hw/hw_objects.h"
#include "app_calibration_context.h"
#include "app_load_context.h"

#define Out_Curr_Calib_Fact (app_calibration_out_curr_factor_ref())
#define Out_Curr_Calib_Offs (app_calibration_out_curr_offset_ref())

namespace {
#ifndef WOKWI_SIMULATION
bool g_dacReady = false;
#endif
}

void app_load_output_set_dac_ready(bool ready) {
#ifndef WOKWI_SIMULATION
  g_dacReady = ready;
#else
  (void)ready;
#endif
}

void app_load_output_apply() {
  digitalWrite(MOSFONOFF, LOW);

  if (app_load_is_enabled()) {
#ifndef WOKWI_SIMULATION
    if (g_dacReady) {
      const float targetCurrent = app_load_set_current_mA();
      const float outputFactor = constrain(Out_Curr_Calib_Fact, 0.9f, 1.1f);
      const float calibratedCurrent =
          (targetCurrent - Out_Curr_Calib_Offs) / outputFactor;
      const unsigned long setDAC = (unsigned long)(
          constrain(calibratedCurrent, 0.0f, 12000.0f) * OUT_CURR_FACT);
      dac.setVoltage(setDAC, false);
    }
#endif
  } else {
#ifndef WOKWI_SIMULATION
    if (g_dacReady) {
      dac.setVoltage(0, false);
    }
#endif
  }
}

void app_load_output_off() {
#ifndef WOKWI_SIMULATION
  if (g_dacReady) {
    dac.setVoltage(0, false);
  }
#endif
  app_load_set_enabled(false);
  app_load_set_set_current_mA(0.0f);
}

void app_load_output_emergency_off() {
  digitalWrite(MOSFONOFF, HIGH);
#ifndef WOKWI_SIMULATION
  if (g_dacReady) {
    dac.setVoltage(0, false);
  }
#endif
  app_load_set_enabled(false);
  app_load_set_set_current_mA(0.0f);
}

