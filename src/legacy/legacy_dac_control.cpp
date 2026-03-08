#include "legacy_dac_control.h"

#include "../variables.h"

void legacy_dac_control() {
#ifndef WOKWI_SIMULATION
  if (toggle) {
    setDAC = (unsigned long)(constrain(setCurrent * Out_Curr_Calib_Fact + Out_Curr_Calib_Offs, 0.0f, 12000.0f) * OUT_CURR_FACT);
    dac.setVoltage(setDAC, false);
  } else {
    dac.setVoltage(0, false);
    setCurrent = 0;
  }
#else
  if (!toggle) {
    setCurrent = 0;
  }
#endif
}
