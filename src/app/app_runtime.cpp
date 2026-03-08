#include "app_runtime.h"

#include "../funciones.h"
#include "../legacy/legacy_hooks.h"
#include "app_loop.h"

void app_run_cycle() {
  Temp_Control();
  Read_Encoder();
  Read_Keypad();
  Read_Load_Button();
  Read_Volts_Current();
  Check_Limits();
  DAC_Control();

  legacy_run_mode_logic();

  Update_LCD();
#ifndef WOKWI_SIMULATION
  // Update_TFT();
#endif

  app_tick();
}
