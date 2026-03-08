#include "app_runtime.h"

#include "../variables.h"
#include "../funciones.h"
#include "app_loop.h"

void app_run_cycle() {
  Temp_Control();
  Read_Encoder();
  Read_Keypad();
  Read_Load_Button();
  Read_Volts_Current();
  Check_Limits();
  DAC_Control();

  switch (Mode) {
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

  Update_LCD();
#ifndef WOKWI_SIMULATION
  // Update_TFT();
#endif

  app_tick();
}
