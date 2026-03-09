#include "legacy_mode_transient.h"

#include "../variables.h"
#include "../ui_lcd.h"
#include "../funciones.h"
#include "../app/app_mode_state_context.h"
#include "../app/app_limits_context.h"
#include "../app/app_load_context.h"
#include "../app/app_value_result_context.h"

void legacy_transient_cont_mode() {
  if (!app_mode_state_configured()) {
    legacy_transient_cont_setup();
    return;
  }

  if (!app_mode_state_initialized()) {
    printLCD_S(3, 2, String(LowCurrent, 3));
    writeLCD(byte(0));
    printLCD_S(14, 2, String(HighCurrent, 3));
    writeLCD(byte(0));
    printLCD(0, 0, F("TC LOAD"));
    printLCD(0, 2, F("I1>"));
    printLCD(11, 2, F("I2>"));
    printLCD(2, 3, F("Time: "));

    if (transientPeriod < 10) {
      Print_Spaces(7, 3, 4);
      printLCDRaw(transientPeriod);
    } else if (transientPeriod < 100) {
      Print_Spaces(7, 3, 3);
      printLCDRaw(transientPeriod);
    } else if (transientPeriod < 1000) {
      Print_Spaces(7, 3, 2);
      printLCDRaw(transientPeriod);
    } else if (transientPeriod < 10000) {
      Print_Spaces(7, 3);
      printLCDRaw(transientPeriod);
    } else {
      setCursorLCD(7, 3);
      printLCDRaw(transientPeriod);
    }

    printLCD(13, 3, F("mSecs"));
    app_load_set_set_current_mA(0.0f);
    app_mode_state_set_initialized(true);
    Encoder_Status(false);
  }

  legacy_transcient_cont_timing();
}

void legacy_transient_cont_setup() {
  clearLCD();

  printLCD(3, 0, F("TRANSIENT CONT."));
  printLCD(5, 1, F("I1(A)"));
  printLCD(5, 2, F("I2(A)"));
  printLCD(4, 3, F("dt(mS)"));

  const int col = 11;
  int row = 1;
  if (!Value_Input(col, row)) {
    return;
  }
  LowCurrent = min(app_value_result_get(), app_limits_current_cutoff());
  printLCDNumber(col, row, LowCurrent, 'A', 3);

  row = 2;
  if (!Value_Input(col, row)) {
    return;
  }
  HighCurrent = min(app_value_result_get(), app_limits_current_cutoff());
  printLCDNumber(col, row, HighCurrent, 'A', 3);

  row = 3;
  if (!Value_Input(col, row, 5, false)) {
    return;
  }
  transientPeriod = static_cast<unsigned long>(app_value_result_get());

  clearLCD();
  app_mode_state_set_configured(true);
  app_mode_state_set_initialized(false);
}

void legacy_transcient_cont_timing() {
  static unsigned long last_time = 0;
  static bool transient_cont_toggle = false;

  if (!app_load_is_enabled()) {
    last_time = 0;
    transient_cont_toggle = false;
    return;
  }

  current_time = micros();

  if ((current_time - last_time) >= (transientPeriod * 1000.0)) {
    last_time = current_time;

    if (!transient_cont_toggle) {
      app_load_set_set_current_mA(LowCurrent * 1000.0f);
    } else {
      app_load_set_set_current_mA(HighCurrent * 1000.0f);
    }

    transient_cont_toggle = !transient_cont_toggle;
  }
}

void legacy_transient_list_mode() {
  static unsigned int last_transientPeriod = -1;

  if (!app_mode_state_configured()) {
    legacy_transient_list_setup();
    return;
  }

  if (!app_mode_state_initialized()) {
    printLCD(0, 0, F("TL LOAD"));
    printLCD(6, 2, F("Step: "));
    printLCD(13, 2, F("/"));
    printLCD_S(14, 2, String(total_steps));
    printLCD(4, 3, F("dt: "));
    printLCD(13, 3, F("mS"));
    app_mode_state_set_initialized(true);
    Encoder_Status(false);
  }

  if (app_mode_state_configured()) {
    printLCD_S(12, 2, String(current_step));
    if (transientPeriod != last_transientPeriod) {
      Print_Spaces(8, 3, 5);
      printLCD_S(8, 3, String(transientPeriod));
      last_transientPeriod = transientPeriod;
    }
  }

  legacy_transient_list_timing();
}

void legacy_transient_list_setup() {
  clearLCD();
  printLCD(3, 0, F("TRANSIENT LIST"));
  printLCD(4, 1, F("Steps(2-10)?"));

  float stepsInput = 0.0f;
  do {
    const int col = 9;
    const int row = 2;
    printLCD(col - 1, row, F(">"));
    Print_Spaces(col, row, 2);
    if (!Value_Input(col, row, 2, false)) {
      return;
    }
    stepsInput = app_value_result_get();
  } while (stepsInput < 2.0f || stepsInput > 10.0f);

  total_steps = static_cast<int>(stepsInput) - 1;

  clearLCD();
  for (int i = 0; i <= total_steps; i++) {
    printLCD(3, 0, F("TRANSIENT LIST"));
    printLCD_S(5, 1, "Set step " + String(i));
    printLCD(0, 2, F("Current (A):"));
    printLCD(0, 3, F("Time (mSec):"));

    const int col = 13;
    int row = 2;
    if (!Value_Input(col, row)) {
      return;
    }
    const float currentInput = min(app_value_result_get(), app_limits_current_cutoff());
    printLCDNumber(col, row, currentInput, 'A', 3);
    transientList[i][0] = static_cast<unsigned long>(currentInput * 1000.0f);

    row = 3;
    if (!Value_Input(col, row, 5, false)) {
      return;
    }
    transientList[i][1] = static_cast<unsigned long>(app_value_result_get());
    clearLCD();
  }

  app_load_set_set_current_mA(0.0f);
  current_step = 0;
  transientPeriod = transientList[current_step][1];
  app_mode_state_set_configured(true);
  app_mode_state_set_initialized(false);
}

void legacy_transient_list_timing() {
  static unsigned long last_time = 0;

  if (!app_load_is_enabled()) {
    current_step = 0;
    last_time = 0;
    transientPeriod = transientList[current_step][1];
    return;
  }

  current_time = micros();

  if (last_time == 0) {
    app_load_set_set_current_mA(static_cast<float>(transientList[current_step][0]));
    transientPeriod = transientList[current_step][1];
    last_time = current_time;
  }

  if ((current_time - last_time) >= transientPeriod * 1000) {
    current_step++;
    if (current_step > total_steps) {
      current_step = 0;
    }
    app_load_set_set_current_mA(static_cast<float>(transientList[current_step][0]));
    transientPeriod = transientList[current_step][1];
    last_time = current_time;
  }
}


