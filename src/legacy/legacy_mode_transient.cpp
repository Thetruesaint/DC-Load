#include "legacy_mode_transient.h"

#include "../config/system_constants.h"
#include "../hw/hw_objects.h"
#include "../ui_lcd.h"
#include "../funciones.h"
#include "../ui/ui_mode_templates.h"
#include "../app/app_mode_state_context.h"
#include "../app/app_limits_context.h"
#include "../app/app_load_context.h"
#include "../app/app_value_result_context.h"
#include "../app/app_transient_context.h"

#define LowCurrent (app_transient_low_current_ref())
#define HighCurrent (app_transient_high_current_ref())
#define transientPeriod (app_transient_period_ref())
#define current_time (app_transient_current_time_ref())
#define transientList (app_transient_list_ref())
#define total_steps (app_transient_total_steps_ref())
#define current_step (app_transient_current_step_ref())

void legacy_transient_cont_mode() {
  if (!app_mode_state_configured()) {
    legacy_transient_cont_setup();
    return;
  }

  if (!app_mode_state_initialized()) {
    ui_draw_transient_cont_mode_template(LowCurrent, HighCurrent, transientPeriod);
    app_load_set_set_current_mA(0.0f);
    app_mode_state_set_initialized(true);
    Encoder_Status(false);
  }

  legacy_transcient_cont_timing();
}

void legacy_transient_cont_setup() {
  ui_draw_transient_cont_setup_template();

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
    ui_draw_transient_list_mode_template(total_steps);
    app_mode_state_set_initialized(true);
    Encoder_Status(false);
  }

  if (app_mode_state_configured()) {
    ui_update_transient_list_step(current_step);
    if (transientPeriod != last_transientPeriod) {
      ui_update_transient_list_period(transientPeriod);
      last_transientPeriod = transientPeriod;
    }
  }

  legacy_transient_list_timing();
}

void legacy_transient_list_setup() {
  ui_draw_transient_list_setup_template();

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
    ui_draw_transient_list_step_template(i);

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

