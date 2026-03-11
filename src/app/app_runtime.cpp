#include "app_runtime.h"

#include "../config/system_constants.h"
#include "../core/core_engine.h"
#include "../hw/hw_objects.h"
#include "../legacy/legacy_base_io.h"
#include "../legacy/legacy_mode_ca.h"
#include "../legacy/legacy_mode_dispatch.h"
#include "../ui/ui_cycle_render.h"
#include "../ui/ui_mode_templates.h"
#include "app_battery_context.h"
#include "app_calibration_context.h"
#include "app_inputs.h"
#include "app_io_context.h"
#include "app_keypad.h"
#include "app_limits_context.h"
#include "app_load_context.h"
#include "app_load_output.h"
#include "app_loop.h"
#include "app_measurements_context.h"
#include "app_mode_state_context.h"
#include "app_protection.h"
#include "app_runtime_context.h"
#include "app_setpoint_context.h"
#include "app_timing_alerts.h"
#include "app_timer_context.h"
#include "app_transient_context.h"
#include "app_value_result_context.h"

namespace {
void prepare_core_managed_home_mode() {
  if (core_get_state().uiScreen != UiScreen::Home) return;
  if (app_mode_state_initialized()) return;

  switch (app_mode_state_mode()) {
    case CC:
      legacy_encoder_status(true, app_limits_current_cutoff());
      app_mode_state_set_initialized(true);
      break;
    case CP:
      legacy_encoder_status(true, app_limits_power_cutoff());
      app_mode_state_set_initialized(true);
      break;
    case CR:
      legacy_encoder_status(true, MAX_RESISTOR);
      app_runtime_set_encoder_position(MAX_RESISTOR * 1000.0f);
      app_setpoint_set_reading(MAX_RESISTOR);
      app_mode_state_set_initialized(true);
      break;
    case BC:
      if (!app_mode_state_configured()) break;
      app_timer_reset();
      app_battery_life_ref() = 0.0f;
      app_battery_life_previous_ref() = 0.0f;
      legacy_encoder_status(true, app_limits_current_cutoff());
      app_mode_state_set_initialized(true);
      break;
    case CA:
      if (!app_mode_state_configured()) break;
      app_calibration_begin_mode_from_selection(app_value_result_get());
      ui_draw_calibration_mode_template(
          app_calibration_is_voltage_mode(),
          app_calibration_first_point_taken());
      legacy_encoder_status(true, app_limits_current_cutoff());
      app_mode_state_set_initialized(true);
      break;
    default:
      break;
  }
}

void update_core_managed_home_cursor() {
  if (core_get_state().uiScreen != UiScreen::Home) return;

  switch (app_mode_state_mode()) {
    case CC:
    case CP:
    case CR:
    case BC:
    case CA:
      legacy_cursor_position();
      break;
    default:
      break;
  }
}

void run_core_managed_battery_mode() {
  if (core_get_state().uiScreen != UiScreen::Home) return;
  if (app_mode_state_mode() != BC) return;
  if (!app_mode_state_configured()) return;

  float &batteryLife = app_battery_life_ref();
  float &batteryLifePrevious = app_battery_life_previous_ref();
  float &batteryCutoffVolts = app_battery_cutoff_volts_ref();
  float &batteryCurrent = app_battery_current_ref();
  bool &timerRunning = app_timer_running_ref();

  static unsigned long lastUpdate = 0;
  const unsigned long currentMillis = app_io_millis();

  if (app_load_is_enabled() && app_measurements_voltage_v() >= batteryCutoffVolts && !timerRunning) {
    app_timer_start();
  }
  if (!app_load_is_enabled() && app_measurements_voltage_v() >= batteryCutoffVolts && timerRunning) {
    app_timer_stop();
  }

  if (currentMillis - lastUpdate >= 500) {
    lastUpdate = currentMillis;
    ui_update_battery_timer(app_timer_get_time());

    const float loadCurrent = (!timerRunning) ? 0.0f : app_measurements_current_a();
    batteryLife += (loadCurrent * 1000.0f) / 7200.0f;
  }

  if (batteryLife > batteryLifePrevious) {
    ui_update_battery_life(batteryLife);
    batteryLifePrevious = batteryLife;
  }

  float readingValue = app_runtime_encoder_position() / 1000.0f;
  readingValue = min(app_setpoint_max_reading(), max(0.0f, readingValue));
  app_setpoint_set_reading(readingValue);
  app_runtime_set_encoder_position(readingValue * 1000.0f);

  if (!app_load_is_enabled()) {
    return;
  }

  app_load_set_set_current_mA(readingValue * 1000.0f);

  if (app_measurements_voltage_v() <= batteryCutoffVolts) {
    const float nextCurrent = max(app_load_set_current_mA() - CRR_STEP_RDCTN, MIN_DISC_CURR);
    app_load_set_set_current_mA(nextCurrent);
    readingValue = app_load_set_current_mA() / 1000.0f;
    app_setpoint_set_reading(readingValue);
    app_runtime_set_encoder_position(readingValue * 1000.0f);
  }

  if (app_measurements_voltage_v() <= (batteryCutoffVolts - VLTG_DROP_MARGIN)) {
    batteryCurrent = app_measurements_current_a();
    app_setpoint_set_reading(0.0f);
    app_runtime_set_encoder_position(0.0f);
    app_load_set_set_current_mA(0.0f);
    encoder.clearCount();
    app_load_output_off();
    app_timer_stop();
    app_beep_buzzer();
    ui_show_battery_done();
  }
}

void run_core_managed_calibration_mode() {
  static bool confirmationDrawn = false;

  if (core_get_state().uiScreen != UiScreen::Home) return;
  if (app_mode_state_mode() != CA) return;

  if (!app_mode_state_configured()) {
    confirmationDrawn = false;
    legacy_calibration_setup();
    return;
  }

  if (app_calibration_confirmation_active()) {
    if (!confirmationDrawn) {
      ui_draw_calibration_result(
          app_calibration_pending_is_voltage_mode(),
          app_calibration_pending_sensor_factor(),
          app_calibration_pending_sensor_offset(),
          app_calibration_pending_output_factor(),
          app_calibration_pending_output_offset());
      confirmationDrawn = true;
    }
    return;
  }

  confirmationDrawn = false;

  float readingValue = app_runtime_encoder_position() / 1000.0f;
  readingValue = min(app_setpoint_max_reading(), max(0.0f, readingValue));
  app_setpoint_set_reading(readingValue);
  app_runtime_set_encoder_position(readingValue * 1000.0f);

  if (!app_load_is_enabled()) {
    return;
  }

  app_load_set_set_current_mA(readingValue * 1000.0f);
}

void run_core_managed_transient_cont_mode() {
  if (core_get_state().uiScreen != UiScreen::Home) return;
  if (app_mode_state_mode() != TC) return;
  if (!app_mode_state_configured()) return;

  float &lowCurrent = app_transient_low_current_ref();
  float &highCurrent = app_transient_high_current_ref();
  unsigned long &transientPeriod = app_transient_period_ref();
  unsigned long &currentTime = app_transient_current_time_ref();

  static unsigned long lastTime = 0;
  static bool transientContToggle = false;

  if (!app_mode_state_initialized()) {
    ui_draw_transient_cont_mode_template(lowCurrent, highCurrent, transientPeriod);
    app_load_set_set_current_mA(0.0f);
    app_mode_state_set_initialized(true);
    legacy_encoder_status(false);
  }

  if (!app_load_is_enabled()) {
    lastTime = 0;
    transientContToggle = false;
    return;
  }

  currentTime = micros();

  if ((currentTime - lastTime) >= (transientPeriod * 1000.0f)) {
    lastTime = currentTime;

    if (!transientContToggle) {
      app_load_set_set_current_mA(lowCurrent * 1000.0f);
    } else {
      app_load_set_set_current_mA(highCurrent * 1000.0f);
    }

    transientContToggle = !transientContToggle;
  }
}

void run_core_managed_transient_list_mode() {
  if (core_get_state().uiScreen != UiScreen::Home) return;
  if (app_mode_state_mode() != TL) return;
  if (!app_mode_state_configured()) return;

  unsigned long &transientPeriod = app_transient_period_ref();
  unsigned long &currentTime = app_transient_current_time_ref();
  unsigned long (*transientList)[2] = app_transient_list_ref();
  int &totalSteps = app_transient_total_steps_ref();
  int &currentStep = app_transient_current_step_ref();

  static unsigned long lastTime = 0;
  static unsigned int lastTransientPeriod = static_cast<unsigned int>(-1);

  if (!app_mode_state_initialized()) {
    ui_draw_transient_list_mode_template(totalSteps);
    app_mode_state_set_initialized(true);
    legacy_encoder_status(false);
  }

  ui_update_transient_list_step(currentStep);
  if (transientPeriod != lastTransientPeriod) {
    ui_update_transient_list_period(transientPeriod);
    lastTransientPeriod = transientPeriod;
  }

  if (!app_load_is_enabled()) {
    currentStep = 0;
    lastTime = 0;
    transientPeriod = transientList[currentStep][1];
    return;
  }

  currentTime = micros();

  if (lastTime == 0) {
    app_load_set_set_current_mA(static_cast<float>(transientList[currentStep][0]));
    transientPeriod = transientList[currentStep][1];
    lastTime = currentTime;
    return;
  }

  if ((currentTime - lastTime) >= transientPeriod * 1000UL) {
    currentStep++;
    if (currentStep > totalSteps) {
      currentStep = 0;
    }
    app_load_set_set_current_mA(static_cast<float>(transientList[currentStep][0]));
    transientPeriod = transientList[currentStep][1];
    lastTime = currentTime;
  }
}
}

void app_run_cycle() {
  app_update_fan_control();
  legacy_read_encoder();

  if (core_get_state().uiScreen != UiScreen::Home) {
    app_read_encoder_button();
  }

  app_read_keypad(1, 3);
  app_read_load_button();
  legacy_read_volts_current();
  app_check_limits();
  app_load_output_apply();

  if (core_get_state().uiScreen == UiScreen::Home) {
    prepare_core_managed_home_mode();
    update_core_managed_home_cursor();
    run_core_managed_battery_mode();
    run_core_managed_calibration_mode();
    run_core_managed_transient_cont_mode();
    run_core_managed_transient_list_mode();
    legacy_run_mode_logic();
  }

  app_tick();
  ui_render_cycle();
}

