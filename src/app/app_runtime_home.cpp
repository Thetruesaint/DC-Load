#include "app_runtime_home.h"

#include <Arduino.h>

#include "../config/system_constants.h"
#include "../core/core_engine.h"
#include "../hw/hw_objects.h"
#include "../ui_display.h"
#include "app_battery_context.h"
#include "app_calibration_context.h"
#include "app_calibration_flow.h"
#include "app_encoder_setup.h"
#include "app_io_context.h"
#include "app_limits_context.h"
#include "app_load_context.h"
#include "app_load_output.h"
#include "app_measurements_context.h"
#include "app_mode_state_context.h"
#include "app_runtime_context.h"
#include "app_setpoint_context.h"
#include "app_setpoint_cursor.h"
#include "app_timer_context.h"
#include "app_timing_alerts.h"
#include "app_transient_context.h"
#include "app_value_result_context.h"

namespace {
constexpr unsigned long kTransientContMaxPeriodMs = 10000UL;

bool home_screen_active() {
  return core_get_state().uiScreen == UiScreen::Home;
}

void prepare_calibration_mode() {
  if (!app_mode_state_configured()) return;
  app_calibration_begin_mode_from_selection(app_value_result_get());
  uiDisplayInvalidateHomeLayout();
  app_encoder_setup_begin(app_limits_current_cutoff());
  app_mode_state_set_initialized(true);
}

void prepare_transient_cont_mode() {
  if (!app_mode_state_configured()) return;
  app_runtime_set_cursor_position(8);
  app_runtime_set_encoder_step(10000.0f);
  app_runtime_set_encoder_position(static_cast<float>(constrain(app_transient_period_ref(), 100UL, kTransientContMaxPeriodMs)));
  app_runtime_set_encoder_max(kTransientContMaxPeriodMs);
  app_load_set_set_current_mA(0.0f);
  app_mode_state_set_initialized(true);
  app_encoder_setup_reset();
}

void prepare_battery_mode() {
  if (!app_mode_state_configured()) return;
  app_timer_reset();
  app_battery_life_ref() = 0.0f;
  app_battery_life_previous_ref() = 0.0f;
  app_battery_done_ref() = false;
  app_encoder_setup_begin(app_limits_current_cutoff());
  app_mode_state_set_initialized(true);
}

void run_battery_mode() {
  if (!home_screen_active()) return;
  if (app_mode_state_mode() != BC) return;
  if (!app_mode_state_configured()) return;

  float &batteryLife = app_battery_life_ref();
  float &batteryLifePrevious = app_battery_life_previous_ref();
  float &batteryCutoffVolts = app_battery_cutoff_volts_ref();
  float &batteryCurrent = app_battery_current_ref();
  bool &batteryDone = app_battery_done_ref();
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
    const float loadCurrent = (!timerRunning) ? 0.0f : app_measurements_current_a();
    batteryLife += (loadCurrent * 1000.0f) / 7200.0f;
  }

  if (batteryLife > batteryLifePrevious) {
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
    batteryDone = true;
  }
}

void run_calibration_mode() {
  if (!home_screen_active()) return;
  if (app_mode_state_mode() != CA) return;
  app_calibration_mode_update();
}

void run_transient_cont_mode() {
  if (!home_screen_active()) return;
  if (app_mode_state_mode() != TC) return;
  if (!app_mode_state_configured()) return;

  float &lowCurrent = app_transient_low_current_ref();
  float &highCurrent = app_transient_high_current_ref();
  unsigned long &transientPeriod = app_transient_period_ref();
  unsigned long &currentTime = app_transient_current_time_ref();

  static unsigned long lastTime = 0;
  static bool transientContToggle = false;

  if (!app_mode_state_initialized()) {
    app_load_set_set_current_mA(0.0f);
    app_mode_state_set_initialized(true);
    app_encoder_setup_reset();
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

void run_transient_list_mode() {
  if (!home_screen_active()) return;
  if (app_mode_state_mode() != TL) return;
  if (!app_mode_state_configured()) return;

  unsigned long &transientPeriod = app_transient_period_ref();
  unsigned long &currentTime = app_transient_current_time_ref();
  unsigned long (*transientList)[2] = app_transient_list_ref();
  int &totalSteps = app_transient_total_steps_ref();
  int &currentStep = app_transient_current_step_ref();

  static unsigned long lastTime = 0;

  if (!app_mode_state_initialized()) {
    app_mode_state_set_initialized(true);
    app_encoder_setup_reset();
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

void app_runtime_prepare_home_mode() {
  if (!home_screen_active()) return;
  if (app_mode_state_initialized()) return;

  switch (app_mode_state_mode()) {
    case CC:
      app_encoder_setup_begin(app_limits_current_cutoff());
      app_mode_state_set_initialized(true);
      break;
    case CP:
      app_encoder_setup_begin(app_limits_power_cutoff());
      app_mode_state_set_initialized(true);
      break;
    case CR:
      app_encoder_setup_begin(MAX_RESISTOR);
      app_runtime_set_encoder_position(MAX_RESISTOR * 1000.0f);
      app_setpoint_set_reading(MAX_RESISTOR);
      app_mode_state_set_initialized(true);
      break;
    case BC:
      prepare_battery_mode();
      break;
    case CA:
      prepare_calibration_mode();
      break;
    case TC:
      prepare_transient_cont_mode();
      break;
    default:
      break;
  }
}

void app_runtime_update_home_cursor() {
  if (!home_screen_active()) return;

  switch (app_mode_state_mode()) {
    case CC:
    case CP:
    case CR:
    case BC:
    case CA:
    case TC:
      app_setpoint_cursor_update();
      break;
    default:
      break;
  }
}

void app_runtime_run_home_mode_tasks() {
  if (!home_screen_active()) return;

  run_battery_mode();
  run_calibration_mode();
  run_transient_cont_mode();
  run_transient_list_mode();
}
