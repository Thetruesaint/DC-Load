#include "app_runtime.h"

#include "../config/system_constants.h"
#include "../core/core_engine.h"
#include "../funciones.h"
#include "../hw/hw_objects.h"
#include "../legacy/legacy_base_io.h"
#include "../legacy/legacy_dac_control.h"
#include "../legacy/legacy_mode_dispatch.h"
#include "../legacy/legacy_safety_control.h"
#include "../ui/ui_cycle_render.h"
#include "../ui/ui_mode_templates.h"
#include "app_battery_context.h"
#include "app_inputs.h"
#include "app_io_context.h"
#include "app_keypad.h"
#include "app_limits_context.h"
#include "app_load_context.h"
#include "app_loop.h"
#include "app_measurements_context.h"
#include "app_mode_state_context.h"
#include "app_runtime_context.h"
#include "app_setpoint_context.h"
#include "app_timer_context.h"

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
      timer_reset();
      app_battery_life_ref() = 0.0f;
      app_battery_life_previous_ref() = 0.0f;
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
    timer_start();
  }
  if (!app_load_is_enabled() && app_measurements_voltage_v() >= batteryCutoffVolts && timerRunning) {
    timer_stop();
  }

  if (currentMillis - lastUpdate >= 500) {
    lastUpdate = currentMillis;
    ui_update_battery_timer(timer_getTime());

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
    Load_OFF();
    timer_stop();
    beepBuzzer();
    ui_show_battery_done();
  }
}
}

void app_run_cycle() {
  legacy_temp_control();
  legacy_read_encoder();

  if (core_get_state().uiScreen != UiScreen::Home) {
    app_read_encoder_button();
  }

  app_read_keypad(1, 3);
  app_read_load_button();
  legacy_read_volts_current();
  legacy_check_limits();
  legacy_dac_control();

  if (core_get_state().uiScreen == UiScreen::Home) {
    prepare_core_managed_home_mode();
    update_core_managed_home_cursor();
    run_core_managed_battery_mode();
    legacy_run_mode_logic();
  }

  app_tick();
  ui_render_cycle();
}
