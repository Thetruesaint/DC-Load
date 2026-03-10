#include "app_runtime.h"

#include "../config/system_constants.h"
#include "../core/core_engine.h"
#include "../legacy/legacy_base_io.h"
#include "../legacy/legacy_dac_control.h"
#include "../legacy/legacy_mode_dispatch.h"
#include "../legacy/legacy_safety_control.h"
#include "../ui/ui_cycle_render.h"
#include "app_inputs.h"
#include "app_keypad.h"
#include "app_limits_context.h"
#include "app_loop.h"
#include "app_mode_state_context.h"
#include "app_runtime_context.h"
#include "app_setpoint_context.h"

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
    default:
      break;
  }
}
}

void app_run_cycle() {
  legacy_temp_control();
  legacy_read_encoder();

  // Menus use the new state machine and do not pass through legacy cursor handling,
  // so encoder button presses must be dispatched from app-level input polling here.
  if (core_get_state().uiScreen != UiScreen::Home) {
    app_read_encoder_button();
  }

  app_read_keypad(1, 3);
  app_read_load_button();
  legacy_read_volts_current();
  legacy_check_limits();
  legacy_dac_control();

  // While in non-home UI screens, skip legacy mode rendering logic.
  // This prevents background templates from overwriting menu/config screens.
  if (core_get_state().uiScreen == UiScreen::Home) {
    prepare_core_managed_home_mode();
    legacy_run_mode_logic();
  }

  app_tick();
  ui_render_cycle();
}
