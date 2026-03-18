#include "app_runtime.h"

#include "../core/core_engine.h"
#include "../ui/ui_cycle_render.h"
#include "app_inputs.h"
#include "app_keypad.h"
#include "app_load_output.h"
#include "app_loop.h"
#include "app_measurements_context.h"
#include "app_measurements_poll.h"
#include "app_protection.h"
#include "app_runtime_home.h"
#include "app_timing_alerts.h"

void app_run_cycle() {
  app_update_fan_control();
  app_read_encoder();

  if (core_get_state().uiScreen != UiScreen::Home) {
    app_read_encoder_button();
  }

  app_read_keypad(1, 3);
  app_read_load_button();
  app_measurements_poll();
  app_check_limits();
  app_load_output_apply();

  if (core_get_state().uiScreen == UiScreen::Home) {
    app_runtime_prepare_home_mode();
    app_runtime_update_home_cursor();
    app_runtime_run_home_mode_tasks();
  }

  app_tick();
  ui_render_cycle();
}
