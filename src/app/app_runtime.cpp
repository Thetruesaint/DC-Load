#include "app_runtime.h"

#include "../legacy/legacy_base_io.h"
#include "../legacy/legacy_dac_control.h"
#include "../legacy/legacy_mode_dispatch.h"
#include "../legacy/legacy_safety_control.h"
#include "../ui/ui_cycle_render.h"
#include "app_inputs.h"
#include "app_keypad.h"
#include "app_loop.h"

void app_run_cycle() {
  legacy_temp_control();
  legacy_read_encoder();
  app_read_keypad(1, 3);
  app_read_load_button();
  legacy_read_volts_current();
  legacy_check_limits();
  legacy_dac_control();

  legacy_run_mode_logic();
  app_tick();
  ui_render_cycle();
}
