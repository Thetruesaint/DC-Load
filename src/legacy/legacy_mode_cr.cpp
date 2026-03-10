#include "legacy_mode_cr.h"

#include "../app/app_mode_state_context.h"
#include "../app/app_runtime_context.h"
#include "../app/app_setpoint_context.h"
#include "../config/system_constants.h"
#include "../funciones.h"
#include "../ui/ui_mode_templates.h"

void legacy_const_resistance_mode() {
  if (!app_mode_state_initialized()) {
    Encoder_Status(true, MAX_RESISTOR);
    app_runtime_set_encoder_position(MAX_RESISTOR * 1000.0f);
    app_mode_state_set_initialized(true);
  }

  float readingValue = app_runtime_encoder_position() / 1000.0f;
  readingValue = min(app_setpoint_max_reading(), max(0.1f, readingValue));
  app_setpoint_set_reading(readingValue);
  app_runtime_set_encoder_position(readingValue * 1000.0f);
  Cursor_Position();

  // Setpoint de CR lo calcula core y se aplica via legacy_apply_state.
}

