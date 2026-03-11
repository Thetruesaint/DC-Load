#include "app_encoder_setup.h"

#include "../hw/hw_objects.h"
#include "app_inputs.h"
#include "app_runtime_context.h"
#include "app_setpoint_context.h"

void app_encoder_setup_begin(float limit) {
  app_runtime_set_cursor_position(8);
  app_setpoint_set_reading(0);
  app_runtime_set_encoder_position(0);
  app_setpoint_set_max_reading(limit);
  app_runtime_set_encoder_max(static_cast<unsigned long>(app_setpoint_max_reading() * 1000));

  encoder.clearCount();
  app_reset_encoder_tracking();
}

void app_encoder_setup_reset() {
  encoder.clearCount();
  app_reset_encoder_tracking();
}
