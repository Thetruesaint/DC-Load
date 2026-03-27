#include "app_io_context.h"

#include "../hal/hal_inputs.h"

bool app_io_encoder_button_low() {
  return hal_encoder_button_low();
}

uint32_t app_io_millis() {
  return hal_millis();
}
