#include "app_io_context.h"

#include "../hal/hal_inputs.h"

int32_t app_io_encoder_count() {
  return hal_encoder_count();
}

bool app_io_load_button_low() {
  return hal_load_button_low();
}

bool app_io_encoder_button_low() {
  return hal_encoder_button_low();
}

uint32_t app_io_millis() {
  return hal_millis();
}
