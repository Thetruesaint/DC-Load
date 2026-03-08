#include "app_io_context.h"

#include "../variables.h"

int32_t app_io_encoder_count() {
  return encoder.getCount();
}

bool app_io_load_button_low() {
  return (digitalRead(LOADONOFF) == LOW);
}
