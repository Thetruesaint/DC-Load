#include "hal_inputs.h"

#include "../config/system_constants.h"
#include "../hw/hw_objects.h"

int32_t hal_encoder_count() {
  return encoder.getCount();
}

bool hal_load_button_low() {
  return (digitalRead(LOADONOFF) == LOW);
}
