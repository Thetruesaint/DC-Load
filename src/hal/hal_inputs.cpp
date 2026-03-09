#include "hal_inputs.h"

#include "../config/system_constants.h"
#include "../hw/hw_objects.h"

int32_t hal_encoder_count() {
  return encoder.getCount();
}

bool hal_load_button_low() {
  return (digitalRead(LOADONOFF) == LOW);
}

bool hal_encoder_button_low() {
  return (digitalRead(ENC_BTN) == LOW);
}

uint32_t hal_millis() {
  return millis();
}

void hal_delay_ms(uint32_t ms) {
  delay(ms);
}
