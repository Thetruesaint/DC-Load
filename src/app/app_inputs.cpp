#include "app_inputs.h"

#include <Arduino.h>

#include "app_io_context.h"
#include "app_loop.h"

void app_read_encoder() {
  static int32_t lastCount = 0;
  int32_t newCount = app_io_encoder_count();
  int32_t diff = newCount - lastCount;

  if (abs(diff) >= 4) {
    lastCount = newCount;
    app_push_action(ActionType::EncoderDelta, diff, '\0');
  }
}

void app_read_load_button() {
  static bool lastButtonLow = false;
  const bool buttonLow = app_io_load_button_low();

  if (buttonLow && !lastButtonLow) {
    delay(40);
    if (app_io_load_button_low()) {
      app_push_action(ActionType::LoadToggle, 0, '\0');
      lastButtonLow = true;
      return;
    }
  }

  if (!buttonLow) {
    lastButtonLow = false;
  }
}
