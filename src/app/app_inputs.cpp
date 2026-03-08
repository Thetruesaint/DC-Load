#include "app_inputs.h"

#include "../variables.h"
#include "app_loop.h"

void app_read_encoder() {
  static int32_t lastCount = 0;
  int32_t newCount = encoder.getCount();
  int32_t diff = newCount - lastCount;

  if (abs(diff) >= 4) {
    lastCount = newCount;
    app_push_action(ActionType::EncoderDelta, diff, '\0');
  }
}

void app_read_load_button() {
  static bool lastButtonLow = false;
  const bool buttonLow = (digitalRead(LOADONOFF) == LOW);

  if (buttonLow && !lastButtonLow) {
    delay(40);
    if (digitalRead(LOADONOFF) == LOW) {
      app_push_action(ActionType::LoadToggle, 0, '\0');
      lastButtonLow = true;
      return;
    }
  }

  if (!buttonLow) {
    lastButtonLow = false;
  }
}
