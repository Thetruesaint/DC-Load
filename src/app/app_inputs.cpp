#include "app_inputs.h"

#include <Arduino.h>

#include "../core/core_engine.h"
#include "../hal/hal_inputs.h"
#include "app_loop.h"

namespace {
int32_t g_lastEncoderCount = 0;
}

void app_read_encoder() {
  int32_t newCount = hal_encoder_count();
  int32_t diff = newCount - g_lastEncoderCount;

  if (abs(diff) >= 4) {
    g_lastEncoderCount = newCount;
    app_push_action(make_encoder_delta_action(diff));
  }
}

void app_reset_encoder_tracking() {
  g_lastEncoderCount = hal_encoder_count();
}

void app_read_encoder_button() {
  static bool lastButtonLow = false;
  const bool buttonLow = hal_encoder_button_low();

  if (buttonLow && !lastButtonLow) {
    hal_delay_ms(40);
    if (hal_encoder_button_low()) {
      app_push_action(make_encoder_button_press_action());
      lastButtonLow = true;
      return;
    }
  }

  if (!buttonLow) {
    lastButtonLow = false;
  }
}

void app_read_load_button() {
  static bool lastButtonLow = false;
  const bool buttonLow = hal_load_button_low();

  if (buttonLow && !lastButtonLow) {
    hal_delay_ms(40);
    if (hal_load_button_low()) {
      app_push_action(make_load_toggle_action());
      lastButtonLow = true;
      return;
    }
  }

  if (!buttonLow) {
    lastButtonLow = false;
  }
}
