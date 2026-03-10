#include "app_fan_context.h"

#include "../config/system_constants.h"

namespace {
int fanTempOnC = DEFAULT_FAN_TEMP_ON_C;
unsigned long fanHoldMs = DEFAULT_FAN_HOLD_MS;
}

int app_fan_temp_on_c() {
  return fanTempOnC;
}

void app_fan_set_temp_on_c(int value) {
  fanTempOnC = value;
}

unsigned long app_fan_hold_ms() {
  return fanHoldMs;
}

void app_fan_set_hold_ms(unsigned long value) {
  fanHoldMs = value;
}

uint8_t app_fan_hold_seconds() {
  return static_cast<uint8_t>(fanHoldMs / 1000UL);
}

void app_fan_set_hold_seconds(uint8_t seconds) {
  app_fan_set_hold_ms(static_cast<unsigned long>(seconds) * 1000UL);
}

