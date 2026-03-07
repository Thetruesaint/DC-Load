#include "core_engine.h"

#include <Arduino.h>

namespace {
SystemState g_state = {0};
unsigned long g_lastTickMs = 0;
}

void core_init() {
  g_lastTickMs = millis();
}

void core_sync_from_legacy(const SystemState &state) {
  g_state = state;
}

void core_tick_10ms() {
  const unsigned long now = millis();
  if ((now - g_lastTickMs) < 10UL) return;
  g_lastTickMs = now;
}

const SystemState &core_get_state() {
  return g_state;
}