#include "app_load_context.h"

namespace {
bool loadEnabled = false;
float setCurrent_mA = 0.0f;
}

bool app_load_is_enabled() {
  return loadEnabled;
}

void app_load_set_enabled(bool enabled) {
  loadEnabled = enabled;
}

float app_load_set_current_mA() {
  return setCurrent_mA;
}

void app_load_set_set_current_mA(float value) {
  setCurrent_mA = value;
}
