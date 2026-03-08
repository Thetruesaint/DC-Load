#include "app_load_context.h"

#include "../variables.h"

bool app_load_is_enabled() {
  return toggle;
}

void app_load_set_enabled(bool enabled) {
  toggle = enabled;
}

float app_load_set_current_mA() {
  return setCurrent;
}

void app_load_set_set_current_mA(float value) {
  setCurrent = value;
}
