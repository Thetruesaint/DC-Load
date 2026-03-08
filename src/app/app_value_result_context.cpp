#include "app_value_result_context.h"

#include "../variables.h"

float app_value_result_get() {
  return x;
}

void app_value_result_set(float value) {
  x = value;
}
