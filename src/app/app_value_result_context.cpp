#include "app_value_result_context.h"

namespace {
float valueResult = 0.0f;
}

float app_value_result_get() {
  return valueResult;
}

void app_value_result_set(float value) {
  valueResult = value;
}
