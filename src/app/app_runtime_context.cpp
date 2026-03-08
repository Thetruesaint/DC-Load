#include "app_runtime_context.h"

#include "../variables.h"

float app_runtime_encoder_position() {
  return encoderPosition;
}

void app_runtime_set_encoder_position(float value) {
  encoderPosition = value;
}

float app_runtime_encoder_step() {
  return factor;
}

unsigned long app_runtime_encoder_max() {
  return maxEncoder;
}

int app_runtime_cursor_position() {
  return CuPo;
}

void app_runtime_set_cursor_position(int position) {
  CuPo = position;
}
