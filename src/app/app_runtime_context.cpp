#include "app_runtime_context.h"

namespace {
float encoderPosition = 0.0f;
float factor = 0.0f;
unsigned long maxEncoder = 999000;
int cursorPosition = 8;
}

float app_runtime_encoder_position() {
  return encoderPosition;
}

void app_runtime_set_encoder_position(float value) {
  encoderPosition = value;
}

float app_runtime_encoder_step() {
  return factor;
}

void app_runtime_set_encoder_step(float value) {
  factor = value;
}

unsigned long app_runtime_encoder_max() {
  return maxEncoder;
}

void app_runtime_set_encoder_max(unsigned long value) {
  maxEncoder = value;
}

int app_runtime_cursor_position() {
  return cursorPosition;
}

void app_runtime_set_cursor_position(int position) {
  cursorPosition = position;
}
