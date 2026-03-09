#include "app_mode_state_context.h"

#include "../variables.h"

uint8_t app_mode_state_mode() {
  return static_cast<uint8_t>(Mode);
}

void app_mode_state_set_mode(uint8_t mode) {
  Mode = static_cast<ModeType>(mode);
}

int app_mode_state_function_index() {
  return functionIndex;
}

void app_mode_state_set_function_index(int index) {
  functionIndex = index;
}

bool app_mode_state_initialized() {
  return modeInitialized;
}

void app_mode_state_set_initialized(bool initialized) {
  modeInitialized = initialized;
}

bool app_mode_state_configured() {
  return modeConfigured;
}

void app_mode_state_set_configured(bool configured) {
  modeConfigured = configured;
}
