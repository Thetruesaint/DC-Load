#include "app_mode_state_context.h"

namespace {
uint8_t modeId = 0;
int functionIndex = 0;
bool modeInitialized = false;
bool modeConfigured = false;
}

uint8_t app_mode_state_mode() {
  return modeId;
}

void app_mode_state_set_mode(uint8_t mode) {
  modeId = mode;
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

