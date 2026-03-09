#include "ui_state_cache.h"

namespace {
SystemState g_uiState = core_state_make_default();
}

void ui_state_cache_set(const SystemState &state) {
  g_uiState = state;
}

const SystemState &ui_state_cache_get() {
  return g_uiState;
}
