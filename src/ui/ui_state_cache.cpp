#include "ui_state_cache.h"

namespace {
UiViewState g_uiState = ui_view_state_make_default();
}

void ui_state_cache_set(const UiViewState &state) {
  g_uiState = state;
}

const UiViewState &ui_state_cache_get() {
  return g_uiState;
}
