#include "ui_renderer.h"

#include "ui_state_cache.h"

namespace {
UiViewState make_ui_view_state(const SystemState &state) {
  UiViewState view = ui_view_state_make_default();
  view.modeInitialized = state.modeInitialized;
  view.loadEnabled = state.loadEnabled;
  view.mode = state.mode;
  view.cursorPosition = state.cursorPosition;
  view.measuredCurrent_A = state.measuredCurrent_A;
  view.measuredVoltage_V = state.measuredVoltage_V;
  view.measuredPower_W = state.measuredPower_W;
  view.readingValue = state.readingValue;
  return view;
}
}

void ui_render(const SystemState &state) {
  ui_state_cache_set(make_ui_view_state(state));
}
