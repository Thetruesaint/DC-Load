#include "ui_renderer.h"

#include "ui_state_cache.h"

void ui_render(const SystemState &state) {
  ui_state_cache_set(state);
}
