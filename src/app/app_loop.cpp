#include "app_loop.h"

#include "../core/core_engine.h"
#include "../legacy/legacy_bridge.h"
#include "../ui/ui_renderer.h"

void app_init() {
  core_init();
}

void app_tick() {
  core_sync_from_legacy(legacy_capture_state());
  core_tick_10ms();
  ui_render(core_get_state());
}