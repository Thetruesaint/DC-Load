#include "app_loop.h"

#include "../core/core_actions.h"
#include "../core/core_engine.h"
#include "../legacy/legacy_bridge.h"
#include "../ui/ui_renderer.h"

void app_init() {
  core_init();
  legacy_reset_action_poll();
}

void app_tick() {
  core_sync_from_legacy(legacy_capture_state());

  UserAction actions[6] = {};
  const size_t actionCount = legacy_poll_actions(actions, 6);
  for (size_t i = 0; i < actionCount; ++i) {
    core_dispatch(actions[i]);
  }

  core_tick_10ms();
  ui_render(core_get_state());
}