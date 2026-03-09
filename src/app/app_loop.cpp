#include "app_loop.h"
#include <cstddef>

#include "../core/core_engine.h"
#include "../legacy/legacy_bridge.h"
#include "../ui/ui_renderer.h"

namespace {
constexpr size_t ACTION_QUEUE_CAPACITY = 16;
UserAction g_actionQueue[ACTION_QUEUE_CAPACITY] = {};
size_t g_queueHead = 0;
size_t g_queueTail = 0;

bool enqueueAction(const UserAction &action) {
  const size_t nextTail = (g_queueTail + 1U) % ACTION_QUEUE_CAPACITY;
  if (nextTail == g_queueHead) {
    return false;
  }

  g_actionQueue[g_queueTail] = action;
  g_queueTail = nextTail;
  return true;
}

bool dequeueAction(UserAction *action) {
  if (action == nullptr || g_queueHead == g_queueTail) {
    return false;
  }

  *action = g_actionQueue[g_queueHead];
  g_queueHead = (g_queueHead + 1U) % ACTION_QUEUE_CAPACITY;
  return true;
}
}

void app_init() {
  core_init();
  g_queueHead = 0;
  g_queueTail = 0;
}

void app_push_action(const UserAction &action) {
  (void)enqueueAction(action);
}

void app_tick() {
  core_sync_from_legacy(legacy_capture_state());
  core_begin_cycle();

  UserAction action = make_none_action();
  while (dequeueAction(&action)) {
    core_dispatch(action);
  }

  core_tick_10ms();
  legacy_apply_state(core_get_state());
  ui_render(core_get_state());
}
