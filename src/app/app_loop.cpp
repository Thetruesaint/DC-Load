#include "app_loop.h"

#include <cstddef>

#include "../core/core_engine.h"
#include "../ui/ui_renderer.h"
#include "app_ota.h"
#include "app_startup.h"
#include "app_runtime_sync.h"

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

bool drain_action_queue() {
  UserAction action = make_none_action();
  bool dispatched = false;

  while (dequeueAction(&action)) {
    core_dispatch(action);
    dispatched = true;
  }

  return dispatched;
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
  core_sync_from_runtime(app_runtime_sync_capture());
  core_begin_cycle();

  if (app_startup_consume_limits_setup_request()) {
    app_push_action(make_open_limits_setup_action());
  }

  (void)drain_action_queue();

  core_tick_10ms();
  app_runtime_sync_apply(core_get_state());

  if (core_get_state().uiScreen == UiScreen::MenuFwUpdate) {
    app_ota_handle();
  }

  core_sync_from_runtime(app_runtime_sync_capture());

  core_begin_cycle();
  if (drain_action_queue()) {
    core_tick_10ms();
    app_runtime_sync_apply(core_get_state());
    if (core_get_state().uiScreen == UiScreen::MenuFwUpdate) {
      app_ota_handle();
    }
  }

  ui_render(core_get_state());
}
