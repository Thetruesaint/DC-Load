#ifndef LEGACY_BRIDGE_H
#define LEGACY_BRIDGE_H

#include <stddef.h>

#include "../core/core_actions.h"
#include "../core/core_state.h"

SystemState legacy_capture_state();
void legacy_reset_action_poll();
size_t legacy_poll_actions(UserAction *actions, size_t maxActions);

#endif