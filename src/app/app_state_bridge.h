#ifndef APP_STATE_BRIDGE_H
#define APP_STATE_BRIDGE_H

#include "../core/core_state.h"

SystemState app_state_bridge_capture();
void app_state_bridge_apply(const SystemState &state);

#endif
