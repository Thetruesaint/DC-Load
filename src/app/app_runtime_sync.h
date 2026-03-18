#ifndef APP_RUNTIME_SYNC_H
#define APP_RUNTIME_SYNC_H

#include "../core/core_state.h"
#include "../core/core_sync_bridge.h"

RuntimeSnapshot app_runtime_sync_capture();
void app_runtime_sync_apply(const SystemState &state);

#endif


