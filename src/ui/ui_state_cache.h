#ifndef UI_STATE_CACHE_H
#define UI_STATE_CACHE_H

#include "../core/core_state.h"

void ui_state_cache_set(const SystemState &state);
const SystemState &ui_state_cache_get();

#endif
