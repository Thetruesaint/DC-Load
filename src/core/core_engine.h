#ifndef CORE_ENGINE_H
#define CORE_ENGINE_H

#include "core_actions.h"
#include "core_state.h"

void core_init();
void core_sync_from_legacy(const SystemState &state);
void core_dispatch(const UserAction &action);
void core_tick_10ms();
const SystemState &core_get_state();

#endif