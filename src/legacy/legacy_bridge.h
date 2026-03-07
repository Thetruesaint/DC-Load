#ifndef LEGACY_BRIDGE_H
#define LEGACY_BRIDGE_H

#include "../core/core_state.h"

SystemState legacy_capture_state();
void legacy_apply_state(const SystemState &state);

#endif