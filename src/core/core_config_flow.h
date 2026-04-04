#ifndef CORE_CONFIG_FLOW_H
#define CORE_CONFIG_FLOW_H

#include "core_state.h"

bool core_config_wants_calibration(const SystemState &state);
UiScreen core_config_target_screen(const SystemState &state);

#endif
