#ifndef CORE_STATE_RULES_H
#define CORE_STATE_RULES_H

#include "core_state.h"

inline bool core_should_reset_mode_init(const SystemState &state) {
  if (state.mode == 0 || state.mode == 1 || state.mode == 2) return false;
  if (state.mode == 3 && state.modeConfigured) return false;
  if (state.mode == 4 && state.modeConfigured) return false;
  return true;
}

#endif
