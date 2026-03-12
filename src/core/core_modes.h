#ifndef CORE_MODES_H
#define CORE_MODES_H

#include <stdint.h>

#include "core_state.h"

bool core_mode_is_managed(uint8_t mode);
void core_mode_normalize_state(SystemState *state);
void core_mode_apply_encoder_delta(SystemState *state, int direction);
void core_mode_move_cursor(SystemState *state, int direction);
void core_mode_update_setpoints(SystemState *state);
void core_mode_apply_selection(SystemState *state, bool shiftPressed, char key);
void core_mode_update_ui_screen(SystemState *state);

#endif
