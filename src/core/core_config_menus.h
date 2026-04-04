#ifndef CORE_CONFIG_MENUS_H
#define CORE_CONFIG_MENUS_H

#include "core_state.h"

ConfigMenu decode_config_root_target(int32_t raw);

void clear_config_runtime_flags(SystemState *state);
void reset_config_navigation(SystemState *state);
void reset_mode_entry_state(SystemState *state, bool preserveBatteryModeValues = false);

void config_menu_step_selection(SystemState *state, ConfigMenu menu, int direction);
void config_menu_select_index(SystemState *state, ConfigMenu menu, uint8_t index);
void config_menu_activate_selection(SystemState *state, ConfigMenu menu);
void config_menu_back(SystemState *state, ConfigMenu menu);

void limits_menu_move_field(SystemState *state, int direction);
void limits_menu_start_edit(SystemState *state);
void limits_menu_cancel_edit(SystemState *state);
bool limits_menu_append_digit(SystemState *state, char key);
bool limits_menu_append_decimal(SystemState *state);
bool limits_menu_backspace(SystemState *state);
void limits_menu_adjust_field(SystemState *state, int direction);
void limits_menu_commit_edit(SystemState *state);
void limits_menu_finish(SystemState *state, bool save, bool returnToParent = false);

bool fan_menu_append_digit(SystemState *state, char key);
bool fan_menu_backspace(SystemState *state);
void fan_menu_cancel_edit(SystemState *state);
void fan_menu_adjust_field(SystemState *state, int direction);
void fan_menu_commit_edit(SystemState *state);

bool clock_menu_append_digit(SystemState *state, char key);
bool clock_menu_backspace(SystemState *state);
void clock_menu_cancel_edit(SystemState *state);
void clock_menu_commit_edit(SystemState *state);
void clock_menu_adjust_field(SystemState *state, int direction);

#endif
