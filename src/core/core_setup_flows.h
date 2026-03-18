#ifndef CORE_SETUP_FLOWS_H
#define CORE_SETUP_FLOWS_H

#include "core_state.h"

bool numeric_input_append_digit(char *buffer, uint8_t *length, uint8_t maxDigits, char key);
bool numeric_input_backspace(char *buffer, uint8_t *length);

void battery_input_reset(SystemState *state);
void battery_type_set(SystemState *state, const char *type);
void battery_setup_begin(SystemState *state);
void battery_setup_select_task(SystemState *state, char key);
bool battery_custom_append(SystemState *state, char key);
bool battery_input_backspace(SystemState *state);
void battery_setup_finish_custom(SystemState *state);
void battery_setup_finish_cells(SystemState *state);

void transient_input_reset(SystemState *state);
void transient_cont_setup_begin(SystemState *state);
bool transient_input_append(SystemState *state, char key);
bool transient_input_backspace(SystemState *state);
void transient_cont_setup_commit(SystemState *state);

void transient_list_input_reset(SystemState *state);
bool transient_list_input_append(SystemState *state, char key);
bool transient_list_input_backspace(SystemState *state);
void transient_list_setup_commit(SystemState *state);

#endif
