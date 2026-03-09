#ifndef APP_MODE_STATE_CONTEXT_H
#define APP_MODE_STATE_CONTEXT_H

#include <stdint.h>

uint8_t app_mode_state_mode();
void app_mode_state_set_mode(uint8_t mode);

int app_mode_state_function_index();
void app_mode_state_set_function_index(int index);

bool app_mode_state_initialized();
void app_mode_state_set_initialized(bool initialized);

bool app_mode_state_configured();
void app_mode_state_set_configured(bool configured);

#endif
