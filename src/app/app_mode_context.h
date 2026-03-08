#ifndef APP_MODE_CONTEXT_H
#define APP_MODE_CONTEXT_H

#include <stdint.h>

bool app_mode_config_allowed();
bool app_mode_is_transient();
bool app_mode_is_calibration();
bool app_mode_is_battery();
uint8_t app_mode_id();

#endif
