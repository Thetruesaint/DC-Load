#include "app_mode_context.h"

#include "app_mode_state_context.h"
#include "../config/system_constants.h"

bool app_mode_config_allowed() {
  const uint8_t mode = app_mode_state_mode();
  return (mode != CA);
}

bool app_mode_is_transient() {
  const uint8_t mode = app_mode_state_mode();
  return (mode == TC || mode == TL);
}

bool app_mode_is_calibration() {
  return (app_mode_state_mode() == CA);
}

bool app_mode_is_battery() {
  return (app_mode_state_mode() == BC);
}

uint8_t app_mode_id() {
  return app_mode_state_mode();
}

