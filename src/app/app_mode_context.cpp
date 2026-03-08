#include "app_mode_context.h"

#include "../variables.h"

bool app_mode_config_allowed() {
  return (Mode != TC && Mode != TL);
}

bool app_mode_is_transient() {
  return (Mode == TC || Mode == TL);
}

bool app_mode_is_calibration() {
  return (Mode == CA);
}

bool app_mode_is_battery() {
  return (Mode == BC);
}

uint8_t app_mode_id() {
  return static_cast<uint8_t>(Mode);
}
