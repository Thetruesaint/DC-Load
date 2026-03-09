#include "app_calibration_context.h"

namespace {
bool calibrationVoltageMode = false;
bool calibrationFirstPointTaken = false;
}

bool app_calibration_is_voltage_mode() {
  return calibrationVoltageMode;
}

void app_calibration_set_voltage_mode(bool enabled) {
  calibrationVoltageMode = enabled;
}

bool app_calibration_first_point_taken() {
  return calibrationFirstPointTaken;
}

void app_calibration_set_first_point_taken(bool taken) {
  calibrationFirstPointTaken = taken;
}
