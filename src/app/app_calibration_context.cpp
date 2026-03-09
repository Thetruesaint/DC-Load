#include "app_calibration_context.h"

namespace {
bool calibrationVoltageMode = false;
bool calibrationFirstPointTaken = false;

float snsVoltCalibFactor = 1.0f;
float snsVoltCalibOffset = 0.0f;
float snsCurrCalibFactor = 1.0f;
float snsCurrCalibOffset = 0.0f;
float outCurrCalibFactor = 1.0f;
float outCurrCalibOffset = 0.0f;
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

float& app_calibration_sns_volt_factor_ref() {
  return snsVoltCalibFactor;
}

float& app_calibration_sns_volt_offset_ref() {
  return snsVoltCalibOffset;
}

float& app_calibration_sns_curr_factor_ref() {
  return snsCurrCalibFactor;
}

float& app_calibration_sns_curr_offset_ref() {
  return snsCurrCalibOffset;
}

float& app_calibration_out_curr_factor_ref() {
  return outCurrCalibFactor;
}

float& app_calibration_out_curr_offset_ref() {
  return outCurrCalibOffset;
}

