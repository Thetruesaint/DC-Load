#include "app_calibration_context.h"

#include <math.h>

#include "../config/system_constants.h"

namespace {
bool calibrationVoltageMode = false;
bool calibrationFirstPointTaken = false;

float snsVoltCalibFactor = 1.0f;
float snsVoltCalibOffset = 0.0f;
float snsCurrCalibFactor = 1.0f;
float snsCurrCalibOffset = 0.0f;
float outCurrCalibFactor = 1.0f;
float outCurrCalibOffset = 0.0f;

float firstMeasuredValue = 0.0f;
float firstRealValue = 0.0f;
float firstSetCurrentA = 0.0f;
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

void app_calibration_reset_session() {
  calibrationFirstPointTaken = false;
  firstMeasuredValue = 0.0f;
  firstRealValue = 0.0f;
  firstSetCurrentA = 0.0f;
}

void app_calibration_begin_mode(bool voltageMode) {
  calibrationVoltageMode = voltageMode;
  app_calibration_reset_session();

  if (voltageMode) {
    snsVoltCalibFactor = 1.0f;
    snsVoltCalibOffset = 0.0f;
    return;
  }

  snsCurrCalibFactor = 1.0f;
  snsCurrCalibOffset = 0.0f;
  outCurrCalibFactor = 1.0f;
  outCurrCalibOffset = 0.0f;
}

bool app_calibration_capture_or_compute(
    float measuredValue,
    float realValue,
    float setCurrentA,
    AppCalibrationComputationResult &result) {
  result = {false, false, false, 1.0f, 0.0f, 1.0f, 0.0f};

  if (!calibrationFirstPointTaken) {
    firstMeasuredValue = measuredValue;
    firstRealValue = realValue;
    firstSetCurrentA = setCurrentA;
    calibrationFirstPointTaken = true;
    return false;
  }

  calibrationFirstPointTaken = false;
  result.ready = true;

  const float measuredDelta = fabsf(measuredValue - firstMeasuredValue);
  const float setCurrentDelta = fabsf(setCurrentA - firstSetCurrentA);

  result.pointsTooClose = calibrationVoltageMode
      ? (measuredDelta < CAL_MIN_VOLTAGE_DELTA)
      : (setCurrentDelta < CAL_MIN_CURRENT_DELTA);

  if (!calibrationVoltageMode) {
    const float errRatio1 = fabsf(firstMeasuredValue - firstSetCurrentA) / max(firstSetCurrentA, 0.001f);
    const float errRatio2 = fabsf(measuredValue - setCurrentA) / max(setCurrentA, 0.001f);
    result.pointMismatch = (errRatio1 > CAL_MAX_POINT_ERROR_RATIO) || (errRatio2 > CAL_MAX_POINT_ERROR_RATIO);
  }

  if (result.pointsTooClose || result.pointMismatch) {
    return true;
  }

  result.sensorFactor = max(0.9f, min(1.1f, (realValue - firstRealValue) / (measuredValue - firstMeasuredValue)));
  result.sensorOffset = max(-0.1f, min(0.1f, firstRealValue - (firstMeasuredValue * result.sensorFactor)));

  if (!calibrationVoltageMode) {
    result.outputFactor = max(0.9f, min(1.1f, (realValue - firstRealValue) / (setCurrentA - firstSetCurrentA)));
    result.outputOffset = max(-0.1f, min(0.1f, firstRealValue - (firstSetCurrentA * result.outputFactor))) * 1000.0f;
  }

  return true;
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
