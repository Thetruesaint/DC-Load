#include "app_calibration_context.h"

#include <math.h>

#include "../app/app_mode_state_context.h"
#include "../config/system_constants.h"
#include "../storage_eeprom.h"

namespace {
bool calibrationVoltageMode = false;
bool calibrationFirstPointTaken = false;
bool calibrationConfirmationActive = false;
bool calibrationPendingVoltageMode = false;
uint8_t calibrationReturnMode = 0;
int calibrationReturnFunctionIndex = 0;

float snsVoltCalibFactor = 1.0f;
float snsVoltCalibOffset = 0.0f;
float snsCurrCalibFactor = 1.0f;
float snsCurrCalibOffset = 0.0f;
float outCurrCalibFactor = 1.0f;
float outCurrCalibOffset = 0.0f;

float pendingSensorFactor = 1.0f;
float pendingSensorOffset = 0.0f;
float pendingOutputFactor = 1.0f;
float pendingOutputOffset = 0.0f;

float firstMeasuredValue = 0.0f;
float firstRealValue = 0.0f;
float firstSetCurrentA = 0.0f;
}

bool app_calibration_is_voltage_mode() {
  return calibrationVoltageMode;
}

bool app_calibration_first_point_taken() {
  return calibrationFirstPointTaken;
}

void app_calibration_reset_session() {
  calibrationFirstPointTaken = false;
  calibrationConfirmationActive = false;
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

void app_calibration_begin_mode_from_selection(float selection) {
  app_calibration_begin_mode(selection == 1.0f);
}

bool app_calibration_capture_or_compute(
    float measuredValue,
    float realValue,
    float setCurrentA,
    AppCalibrationComputationResult &result) {
  result = {false, false, false, false, false, false, false, 1.0f, 0.0f, 1.0f, 0.0f};

  if (!calibrationFirstPointTaken) {
    const float senseErrRatio1 = fabsf(measuredValue - realValue) / max(fabsf(measuredValue), 0.001f);
    result.ready = true;
    result.point1SenseMismatch = senseErrRatio1 > CAL_MAX_POINT1_ERROR_RATIO;

    if (!calibrationVoltageMode) {
      const float outputErrRatio1 = fabsf(setCurrentA - realValue) / max(fabsf(setCurrentA), 0.001f);
      result.point1OutputMismatch = outputErrRatio1 > CAL_MAX_POINT1_ERROR_RATIO;
    }

    result.pointMismatch = result.point1SenseMismatch || result.point1OutputMismatch;
    if (result.pointMismatch) {
      calibrationFirstPointTaken = false;
      return true;
    }

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

  if (result.pointsTooClose) {
    return true;
  }

  const float senseErrRatio2 = fabsf(measuredValue - realValue) / max(fabsf(measuredValue), 0.001f);
  result.point2SenseMismatch = senseErrRatio2 > CAL_MAX_POINT2_ERROR_RATIO;

  if (!calibrationVoltageMode) {
    const float outputErrRatio2 = fabsf(setCurrentA - realValue) / max(fabsf(setCurrentA), 0.001f);
    result.point2OutputMismatch = outputErrRatio2 > CAL_MAX_POINT2_ERROR_RATIO;
  }

  result.pointMismatch = result.point2SenseMismatch || result.point2OutputMismatch;

  if (result.pointMismatch) {
    return true;
  }

  result.sensorFactor = max(CAL_FACTOR_MIN, min(CAL_FACTOR_MAX, (realValue - firstRealValue) / (measuredValue - firstMeasuredValue)));
  result.sensorOffset = max(CAL_OFFSET_MIN, min(CAL_OFFSET_MAX, firstRealValue - (firstMeasuredValue * result.sensorFactor)));

  if (!calibrationVoltageMode) {
    result.outputFactor = max(CAL_FACTOR_MIN, min(CAL_FACTOR_MAX, (realValue - firstRealValue) / (setCurrentA - firstSetCurrentA)));
    result.outputOffset = max(CAL_OFFSET_MIN, min(CAL_OFFSET_MAX, firstRealValue - (firstSetCurrentA * result.outputFactor))) * 1000.0f;
  }

  return true;
}

void app_calibration_apply_result(const AppCalibrationComputationResult &result) {
  if (calibrationVoltageMode) {
    snsVoltCalibFactor = result.sensorFactor;
    snsVoltCalibOffset = result.sensorOffset;
    return;
  }

  snsCurrCalibFactor = result.sensorFactor;
  snsCurrCalibOffset = result.sensorOffset;
  outCurrCalibFactor = result.outputFactor;
  outCurrCalibOffset = result.outputOffset;
}

void app_calibration_prepare_pending_result(const AppCalibrationComputationResult &result) {
  calibrationPendingVoltageMode = calibrationVoltageMode;
  pendingSensorFactor = result.sensorFactor;
  pendingSensorOffset = result.sensorOffset;
  pendingOutputFactor = result.outputFactor;
  pendingOutputOffset = result.outputOffset;
  calibrationConfirmationActive = true;
}

bool app_calibration_confirmation_active() {
  return calibrationConfirmationActive;
}

bool app_calibration_pending_is_voltage_mode() {
  return calibrationPendingVoltageMode;
}

float app_calibration_pending_sensor_factor() {
  return pendingSensorFactor;
}

float app_calibration_pending_sensor_offset() {
  return pendingSensorOffset;
}

float app_calibration_pending_output_factor() {
  return pendingOutputFactor;
}

float app_calibration_pending_output_offset() {
  return pendingOutputOffset;
}

void app_calibration_accept_pending_result() {
  if (!calibrationConfirmationActive) return;

  AppCalibrationComputationResult result = {
    true,
    false,
    false,
    false,
    false,
    false,
    false,
    pendingSensorFactor,
    pendingSensorOffset,
    pendingOutputFactor,
    pendingOutputOffset
  };

  calibrationVoltageMode = calibrationPendingVoltageMode;
  app_calibration_apply_result(result);
  calibrationConfirmationActive = false;
}

void app_calibration_reject_pending_result() {
  if (!calibrationConfirmationActive) return;
  Load_Calibration();
  calibrationConfirmationActive = false;
}

void app_calibration_finish_mode() {
  app_mode_state_set_mode(calibrationReturnMode);
  app_mode_state_set_function_index(calibrationReturnFunctionIndex);
  app_mode_state_set_initialized(false);
  app_mode_state_set_configured(false);
  app_calibration_reset_session();
}

void app_calibration_store_return_mode(uint8_t mode, int functionIndex) {
  calibrationReturnMode = mode;
  calibrationReturnFunctionIndex = functionIndex;
}

uint8_t app_calibration_return_mode() {
  return calibrationReturnMode;
}

int app_calibration_return_function_index() {
  return calibrationReturnFunctionIndex;
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
