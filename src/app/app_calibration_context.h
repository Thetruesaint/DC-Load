#ifndef APP_CALIBRATION_CONTEXT_H
#define APP_CALIBRATION_CONTEXT_H

#include <stdint.h>

struct AppCalibrationComputationResult {
  bool ready;
  bool pointsTooClose;
  bool pointMismatch;
  float sensorFactor;
  float sensorOffset;
  float outputFactor;
  float outputOffset;
};

bool app_calibration_is_voltage_mode();
void app_calibration_set_voltage_mode(bool enabled);

bool app_calibration_first_point_taken();
void app_calibration_set_first_point_taken(bool taken);

void app_calibration_reset_session();
void app_calibration_begin_mode(bool voltageMode);
void app_calibration_begin_mode_from_selection(float selection);
bool app_calibration_capture_or_compute(
    float measuredValue,
    float realValue,
    float setCurrentA,
    AppCalibrationComputationResult &result);
void app_calibration_apply_result(const AppCalibrationComputationResult &result);
void app_calibration_finish_mode();

void app_calibration_store_return_mode(uint8_t mode, int functionIndex);
uint8_t app_calibration_return_mode();
int app_calibration_return_function_index();
void app_calibration_request_menu_return();
bool app_calibration_consume_menu_return_request();

float& app_calibration_sns_volt_factor_ref();
float& app_calibration_sns_volt_offset_ref();
float& app_calibration_sns_curr_factor_ref();
float& app_calibration_sns_curr_offset_ref();
float& app_calibration_out_curr_factor_ref();
float& app_calibration_out_curr_offset_ref();

#endif
