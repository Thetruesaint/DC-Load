#ifndef APP_CALIBRATION_CONTEXT_H
#define APP_CALIBRATION_CONTEXT_H

bool app_calibration_is_voltage_mode();
void app_calibration_set_voltage_mode(bool enabled);

bool app_calibration_first_point_taken();
void app_calibration_set_first_point_taken(bool taken);

float& app_calibration_sns_volt_factor_ref();
float& app_calibration_sns_volt_offset_ref();
float& app_calibration_sns_curr_factor_ref();
float& app_calibration_sns_curr_offset_ref();
float& app_calibration_out_curr_factor_ref();
float& app_calibration_out_curr_offset_ref();

#endif

