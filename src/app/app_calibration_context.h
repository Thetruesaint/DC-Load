#ifndef APP_CALIBRATION_CONTEXT_H
#define APP_CALIBRATION_CONTEXT_H

bool app_calibration_is_voltage_mode();
void app_calibration_set_voltage_mode(bool enabled);

bool app_calibration_first_point_taken();
void app_calibration_set_first_point_taken(bool taken);

#endif
