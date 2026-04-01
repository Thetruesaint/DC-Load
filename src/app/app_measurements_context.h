#ifndef APP_MEASUREMENTS_CONTEXT_H
#define APP_MEASUREMENTS_CONTEXT_H

#include <stdint.h>

float app_measurements_current_a();
void app_measurements_set_current_a(float value);

float app_measurements_voltage_v();
void app_measurements_set_voltage_v(float value);

int app_measurements_temp_c();
void app_measurements_set_temp_c(int value);
float app_measurements_read_temp_raw_c();
int app_measurements_read_temp_c();

float app_measurements_power_w();

#endif
