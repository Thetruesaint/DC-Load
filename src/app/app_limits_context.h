#ifndef APP_LIMITS_CONTEXT_H
#define APP_LIMITS_CONTEXT_H

float app_limits_current_cutoff();
void app_limits_set_current_cutoff(float value);

float app_limits_power_cutoff();
void app_limits_set_power_cutoff(float value);

float app_limits_temp_cutoff();
void app_limits_set_temp_cutoff(float value);

#endif
