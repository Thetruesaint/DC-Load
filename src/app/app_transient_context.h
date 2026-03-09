#ifndef APP_TRANSIENT_CONTEXT_H
#define APP_TRANSIENT_CONTEXT_H

float& app_transient_low_current_ref();
float& app_transient_high_current_ref();
unsigned long& app_transient_period_ref();
unsigned long& app_transient_current_time_ref();
unsigned long (*app_transient_list_ref())[2];
int& app_transient_total_steps_ref();
int& app_transient_current_step_ref();

#endif

