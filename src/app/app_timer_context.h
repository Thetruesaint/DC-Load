#ifndef APP_TIMER_CONTEXT_H
#define APP_TIMER_CONTEXT_H

#include <RTClib.h>

bool& app_timer_running_ref();
DateTime& app_timer_start_time_ref();
float& app_timer_elapsed_seconds_ref();

#endif

