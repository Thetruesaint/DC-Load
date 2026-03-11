#ifndef APP_TIMING_ALERTS_H
#define APP_TIMING_ALERTS_H

#include <Arduino.h>

void app_beep_buzzer();
void app_timer_start();
void app_timer_stop();
void app_timer_reset();
float app_timer_get_total_seconds();
String app_timer_get_time();

#endif
