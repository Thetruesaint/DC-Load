#ifndef LEGACY_TIMING_BUZZER_H
#define LEGACY_TIMING_BUZZER_H

#include <Arduino.h>

void legacy_beep_buzzer();
void legacy_timer_start();
void legacy_timer_stop();
void legacy_timer_reset();
float legacy_timer_get_total_seconds();
String legacy_timer_get_time();

#endif

