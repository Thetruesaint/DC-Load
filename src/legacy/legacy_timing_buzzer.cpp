#include "legacy_timing_buzzer.h"

#include "../app/app_timer_context.h"
#include "../hal/hal_alerts.h"
#include "../hal/hal_inputs.h"
#include "../hw/hw_objects.h"

#define mytimerStarted (app_timer_running_ref())
#define startTime (app_timer_start_time_ref())
#define elapsedSeconds (app_timer_elapsed_seconds_ref())

void legacy_beep_buzzer() {
  for (int i = 0; i < 2; i++) {
    hal_buzzer_set(true);
    hal_delay_ms(150);
    hal_buzzer_set(false);
    hal_delay_ms(150);
  }
}

void legacy_timer_start() {
  if (!mytimerStarted) {
    startTime = rtc.now();
    mytimerStarted = true;
  }
}

void legacy_timer_stop() {
  if (mytimerStarted) {
    DateTime now = rtc.now();
    TimeSpan elapsedTime = now - startTime;
    elapsedSeconds += elapsedTime.totalseconds();
    mytimerStarted = false;
  }
}

void legacy_timer_reset() {
  elapsedSeconds = 0.0;
  mytimerStarted = false;
}

float legacy_timer_get_total_seconds() {
  if (mytimerStarted) {
    DateTime now = rtc.now();
    TimeSpan elapsedTime = now - startTime;
    return elapsedSeconds + elapsedTime.totalseconds();
  }
  return elapsedSeconds;
}

String legacy_timer_get_time() {
  int totalSeconds = static_cast<int>(legacy_timer_get_total_seconds());

  int minutes = (totalSeconds / 60);
  int seconds = (totalSeconds % 60);

  String formattedTime = "";

  if (minutes < 10) {
    formattedTime += "0";
  }
  formattedTime += String(minutes) + ":";

  if (seconds < 10) {
    formattedTime += "0";
  }
  formattedTime += String(seconds);

  return formattedTime;
}
