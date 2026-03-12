#include "app_timer_context.h"

namespace {
bool timerRunning = false;
DateTime timerStartTime;
float timerElapsedSeconds = 0.0f;
}

bool& app_timer_running_ref() {
  return timerRunning;
}

DateTime& app_timer_start_time_ref() {
  return timerStartTime;
}

float& app_timer_elapsed_seconds_ref() {
  return timerElapsedSeconds;
}

