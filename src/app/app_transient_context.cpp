#include "app_transient_context.h"

namespace {
float lowCurrent = 0.0f;
float highCurrent = 0.0f;
unsigned long transientPeriod = 0;
unsigned long currentTime = 0;
unsigned long transientList[10][2] = {};
int totalSteps = 0;
int currentStep = 0;
}

float& app_transient_low_current_ref() {
  return lowCurrent;
}

float& app_transient_high_current_ref() {
  return highCurrent;
}

unsigned long& app_transient_period_ref() {
  return transientPeriod;
}

unsigned long& app_transient_current_time_ref() {
  return currentTime;
}

unsigned long (*app_transient_list_ref())[2] {
  return transientList;
}

int& app_transient_total_steps_ref() {
  return totalSteps;
}

int& app_transient_current_step_ref() {
  return currentStep;
}

