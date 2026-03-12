#include "app_transient_context.h"

#include "app_load_context.h"
#include "app_mode_state_context.h"
#include "../config/system_constants.h"

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

void app_transient_apply_from_state(const SystemState &state) {
  if (state.mode == TC) {
    app_transient_low_current_ref() = state.transientLowCurrentA;
    app_transient_high_current_ref() = state.transientHighCurrentA;
    app_transient_period_ref() = static_cast<unsigned long>(state.transientPeriodMs);
    return;
  }

  if (state.mode == TL && state.modeConfigured) {
    for (int i = 0; i < state.transientListDraftStepCount && i < 10; ++i) {
      app_transient_list_ref()[i][0] = static_cast<unsigned long>(state.transientListDraftCurrentsA[i] * 1000.0f);
      app_transient_list_ref()[i][1] = static_cast<unsigned long>(state.transientListDraftPeriodsMs[i]);
    }
    app_transient_total_steps_ref() = (state.transientListDraftStepCount > 0) ? static_cast<int>(state.transientListDraftStepCount) - 1 : 0;
    if (!app_mode_state_initialized()) {
      app_load_set_set_current_mA(0.0f);
      app_transient_current_step_ref() = 0;
      app_transient_period_ref() = app_transient_list_ref()[0][1];
    }
  }
}
