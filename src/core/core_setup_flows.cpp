#include "core_setup_flows.h"

#include <Arduino.h>
#include <cstdlib>
#include <cstring>

#include "../config/system_constants.h"
#include "core_state_rules.h"

bool numeric_input_append_digit(char *buffer, uint8_t *length, uint8_t maxDigits, char key) {
  if (buffer == nullptr || length == nullptr) return false;
  if (key < '0' || key > '9' || *length >= maxDigits) return false;
  buffer[(*length)++] = key;
  buffer[*length] = '\0';
  return true;
}

bool numeric_input_backspace(char *buffer, uint8_t *length) {
  if (buffer == nullptr || length == nullptr || *length == 0) return false;
  (*length)--;
  buffer[*length] = '\0';
  return true;
}

void battery_input_reset(SystemState *state) {
  if (state == nullptr) return;
  state->batteryInputText[0] = '\0';
  state->batteryInputLength = 0;
  state->batteryInputHasDecimal = false;
}

void battery_type_set(SystemState *state, const char *type) {
  if (state == nullptr || type == nullptr) return;
  std::strncpy(state->batteryType, type, sizeof(state->batteryType) - 1);
  state->batteryType[sizeof(state->batteryType) - 1] = '\0';
}

void battery_setup_begin(SystemState *state) {
  if (state == nullptr) return;
  state->batterySetupStage = 0;
  battery_input_reset(state);
}

void battery_setup_select_task(SystemState *state, char key) {
  if (state == nullptr) return;

  switch (key) {
    case '1':
      state->batteryCutoffVolts = LIPO_STOR_CELL_VLTG;
      battery_type_set(state, "Li-Po");
      state->batterySetupStage = 2;
      break;
    case '2':
      state->batteryCutoffVolts = LION_STOR_CELL_VLTG;
      battery_type_set(state, "Li-Ion");
      state->batterySetupStage = 2;
      break;
    case '3':
      state->batteryCutoffVolts = LIPO_DISC_CELL_VLTG;
      battery_type_set(state, "Li-Po");
      state->batterySetupStage = 2;
      break;
    case '4':
      state->batteryCutoffVolts = LION_DISC_CELL_VLTG;
      battery_type_set(state, "Li-Ion");
      state->batterySetupStage = 2;
      break;
    case '5':
      battery_type_set(state, "Custom");
      state->batterySetupStage = 1;
      break;
    default:
      return;
  }

  battery_input_reset(state);
}

bool battery_custom_append(SystemState *state, char key) {
  if (state == nullptr) return false;
  if (key == '.') {
    if (state->batteryInputHasDecimal || state->batteryInputLength >= 5) return false;
    state->batteryInputText[state->batteryInputLength++] = '.';
    state->batteryInputText[state->batteryInputLength] = '\0';
    state->batteryInputHasDecimal = true;
    return true;
  }
  if (key < '0' || key > '9' || state->batteryInputLength >= 5) return false;
  state->batteryInputText[state->batteryInputLength++] = key;
  state->batteryInputText[state->batteryInputLength] = '\0';
  return true;
}

bool battery_input_backspace(SystemState *state) {
  if (state == nullptr || state->batteryInputLength == 0) return false;
  state->batteryInputLength--;
  if (state->batteryInputText[state->batteryInputLength] == '.') {
    state->batteryInputHasDecimal = false;
  }
  state->batteryInputText[state->batteryInputLength] = '\0';
  return true;
}

void battery_setup_finish_custom(SystemState *state) {
  if (state == nullptr || state->batteryInputLength == 0) return;
  const float value = static_cast<float>(atof(state->batteryInputText));
  if (value < 0.1f || value > 25.0f) return;
  state->batteryCutoffVolts = value;
  state->modeConfigured = true;
  if (core_should_reset_mode_init(*state)) state->modeInitialized = false;
  state->batterySetupStage = 0;
  battery_input_reset(state);
}

void battery_setup_finish_cells(SystemState *state) {
  if (state == nullptr || state->batteryInputLength == 0) return;
  const int cells = atoi(state->batteryInputText);
  if (cells < 1 || cells > 6) return;
  state->batteryCutoffVolts *= static_cast<float>(cells);
  state->modeConfigured = true;
  if (core_should_reset_mode_init(*state)) state->modeInitialized = false;
  state->batterySetupStage = 0;
  battery_input_reset(state);
}

void transient_input_reset(SystemState *state) {
  if (state == nullptr) return;
  state->transientInputText[0] = '\0';
  state->transientInputLength = 0;
  state->transientInputHasDecimal = false;
}

void transient_cont_setup_begin(SystemState *state) {
  if (state == nullptr) return;
  state->transientSetupStage = 0;
  state->transientLowCurrentA = state->currentCutOffA;
  state->transientHighCurrentA = state->currentCutOffA;
  state->transientPeriodMs = 1000.0f;
  transient_input_reset(state);
}

bool transient_input_append(SystemState *state, char key) {
  if (state == nullptr) return false;
  if (state->transientSetupStage < 2) {
    if (key == '.') {
      if (state->transientInputHasDecimal || state->transientInputLength >= 5) return false;
      state->transientInputText[state->transientInputLength++] = '.';
      state->transientInputText[state->transientInputLength] = '\0';
      state->transientInputHasDecimal = true;
      return true;
    }
    if (key < '0' || key > '9' || state->transientInputLength >= 5) return false;
    state->transientInputText[state->transientInputLength++] = key;
    state->transientInputText[state->transientInputLength] = '\0';
    return true;
  }
  if (key < '0' || key > '9' || state->transientInputLength >= 5) return false;
  state->transientInputText[state->transientInputLength++] = key;
  state->transientInputText[state->transientInputLength] = '\0';
  return true;
}

bool transient_input_backspace(SystemState *state) {
  if (state == nullptr || state->transientInputLength == 0) return false;
  state->transientInputLength--;
  if (state->transientInputText[state->transientInputLength] == '.') {
    state->transientInputHasDecimal = false;
  }
  state->transientInputText[state->transientInputLength] = '\0';
  return true;
}

void transient_cont_setup_commit(SystemState *state) {
  if (state == nullptr || state->transientInputLength == 0) return;

  if (state->transientSetupStage == 0) {
    const float value = constrain(static_cast<float>(atof(state->transientInputText)), 0.0f, state->currentCutOffA);
    state->transientLowCurrentA = value;
    state->transientSetupStage = 1;
    transient_input_reset(state);
    return;
  }

  if (state->transientSetupStage == 1) {
    const float value = constrain(static_cast<float>(atof(state->transientInputText)), 0.0f, state->currentCutOffA);
    state->transientHighCurrentA = value;
    state->transientSetupStage = 2;
    transient_input_reset(state);
    return;
  }

  const float value = static_cast<float>(atoi(state->transientInputText));
  if (value < 1.0f || value > 99999.0f) return;
  state->transientPeriodMs = value;
  state->modeConfigured = true;
  state->modeInitialized = false;
  state->transientSetupStage = 0;
  transient_input_reset(state);
}

void transient_list_input_reset(SystemState *state) {
  if (state == nullptr) return;
  state->transientListInputText[0] = '\0';
  state->transientListInputLength = 0;
  state->transientListInputHasDecimal = false;
}

bool transient_list_input_append(SystemState *state, char key) {
  if (state == nullptr) return false;
  if (state->transientListSetupStage == 0) {
    if (key < '0' || key > '9' || state->transientListInputLength >= 2) return false;
    state->transientListInputText[state->transientListInputLength++] = key;
    state->transientListInputText[state->transientListInputLength] = '\0';
    return true;
  }

  if (state->transientListDraftField == 0) {
    if (key == '.') {
      if (state->transientListInputHasDecimal || state->transientListInputLength >= 5) return false;
      state->transientListInputText[state->transientListInputLength++] = '.';
      state->transientListInputText[state->transientListInputLength] = '\0';
      state->transientListInputHasDecimal = true;
      return true;
    }
    if (key < '0' || key > '9' || state->transientListInputLength >= 5) return false;
    state->transientListInputText[state->transientListInputLength++] = key;
    state->transientListInputText[state->transientListInputLength] = '\0';
    return true;
  }

  if (key < '0' || key > '9' || state->transientListInputLength >= 5) return false;
  state->transientListInputText[state->transientListInputLength++] = key;
  state->transientListInputText[state->transientListInputLength] = '\0';
  return true;
}

bool transient_list_input_backspace(SystemState *state) {
  if (state == nullptr || state->transientListInputLength == 0) return false;
  state->transientListInputLength--;
  if (state->transientListInputText[state->transientListInputLength] == '.') {
    state->transientListInputHasDecimal = false;
  }
  state->transientListInputText[state->transientListInputLength] = '\0';
  return true;
}

void transient_list_setup_commit(SystemState *state) {
  if (state == nullptr || state->transientListInputLength == 0) return;

  if (state->transientListSetupStage == 0) {
    const int steps = atoi(state->transientListInputText);
    if (steps < 2 || steps > 10) return;
    state->transientListDraftStepCount = static_cast<uint8_t>(steps);
    state->transientListSetupStage = 1;
    state->transientListDraftStepIndex = 0;
    state->transientListDraftField = 0;
    transient_list_input_reset(state);
    return;
  }

  if (state->transientListDraftField == 0) {
    const float current = constrain(static_cast<float>(atof(state->transientListInputText)), 0.0f, state->currentCutOffA);
    state->transientListDraftCurrentsA[state->transientListDraftStepIndex] = current;
    state->transientListDraftField = 1;
    transient_list_input_reset(state);
    return;
  }

  const float period = static_cast<float>(atoi(state->transientListInputText));
  if (period < 10.0f || period > 10000.0f) return;
  state->transientListDraftPeriodsMs[state->transientListDraftStepIndex] = period;
  transient_list_input_reset(state);

  if ((state->transientListDraftStepIndex + 1) < state->transientListDraftStepCount) {
    state->transientListDraftStepIndex++;
    state->transientListDraftField = 0;
    return;
  }

  state->modeConfigured = true;
  state->modeInitialized = false;
  state->transientListSetupStage = 0;
  state->transientListDraftField = 0;
}
