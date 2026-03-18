#include "app_runtime_sync.h"

#include <cstring>

#include "app_calibration_flow.h"
#include "app_battery_context.h"
#include "app_calibration_context.h"
#include "app_fan_context.h"
#include "app_limits_bootstrap.h"
#include "app_limits_context.h"
#include "app_load_context.h"
#include "app_measurements_context.h"
#include "app_mode_setpoint_context.h"
#include "app_mode_state_context.h"
#include "app_runtime_context.h"
#include "app_setpoint_context.h"
#include "app_transient_context.h"
#include "app_value_result_context.h"
#include "../config/system_constants.h"
#include "../core/core_config_flow.h"
#include "../hw/hw_objects.h"

namespace {
void capture_measurements_and_limits(RuntimeSnapshot *state) {
  if (state == nullptr) return;

  state->measuredCurrent_A = app_measurements_current_a();
  state->measuredVoltage_V = app_measurements_voltage_v();
  state->measuredPower_W = app_measurements_power_w();
  state->temp_C = static_cast<float>(app_measurements_temp_c());

  state->readingValue = app_setpoint_reading();
  state->currentCutOffA = app_limits_current_cutoff();
  state->powerCutOffW = app_limits_power_cutoff();
  state->tempCutOffC = app_limits_temp_cutoff();
}

void capture_runtime_controls(RuntimeSnapshot *state) {
  if (state == nullptr) return;

  state->encoderPositionRaw = app_runtime_encoder_position();
  state->encoderStep = app_runtime_encoder_step();
  state->encoderMaxRaw = static_cast<float>(app_runtime_encoder_max());
  state->cursorPosition = app_runtime_cursor_position();
  state->functionIndex = app_mode_state_function_index();
  state->loadEnabled = app_load_is_enabled();
  state->mode = app_mode_state_mode();
  state->modeInitialized = app_mode_state_initialized();
  state->modeConfigured = app_mode_state_configured();
}

void capture_mode_and_setup_state(RuntimeSnapshot *state) {
  if (state == nullptr) return;

  state->setCurrent_mA = app_load_set_current_mA();
  state->setPower_W = app_mode_setpoint_power_w();
  state->setResistance_Ohm = app_mode_setpoint_resistance_ohm();

  state->batteryCutoffVolts = app_battery_cutoff_volts_ref();
  state->batteryLife = app_battery_life_ref();
  state->batteryDone = app_battery_done_ref();
  std::strncpy(state->batteryType, app_battery_type_ref().c_str(), sizeof(state->batteryType) - 1);
  state->batteryType[sizeof(state->batteryType) - 1] = '\0';

  state->transientLowCurrentA = app_transient_low_current_ref();
  state->transientHighCurrentA = app_transient_high_current_ref();
  state->transientPeriodMs = static_cast<float>(app_transient_period_ref());
  state->transientListActiveStep = static_cast<uint8_t>(max(0, app_transient_current_step_ref()));
  state->transientListTotalSteps = static_cast<uint8_t>(max(0, app_transient_total_steps_ref() + 1));
  if (state->mode == TL && state->transientListTotalSteps > 0) {
    state->transientPeriodMs = static_cast<float>(app_transient_list_ref()[state->transientListActiveStep][1]);
  }
}

void capture_fan_state(RuntimeSnapshot *state) {
  if (state == nullptr) return;

  state->fanTempOnC = static_cast<float>(app_fan_temp_on_c());
  state->fanHoldSeconds = static_cast<float>(app_fan_hold_seconds());
  state->fanManualOverrideActive = app_fan_manual_override_active();
  state->fanManualStateOn = state->fanManualOverrideActive
                              ? app_fan_manual_state_on()
                              : app_fan_output_is_on();
}

void apply_limit_runtime_updates(const SystemState &state) {
  if (state.mode == CC || state.mode == BC) {
    app_setpoint_set_max_reading(app_limits_current_cutoff());
    app_runtime_set_encoder_max(static_cast<unsigned long>(app_setpoint_max_reading() * 1000.0f));
    if (app_runtime_encoder_position() > static_cast<float>(app_runtime_encoder_max())) {
      app_runtime_set_encoder_position(static_cast<float>(app_runtime_encoder_max()));
    }
    return;
  }

  if (state.mode == CP) {
    app_setpoint_set_max_reading(app_limits_power_cutoff());
    app_runtime_set_encoder_max(static_cast<unsigned long>(app_setpoint_max_reading() * 1000.0f));
    if (app_runtime_encoder_position() > static_cast<float>(app_runtime_encoder_max())) {
      app_runtime_set_encoder_position(static_cast<float>(app_runtime_encoder_max()));
    }
  }
}

void apply_runtime_controls(const SystemState &state) {
  app_runtime_set_encoder_position(state.encoderPositionRaw);
  app_runtime_set_encoder_step(state.encoderStep);
  app_runtime_set_cursor_position(state.cursorPosition);
  app_mode_state_set_function_index(state.functionIndex);
  app_mode_state_set_mode(state.mode);
  app_mode_state_set_initialized(state.modeInitialized);
  app_mode_state_set_configured(state.modeConfigured);
  app_setpoint_set_reading(state.readingValue);
  app_load_set_enabled(state.loadEnabled);
}

void apply_mode_and_setup_state(const SystemState &state) {
  app_battery_cutoff_volts_ref() = state.batteryCutoffVolts;
  app_battery_done_ref() = state.batteryDone;
  app_battery_type_ref() = String(state.batteryType);
  app_transient_apply_from_state(state);
  app_fan_set_manual_override(state.fanManualOverrideActive, state.fanManualStateOn);
}

void apply_config_events(const SystemState &state) {
  if (state.limitsSaveEvent) {
    app_limits_apply_and_save(state.limitsDraftCurrentA, state.limitsDraftPowerW, state.limitsDraftTempC);
    apply_limit_runtime_updates(state);
  }

  if (state.fanSaveEvent) {
    app_fan_save_settings(static_cast<int>(state.fanDraftTempC), static_cast<uint8_t>(state.fanDraftHoldSeconds));
  }

  if (state.calibrationMenuApplyEvent) {
    app_calibration_apply_menu_option(state.calibrationMenuOption);
  }

  if (state.calibrationValueConfirmEvent && state.mode == CA) {
    app_calibration_confirm_value(state.calibrationRealValue);
  }

  if (core_config_wants_calibration(state)) {
    app_calibration_run_setup();
  }
}

void apply_managed_mode_setpoints(const SystemState &state) {
  if (state.mode == CC || state.mode == CP || state.mode == CR || state.mode == CA) {
    app_load_set_set_current_mA(state.setCurrent_mA);
    app_mode_setpoint_set_power_w(state.setPower_W);
    app_mode_setpoint_set_resistance_ohm(state.setResistance_Ohm);
  }
}
}

RuntimeSnapshot app_runtime_sync_capture() {
  RuntimeSnapshot state = core_runtime_snapshot_make_default();

  capture_runtime_controls(&state);
  capture_measurements_and_limits(&state);
  capture_fan_state(&state);
  capture_mode_and_setup_state(&state);

  return state;
}

void app_runtime_sync_apply(const SystemState &state) {
  static bool lastAppliedLoadEnabled = false;
  static bool initialized = false;

  const bool previous = initialized ? lastAppliedLoadEnabled : app_load_is_enabled();

  apply_runtime_controls(state);
  apply_mode_and_setup_state(state);
  apply_config_events(state);
  apply_managed_mode_setpoints(state);

#ifndef WOKWI_SIMULATION
  if (initialized && previous != app_load_is_enabled() && !app_load_is_enabled()) {
    dac.setVoltage(0, false);
  }
#endif

  lastAppliedLoadEnabled = app_load_is_enabled();
  initialized = true;
}


