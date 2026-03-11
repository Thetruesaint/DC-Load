#include "legacy_bridge.h"

#include <cstring>

#include "legacy_mode_ca.h"
#include "legacy_mode_limits.h"

#include "../app/app_battery_context.h"
#include "../app/app_calibration_context.h"
#include "../app/app_fan_context.h"
#include "../app/app_limits_context.h"
#include "../app/app_load_context.h"
#include "../app/app_measurements_context.h"
#include "../app/app_mode_setpoint_context.h"
#include "../app/app_mode_state_context.h"
#include "../app/app_runtime_context.h"
#include "../app/app_setpoint_context.h"
#include "../app/app_transient_context.h"
#include "../app/app_value_result_context.h"
#include "../config/system_constants.h"
#include "../core/core_config_flow.h"
#include "../funciones.h"
#include "../hw/hw_objects.h"
#include "../ui/ui_mode_templates.h"

namespace {
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
}

SystemState legacy_capture_state() {
  SystemState state = core_state_make_default();

  state.setCurrent_mA = app_load_set_current_mA();
  state.setPower_W = app_mode_setpoint_power_w();
  state.setResistance_Ohm = app_mode_setpoint_resistance_ohm();

  state.measuredCurrent_A = app_measurements_current_a();
  state.measuredVoltage_V = app_measurements_voltage_v();
  state.measuredPower_W = app_measurements_power_w();
  state.temp_C = static_cast<float>(app_measurements_temp_c());

  state.readingValue = app_setpoint_reading();
  state.encoderPositionRaw = app_runtime_encoder_position();
  state.encoderStep = app_runtime_encoder_step();
  state.encoderMaxRaw = static_cast<float>(app_runtime_encoder_max());
  state.currentCutOffA = app_limits_current_cutoff();
  state.powerCutOffW = app_limits_power_cutoff();
  state.tempCutOffC = app_limits_temp_cutoff();
  state.fanTempOnC = static_cast<float>(app_fan_temp_on_c());
  state.fanHoldSeconds = static_cast<float>(app_fan_hold_seconds());
  state.batteryCutoffVolts = app_battery_cutoff_volts_ref();
  state.batteryLife = app_battery_life_ref();
  std::strncpy(state.batteryType, app_battery_type_ref().c_str(), sizeof(state.batteryType) - 1);
  state.batteryType[sizeof(state.batteryType) - 1] = '\0';
  state.transientLowCurrentA = app_transient_low_current_ref();
  state.transientHighCurrentA = app_transient_high_current_ref();
  state.transientPeriodMs = static_cast<float>(app_transient_period_ref());
  state.cursorPosition = app_runtime_cursor_position();
  state.functionIndex = app_mode_state_function_index();

  state.lastEncoderDelta = 0;
  state.lastKeyPressed = '\0';
  state.loadEnabled = app_load_is_enabled();
  state.mode = app_mode_state_mode();
  state.modeInitialized = app_mode_state_initialized();
  state.modeConfigured = app_mode_state_configured();

  core_state_clear_one_shot_events(&state);
  return state;
}

void legacy_apply_state(const SystemState &state) {
  static bool lastAppliedLoadEnabled = false;
  static bool initialized = false;

  const bool previous = initialized ? lastAppliedLoadEnabled : app_load_is_enabled();

  app_runtime_set_encoder_position(state.encoderPositionRaw);
  app_runtime_set_encoder_step(state.encoderStep);
  app_runtime_set_cursor_position(state.cursorPosition);
  app_mode_state_set_function_index(state.functionIndex);
  app_mode_state_set_mode(state.mode);
  app_mode_state_set_initialized(state.modeInitialized);
  app_mode_state_set_configured(state.modeConfigured);
  app_setpoint_set_reading(state.readingValue);
  app_load_set_enabled(state.loadEnabled);
  app_battery_cutoff_volts_ref() = state.batteryCutoffVolts;
  app_battery_type_ref() = String(state.batteryType);
  if (state.mode == TC) {
    app_transient_low_current_ref() = state.transientLowCurrentA;
    app_transient_high_current_ref() = state.transientHighCurrentA;
    app_transient_period_ref() = static_cast<unsigned long>(state.transientPeriodMs);
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

  if (state.limitsSaveEvent) {
    app_limits_set_current_cutoff(state.limitsDraftCurrentA);
    app_limits_set_power_cutoff(state.limitsDraftPowerW);
    app_limits_set_temp_cutoff(state.limitsDraftTempC);

    Save_EEPROM(ADD_CURRENT_CUT_OFF, app_limits_current_cutoff());
    Save_EEPROM(ADD_POWER_CUT_OFF, app_limits_power_cutoff());
    Save_EEPROM(ADD_TEMP_CUT_OFF, app_limits_temp_cutoff());

    apply_limit_runtime_updates(state);
  }

  if (state.fanSaveEvent) {
    app_fan_set_temp_on_c(static_cast<int>(state.fanDraftTempC));
    app_fan_set_hold_seconds(static_cast<uint8_t>(state.fanDraftHoldSeconds));
    Save_EEPROM(ADD_FAN_TEMP_ON, static_cast<float>(app_fan_temp_on_c()));
    Save_EEPROM(ADD_FAN_HOLD_MS, static_cast<float>(app_fan_hold_ms()));
  }

  if (state.calibrationMenuApplyEvent) {
    const uint8_t option = state.calibrationMenuOption;

    if (option == 1 || option == 2) {
      app_mode_state_set_mode(CA);
      app_mode_state_set_configured(true);
      app_mode_state_set_initialized(false);
      app_calibration_set_first_point_taken(false);
      app_value_result_set(static_cast<float>(option));
    } else if (option == 3) {
      Load_Calibration();
      ui_draw_calibration_loaded_message();
      delay(1500);
      app_mode_state_set_configured(false);
      app_mode_state_set_initialized(false);
    } else if (option == 4) {
      Save_Calibration();
      ui_draw_calibration_saved_message();
      delay(1500);
      app_mode_state_set_configured(false);
      app_mode_state_set_initialized(false);
    }
  }

  if (state.calibrationValueConfirmEvent && state.mode == CA) {
    legacy_calibrate(state.calibrationRealValue);
  }

  if (core_config_wants_limits(state)) {
    legacy_config_limits();
  } else if (core_config_wants_calibration(state)) {
    legacy_calibration_setup();
  }

  if (state.mode == CC || state.mode == CP || state.mode == CR || state.mode == CA) {
    app_load_set_set_current_mA(state.setCurrent_mA);
    app_mode_setpoint_set_power_w(state.setPower_W);
    app_mode_setpoint_set_resistance_ohm(state.setResistance_Ohm);
  }

#ifndef WOKWI_SIMULATION
  if (initialized && previous != app_load_is_enabled() && !app_load_is_enabled()) {
    dac.setVoltage(0, false);
  }
#endif

  lastAppliedLoadEnabled = app_load_is_enabled();
  initialized = true;
}


