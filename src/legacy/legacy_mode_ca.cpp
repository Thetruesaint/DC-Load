#include "legacy_mode_ca.h"

#include "legacy_base_io.h"

#include "../config/system_constants.h"
#include "../hw/hw_objects.h"
#include "../ui/ui_mode_templates.h"
#include "../app/app_load_context.h"
#include "../app/app_runtime_context.h"
#include "../app/app_mode_state_context.h"
#include "../app/app_limits_context.h"
#include "../app/app_measurements_context.h"
#include "../app/app_calibration_context.h"
#include "../app/app_setpoint_context.h"
#include "../app/app_value_input.h"
#include "../app/app_value_result_context.h"
#include "../storage_eeprom.h"
#define Sns_Volt_Calib_Fact (app_calibration_sns_volt_factor_ref())
#define Sns_Volt_Calib_Offs (app_calibration_sns_volt_offset_ref())
#define Sns_Curr_Calib_Fact (app_calibration_sns_curr_factor_ref())
#define Sns_Curr_Calib_Offs (app_calibration_sns_curr_offset_ref())
#define Out_Curr_Calib_Fact (app_calibration_out_curr_factor_ref())
#define Out_Curr_Calib_Offs (app_calibration_out_curr_offset_ref())

void legacy_calibration_mode() {
  if (!app_mode_state_configured()) {
    legacy_calibration_setup();
    return;
  }

  if (!app_mode_state_initialized()) {
    const float selection = app_value_result_get();
    app_calibration_begin_mode(selection == 1.0f);

    ui_draw_calibration_mode_template(
        app_calibration_is_voltage_mode(),
        app_calibration_first_point_taken());
    legacy_encoder_status(true, app_limits_current_cutoff());
    app_mode_state_set_initialized(true);
  }

  float readingValue = app_runtime_encoder_position() / 1000.0f;
  readingValue = min(app_setpoint_max_reading(), max(0.0f, readingValue));
  app_setpoint_set_reading(readingValue);
  app_runtime_set_encoder_position(readingValue * 1000.0f);
  legacy_cursor_position();

  if (!app_load_is_enabled()) {
    return;
  }
  app_load_set_set_current_mA(readingValue * 1000.0f);
}

void legacy_calibration_setup() {
  ui_draw_calibration_setup_menu();

  float selection = 0.0f;
  do {
    const int col = 1;
    const int row = 3;
    ui_prepare_value_input_prompt(col, row, 1);
    if (!app_value_input(col, row, 1, false)) {
      return;
    }
    selection = app_value_result_get();
  } while (selection < 1.0f || selection > 4.0f);

  app_mode_state_set_configured(true);
  if (selection == 3.0f) {
    Load_Calibration();
    ui_draw_calibration_loaded_message();
    delay(1500);
    app_mode_state_set_configured(false);
  }
  if (selection == 4.0f) {
    Save_Calibration();
    ui_draw_calibration_saved_message();
    delay(1500);
    app_mode_state_set_configured(false);
  }

  app_calibration_reset_session();
  app_mode_state_set_initialized(false);
}

void legacy_calibrate(float realValue) {
  const float measuredValue = app_calibration_is_voltage_mode() ? app_measurements_voltage_v() : app_measurements_current_a();
  const float setCurrentA = app_load_set_current_mA() / 1000.0f;

  AppCalibrationComputationResult result{};
  if (!app_calibration_capture_or_compute(measuredValue, realValue, setCurrentA, result)) {
    app_mode_state_set_initialized(false);
    return;
  }

  legacy_load_off();

  if (result.pointsTooClose || result.pointMismatch) {
    ui_draw_calibration_abort(result.pointsTooClose);
    app_mode_state_set_initialized(false);
    delay(2000);
    return;
  }

  if (app_calibration_is_voltage_mode()) {
    Sns_Volt_Calib_Fact = result.sensorFactor;
    Sns_Volt_Calib_Offs = result.sensorOffset;
  } else {
    Sns_Curr_Calib_Fact = result.sensorFactor;
    Sns_Curr_Calib_Offs = result.sensorOffset;
    Out_Curr_Calib_Fact = result.outputFactor;
    Out_Curr_Calib_Offs = result.outputOffset;
  }

  ui_draw_calibration_success();
  app_mode_state_set_configured(false);
  app_calibration_reset_session();
  delay(2000);
}
