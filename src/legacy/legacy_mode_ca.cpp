#include "legacy_mode_ca.h"

#include "legacy_base_io.h"

#include "../config/system_constants.h"
#include "../funciones.h"
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
    if (selection == 1.0f) {
      app_calibration_set_voltage_mode(true);
      Sns_Volt_Calib_Fact = 1.0;
      Sns_Volt_Calib_Offs = 0.0;
    } else if (selection == 2.0f) {
      app_calibration_set_voltage_mode(false);
      Sns_Curr_Calib_Fact = 1.0;
      Sns_Curr_Calib_Offs = 0.0;
      Out_Curr_Calib_Fact = 1.0;
      Out_Curr_Calib_Offs = 0.0;
    }

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
    if (!Value_Input(col, row, 1, false)) {
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

  app_calibration_set_first_point_taken(false);
  app_mode_state_set_initialized(false);
}

void legacy_calibrate(float realValue) {
  static float measuredValue1 = 0, realValue1 = 0;
  static float measuredValue2 = 0, realValue2 = 0;
  static float setCurrent1 = 0;
  static float setCurrent2 = 0;

  float measuredValue = app_calibration_is_voltage_mode() ? app_measurements_voltage_v() : app_measurements_current_a();

  if (!app_calibration_first_point_taken()) {
    measuredValue1 = measuredValue;
    realValue1 = realValue;
    setCurrent1 = app_load_set_current_mA() / 1000.0f;
    app_calibration_set_first_point_taken(true);
    app_mode_state_set_initialized(false);
    return;
  }

  measuredValue2 = measuredValue;
  realValue2 = realValue;
  setCurrent2 = app_load_set_current_mA() / 1000.0f;
  legacy_load_off();
  app_calibration_set_first_point_taken(false);

  float measuredDelta = fabsf(measuredValue2 - measuredValue1);
  float setCurrentDelta = fabsf(setCurrent2 - setCurrent1);

  bool pointsTooClose = app_calibration_is_voltage_mode()
    ? (measuredDelta < CAL_MIN_VOLTAGE_DELTA)
    : (setCurrentDelta < CAL_MIN_CURRENT_DELTA);

  float errRatio1 = 0.0f;
  float errRatio2 = 0.0f;
  bool pointMismatch = false;
  if (!app_calibration_is_voltage_mode()) {
    errRatio1 = fabsf(measuredValue1 - setCurrent1) / max(setCurrent1, 0.001f);
    errRatio2 = fabsf(measuredValue2 - setCurrent2) / max(setCurrent2, 0.001f);
    pointMismatch = (errRatio1 > CAL_MAX_POINT_ERROR_RATIO) || (errRatio2 > CAL_MAX_POINT_ERROR_RATIO);
  }

  if (pointsTooClose || pointMismatch) {
    ui_draw_calibration_abort(pointsTooClose);
    app_mode_state_set_initialized(false);
    delay(2000);
    return;
  }

  float factor = max(0.9f, min(1.1f, (realValue2 - realValue1) / (measuredValue2 - measuredValue1)));
  float offset = max(-0.1f, min(0.1f, realValue1 - (measuredValue1 * factor)));

  if (app_calibration_is_voltage_mode()) {
    Sns_Volt_Calib_Fact = factor;
    Sns_Volt_Calib_Offs = offset;
  } else {
    Sns_Curr_Calib_Fact = factor;
    Sns_Curr_Calib_Offs = offset;
    Out_Curr_Calib_Fact = max(0.9f, min(1.1f, (realValue2 - realValue1) / (setCurrent2 - setCurrent1)));
    Out_Curr_Calib_Offs = max(-0.1f, min(0.1f, realValue1 - (setCurrent1 * Out_Curr_Calib_Fact))) * 1000;
  }

  ui_draw_calibration_success();
  app_mode_state_set_configured(false);
  app_calibration_set_first_point_taken(false);
  delay(2000);
}
