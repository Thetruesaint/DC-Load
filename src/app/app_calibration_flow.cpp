#include "app_calibration_flow.h"

#include "app_calibration_context.h"
#include "app_encoder_setup.h"
#include "app_limits_context.h"
#include "app_load_context.h"
#include "app_load_output.h"
#include "app_measurements_context.h"
#include "app_mode_state_context.h"
#include "app_runtime_context.h"
#include "app_setpoint_cursor.h"
#include "app_setpoint_context.h"
#include "app_value_input.h"
#include "app_value_result_context.h"
#include "../config/system_constants.h"
#include "../storage_eeprom.h"
#include "../ui/ui_mode_templates.h"

void app_calibration_mode_update() {
  static bool confirmationDrawn = false;

  if (!app_mode_state_configured()) {
    confirmationDrawn = false;
    app_calibration_run_setup();
    return;
  }

  if (app_calibration_confirmation_active()) {
    if (!confirmationDrawn) {
      ui_draw_calibration_result(
          app_calibration_pending_is_voltage_mode(),
          app_calibration_pending_sensor_factor(),
          app_calibration_pending_sensor_offset(),
          app_calibration_pending_output_factor(),
          app_calibration_pending_output_offset());
      confirmationDrawn = true;
    }
    return;
  }

  confirmationDrawn = false;

  if (!app_mode_state_initialized()) {
    app_calibration_begin_mode_from_selection(app_value_result_get());
    ui_draw_calibration_mode_template(
        app_calibration_is_voltage_mode(),
        app_calibration_first_point_taken());
    app_encoder_setup_begin(app_limits_current_cutoff());
    app_mode_state_set_initialized(true);
  }

  float readingValue = app_runtime_encoder_position() / 1000.0f;
  readingValue = min(app_setpoint_max_reading(), max(0.0f, readingValue));
  app_setpoint_set_reading(readingValue);
  app_runtime_set_encoder_position(readingValue * 1000.0f);
  app_setpoint_cursor_update();

  if (!app_load_is_enabled()) {
    return;
  }
  app_load_set_set_current_mA(readingValue * 1000.0f);
}

void app_calibration_run_setup() {
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

void app_calibration_confirm_value(float realValue) {
  const float measuredValue = app_calibration_is_voltage_mode() ? app_measurements_voltage_v() : app_measurements_current_a();
  const float setCurrentA = app_load_set_current_mA() / 1000.0f;

  AppCalibrationComputationResult result{};
  if (!app_calibration_capture_or_compute(measuredValue, realValue, setCurrentA, result)) {
    ui_draw_calibration_mode_template(
        app_calibration_is_voltage_mode(),
        app_calibration_first_point_taken());
    return;
  }

  app_load_output_off();

  if (result.pointsTooClose || result.pointMismatch) {
    ui_draw_calibration_abort(result.pointsTooClose);
    app_calibration_finish_mode();
    delay(2000);
    return;
  }

  app_calibration_prepare_pending_result(result);
  ui_draw_calibration_result(
      app_calibration_pending_is_voltage_mode(),
      app_calibration_pending_sensor_factor(),
      app_calibration_pending_sensor_offset(),
      app_calibration_pending_output_factor(),
      app_calibration_pending_output_offset());
}

void app_calibration_apply_menu_option(uint8_t option) {
  if (option == 1 || option == 2) {
    app_calibration_store_return_mode(app_mode_state_mode(), app_mode_state_function_index());
    app_mode_state_set_mode(CA);
    app_mode_state_set_configured(true);
    app_mode_state_set_initialized(false);
    app_calibration_reset_session();
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
