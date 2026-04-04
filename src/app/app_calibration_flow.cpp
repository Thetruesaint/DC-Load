#include "app_calibration_flow.h"

#include "app_calibration_context.h"
#include "app_encoder_setup.h"
#include "app_input_buffer.h"
#include "app_limits_context.h"
#include "app_load_context.h"
#include "app_load_output.h"
#include "app_loop.h"
#include "app_msc.h"
#include "app_measurements_context.h"
#include "app_mode_state_context.h"
#include "app_runtime_context.h"
#include "app_setpoint_cursor.h"
#include "app_setpoint_context.h"
#include "app_value_result_context.h"
#include "../ui_display.h"
#include "../ui/ui_state_machine.h"
#include "../config/system_constants.h"
#include "../storage_eeprom.h"

namespace {
constexpr unsigned long kCalibrationUiRefreshMs = TMP_CHK_TIME;

const char *calibration_abort_detail(const AppCalibrationComputationResult &result, bool voltageMode) {
  if (result.pointsTooClose) {
    return "P1/P2 too close";
  }
  if (result.point1SenseMismatch) {
    return voltageMode
        ? "P1 Sense Read/Real >30%"
        : "P1 Sense Read/Real >30%";
  }
  if (result.point1OutputMismatch) {
    return "P1 Out Set/Real >30%";
  }
  if (result.point2SenseMismatch) {
    return voltageMode
        ? "P2 Sense Read/Real >20%"
        : "P2 Sense Read/Real >20%";
  }
  if (result.point2OutputMismatch) {
    return "P2 Out Set/Real >20%";
  }
  return "Calibration mismatch";
}

void wait_for_calibration_abort_ack(const char *detail) {
  app_input_reset();
  uiDisplayRenderCalibrationAbortScreen(detail);

  while (true) {
    const char key = app_input_read_key();
    if (!app_input_is_no_key(key) && key == 'E') {
      app_input_reset();
      return;
    }

    delay(10);
  }
}

void refresh_calibration_temperature() {
  app_measurements_set_temp_c(app_measurements_read_temp_c());
}

void wait_for_key_release() {
  while (true) {
    const char key = app_input_read_key();
    if (app_input_is_no_key(key)) {
      return;
    }
    delay(10);
  }
}

template <typename RefreshFn>
char wait_for_key_with_refresh(RefreshFn refreshContent) {
  unsigned long lastRefreshMs = 0;

  while (true) {
    const char key = app_input_read_key();
    if (!app_input_is_no_key(key)) {
      return key;
    }

    const unsigned long now = millis();
    if ((now - lastRefreshMs) >= kCalibrationUiRefreshMs) {
      refresh_calibration_temperature();
      uiDisplayUpdateAccentChromeStatus();
      refreshContent();
      lastRefreshMs = now;
    }

    delay(10);
  }
}

void run_temp_calibration_entry() {
  float referenceTempC = 0.0f;
  app_input_reset();

  while (referenceTempC <= 0.0f || referenceTempC > MAX_TEMP) {
    refresh_calibration_temperature();
    uiDisplayRenderTempCalibrationEntryScreen(app_measurements_read_temp_raw_c(), app_input_text());

    while (true) {
      const char key = wait_for_key_with_refresh([]() {
        uiDisplayUpdateTempCalibrationRawValue(app_measurements_read_temp_raw_c());
      });

      if (key == '<' || key == 'M') {
        app_input_reset();
        wait_for_key_release();
        ui_state_machine_invalidate_menu_calibration();
        return;
      }

      if (!app_handle_msc_keys(key)) {
        app_input_reset();
        return;
      }

      bool handled = app_input_append_digit(key, 4);
      if (!handled && key == '.') {
        handled = app_input_append_decimal(4);
      }
      if (!handled && key == '<') {
        handled = app_input_backspace();
      }

      if (!handled && key == 'E' && app_input_length() > 0) {
        referenceTempC = app_input_parse_float();
        break;
      }

      uiDisplayRenderTempCalibrationEntryScreen(app_measurements_read_temp_raw_c(), app_input_text());
    }

    if (referenceTempC > 0.0f && referenceTempC <= MAX_TEMP) {
      const float rawTempC = app_measurements_read_temp_raw_c();
      const float factor = constrain(referenceTempC / max(rawTempC, 1.0f), TEMP_CAL_FACTOR_MIN, TEMP_CAL_FACTOR_MAX);
      app_calibration_temp_factor_ref() = factor;
      app_measurements_set_temp_c(app_measurements_read_temp_c());
      uiDisplayRenderCalibrationNoticeScreen("CAL TEMP", "Factor updated");
      delay(1500);
      app_input_reset();
      ui_state_machine_invalidate_menu_calibration();
      return;
    }

    uiDisplayRenderCalibrationNoticeScreen("CAL TEMP", "Invalid temp");
    delay(1200);
    app_input_reset();
  }
}
}  // namespace

void app_calibration_mode_update() {
  static bool confirmationDrawn = false;

  if (!app_mode_state_configured()) {
    confirmationDrawn = false;
    app_calibration_run_setup();
    return;
  }

  if (app_calibration_confirmation_active()) {
    if (!confirmationDrawn) {
      uiDisplayRenderCalibrationResultScreen(
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
    uiDisplayInvalidateHomeLayout();
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
  float selection = 0.0f;
  app_input_reset();
  do {
    refresh_calibration_temperature();
    uiDisplayRenderCalibrationSetupMenu(app_input_text());

    while (true) {
      char key = wait_for_key_with_refresh([]() {});

      if (!app_handle_msc_keys(key)) {
        app_input_reset();
        return;
      }

      bool handled = app_input_append_digit(key, 1);
      if (!handled && key == '<') {
        handled = app_input_backspace();
      }

      if (!handled && key == 'E' && app_input_length() > 0) {
        selection = app_input_parse_float();
        app_value_result_set(selection);
        app_input_reset();
        break;
      }

      uiDisplayRenderCalibrationSetupMenu(app_input_text());
    }
  } while (selection < 1.0f || selection > 6.0f);

  app_mode_state_set_configured(true);
  if (selection == 3.0f) {
    run_temp_calibration_entry();
    app_mode_state_set_configured(false);
  }
  if (selection == 4.0f) {
    Load_Calibration();
    uiDisplayRenderCalibrationNoticeScreen("CALIBRATION", "Loaded");
    delay(1500);
    app_mode_state_set_configured(false);
  }
  if (selection == 5.0f) {
    Save_Calibration();
    uiDisplayRenderCalibrationNoticeScreen("CALIBRATION", "Saved");
    delay(1500);
    app_mode_state_set_configured(false);
  }
  if (selection == 6.0f) {
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
    uiDisplayInvalidateHomeLayout();
    return;
  }

  app_load_output_off();
  app_setpoint_set_reading(0.0f);
  app_runtime_set_encoder_position(0.0f);
  app_load_set_set_current_mA(0.0f);

  if (result.pointsTooClose || result.pointMismatch) {
    wait_for_calibration_abort_ack(calibration_abort_detail(result, app_calibration_is_voltage_mode()));
    app_calibration_finish_mode();
    return;
  }

  app_calibration_prepare_pending_result(result);
  uiDisplayRenderCalibrationResultScreen(
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
    run_temp_calibration_entry();
    app_mode_state_set_configured(false);
    app_mode_state_set_initialized(false);
  } else if (option == 4) {
    Load_Calibration();
    uiDisplayRenderCalibrationNoticeScreen("CALIBRATION", "Loaded");
    delay(1500);
    app_mode_state_set_configured(false);
    app_mode_state_set_initialized(false);
  } else if (option == 5) {
    Save_Calibration();
    uiDisplayRenderCalibrationNoticeScreen("CALIBRATION", "Saved");
    delay(1500);
    app_mode_state_set_configured(false);
    app_mode_state_set_initialized(false);
  }
}
