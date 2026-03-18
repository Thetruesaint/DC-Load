#include "app_protection.h"

#include "../config/system_constants.h"
#include "../hal/hal_inputs.h"
#include "../hw/hw_objects.h"
#include "../ui_display.h"
#include "../ui/ui_mode_templates.h"
#include "../ui/ui_state_machine.h"
#include "app_fan_context.h"
#include "app_input_buffer.h"
#include "app_limits_context.h"
#include "app_load_context.h"
#include "app_load_output.h"
#include "app_measurements_context.h"
#include "app_mode_state_context.h"
#include "app_runtime_context.h"
#include "app_setpoint_context.h"
#include "app_value_input.h"

namespace {
constexpr unsigned long RUNOUT_SETTLE_MS = 200UL;
constexpr unsigned long PROTECTION_BLINK_MS = 250UL;

bool uses_legacy_home_temperature_overlay() {
  if (ui_state_machine_current_screen() != UiScreen::Home) return false;

  const uint8_t mode = app_mode_state_mode();
  return mode != CC && mode != CP && mode != CR && mode != BC && mode != TC && mode != TL && mode != CA;
}

void set_fan_output(bool on) {
  digitalWrite(FAN_CTRL, on ? HIGH : LOW);
  app_fan_set_output_state(on);
}

char protection_cause_code(bool vlimit, bool ilimit, bool plimit, bool climit) {
  if (vlimit) return 'V';
  if (ilimit) return 'I';
  if (plimit) return 'P';
  if (climit) return 'T';
  return '!';
}

void wait_for_protection_ack(const char *message, char causeCode) {
  app_reset_input_pointers();

  bool encoderWasPressed = false;
  int lastTempC = app_measurements_temp_c();
  uiDisplayInvalidateHomeLayout();
  uiDisplayRenderProtectionModal(message, causeCode);

  while (true) {
    const char key = app_input_read_key();
    if (!app_input_is_no_key(key) && key == 'E') {
      app_reset_input_pointers();
      return;
    }

    const bool encoderPressed = hal_encoder_button_low();
    if (encoderPressed && !encoderWasPressed) {
      while (hal_encoder_button_low()) {
        hal_delay_ms(10);
      }
      app_reset_input_pointers();
      return;
    }
    encoderWasPressed = encoderPressed;

    const int tempC = static_cast<int>(analogRead(TEMP_SNSR) * TEMP_CONVERSION_FACTOR);
    if (tempC != lastTempC) {
      app_measurements_set_temp_c(tempC);
      lastTempC = tempC;
      uiDisplayRenderProtectionModal(message, causeCode);
    }

    hal_delay_ms(10);
  }
}
}

void app_update_fan_control() {
  static unsigned long fan_on_time = 0;
  static unsigned long last_tmpchk = 0;
  static bool fans_on = false;
  static bool was_manual_override_active = false;

  const unsigned long currentMillis = millis();
  const bool manualOverrideActive = app_fan_manual_override_active();

  if (was_manual_override_active && !manualOverrideActive) {
    set_fan_output(false);
    fans_on = false;
    fan_on_time = currentMillis;
    last_tmpchk = currentMillis - TMP_CHK_TIME;
  }

  if (manualOverrideActive) {
    set_fan_output(app_fan_manual_state_on());
    fans_on = app_fan_manual_state_on();
    if (fans_on) {
      fan_on_time = currentMillis;
    }
    was_manual_override_active = true;
    if (uses_legacy_home_temperature_overlay()) {
      ui_draw_header_temperature(app_measurements_temp_c());
    }
    return;
  }

  if ((currentMillis - last_tmpchk) < TMP_CHK_TIME) {
    was_manual_override_active = false;
    return;
  }

  last_tmpchk = currentMillis;
  app_measurements_set_temp_c(static_cast<int>(analogRead(TEMP_SNSR) * TEMP_CONVERSION_FACTOR));

  if (app_measurements_temp_c() >= app_fan_temp_on_c()) {
    if (!fans_on) {
      set_fan_output(true);
      fans_on = true;
    }
    fan_on_time = currentMillis;
  } else if (fans_on && (currentMillis - fan_on_time) >= app_fan_hold_ms()) {
    set_fan_output(false);
    fans_on = false;
  }

  was_manual_override_active = false;

  if (uses_legacy_home_temperature_overlay()) {
    ui_draw_header_temperature(app_measurements_temp_c());
  }
}

void app_check_limits() {
  static float lastSetCurrentA = 0.0f;
  static unsigned long lastSetCurrentChangeMs = 0;

  char message[20] = "";
  const unsigned long currentMillis = millis();
  float power = app_measurements_power_w();
  float maxpwrdis = constrain(249 - 1.4 * app_measurements_temp_c(), 0, 214);
  float actpwrdis = max(0.0f, power / 4);
  const float measuredCurrent = app_measurements_current_a();
  const float setCurrentA = app_load_set_current_mA() / 1000.0f;
  bool vlimit = false;
  bool ilimit = false;
  bool plimit = false;
  bool climit = false;

  if (setCurrentA != lastSetCurrentA) {
    lastSetCurrentA = setCurrentA;
    lastSetCurrentChangeMs = currentMillis;
  }

  const bool runoutSettled = (currentMillis - lastSetCurrentChangeMs) >= RUNOUT_SETTLE_MS;

  if (app_measurements_voltage_v() > MAX_VOLTAGE) {
    strcpy(message, "Max Voltage!");
    vlimit = true;
  } else if (measuredCurrent > app_limits_current_cutoff() * 1.01f) {
    strcpy(message, "Current Cut Off!");
    ilimit = true;
  } else if (app_load_is_enabled() && setCurrentA > 0.0f && runoutSettled && measuredCurrent > setCurrentA * 1.11f) {
    strcpy(message, "Runout Cut Off!");
    ilimit = true;
  } else if (power > app_limits_power_cutoff()) {
    strcpy(message, "Power Cut Off!");
    plimit = true;
  } else if (app_measurements_temp_c() > app_limits_temp_cutoff()) {
    strcpy(message, "Over Temperature!");
    climit = true;
  } else if (actpwrdis >= maxpwrdis) {
    strcpy(message, "Max PWR Disipation");
  }

  if (strlen(message) > 0) {
    app_load_output_off();
    app_setpoint_set_reading(0.0f);
    app_runtime_set_encoder_position(0.0f);
    encoder.clearCount();

    wait_for_protection_ack(message, protection_cause_code(vlimit, ilimit, plimit, climit));

    app_reset_input_pointers();
    app_mode_state_set_initialized(false);
    ui_state_machine_reset();
  }
}

