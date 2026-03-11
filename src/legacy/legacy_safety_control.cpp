#include "legacy_safety_control.h"

#include "legacy_base_io.h"

#include "../config/system_constants.h"
#include "../hw/hw_objects.h"
#include "../ui/ui_mode_templates.h"
#include "../ui/ui_state_machine.h"
#include "../app/app_load_context.h"
#include "../app/app_runtime_context.h"
#include "../app/app_mode_state_context.h"
#include "../app/app_limits_context.h"
#include "../app/app_measurements_context.h"
#include "../app/app_setpoint_context.h"
#include "../app/app_fan_context.h"
#include "../app/app_value_input.h"

void legacy_temp_control() {
  static unsigned long fan_on_time = 0;
  static unsigned long last_tmpchk = 0;
  static bool fans_on = false;

  unsigned long currentMillis = millis();
  if ((currentMillis - last_tmpchk) < TMP_CHK_TIME) {
    return;
  }

  last_tmpchk = currentMillis;
  app_measurements_set_temp_c(static_cast<int>(analogRead(TEMP_SNSR) * TEMP_CONVERSION_FACTOR));

  if (app_measurements_temp_c() >= app_fan_temp_on_c()) {
    if (!fans_on) {
      digitalWrite(FAN_CTRL, HIGH);
      fans_on = true;
    }
    fan_on_time = currentMillis;
  } else if (fans_on && (currentMillis - fan_on_time) >= app_fan_hold_ms()) {
    digitalWrite(FAN_CTRL, LOW);
    fans_on = false;
  }

  if (ui_state_machine_current_screen() == UiScreen::Home) {
    ui_draw_header_temperature(app_measurements_temp_c());
  }
}

void legacy_check_limits() {
  char message[20] = "";
  float power = app_measurements_power_w();
  float maxpwrdis = constrain(249 - 1.4 * app_measurements_temp_c(), 0, 214);
  float actpwrdis = max(0.0f, power / 4);
  bool vlimit = false;
  bool ilimit = false;
  bool plimit = false;
  bool climit = false;

  if (app_measurements_voltage_v() > MAX_VOLTAGE) {
    strcpy(message, "Max Voltage!      ");
    vlimit = true;
  } else if (app_measurements_current_a() > app_limits_current_cutoff() * 1.01) {
    strcpy(message, "Current Cut Off!  ");
    ilimit = true;
  } else if (power > app_limits_power_cutoff()) {
    strcpy(message, "Power Cut Off!    ");
    plimit = true;
  } else if (app_measurements_temp_c() > app_limits_temp_cutoff()) {
    strcpy(message, "Over Temperature! ");
    climit = true;
  } else if (actpwrdis >= maxpwrdis) {
    strcpy(message, "Max PWR Disipation");
  }

  if (strlen(message) > 0) {
    legacy_load_off();
    app_setpoint_set_reading(0.0f);
    app_runtime_set_encoder_position(0.0f);
    encoder.clearCount();
    app_load_set_set_current_mA(0.0f);

    ui_blink_limit_alarm(message, vlimit, ilimit, plimit, climit);

    app_reset_input_pointers();
    app_mode_state_set_initialized(false);
  }
}
