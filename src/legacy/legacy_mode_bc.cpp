#include "legacy_mode_bc.h"

#include "../variables.h"
#include "../ui_lcd.h"
#include "../funciones.h"
#include "../app/app_msc.h"
#include "../app/app_value_input.h"
#include "../app/app_io_context.h"
#include "../app/app_mode_state_context.h"
#include "../app/app_limits_context.h"
#include "../app/app_load_context.h"
#include "../app/app_runtime_context.h"
#include "../app/app_measurements_context.h"
#include "../app/app_setpoint_context.h"
#include "../app/app_value_result_context.h"
#include "../app/app_battery_context.h"
#include "../app/app_timer_context.h"

#define BatteryLife (app_battery_life_ref())
#define BatteryLifePrevious (app_battery_life_previous_ref())
#define BatteryCutoffVolts (app_battery_cutoff_volts_ref())
#define BatteryCurrent (app_battery_current_ref())
#define BatteryType (app_battery_type_ref())
#define mytimerStarted (app_timer_running_ref())

void legacy_battery_mode() {
  if (!app_mode_state_configured()) {
    legacy_battery_type_selec();
    return;
  }

  if (!app_mode_state_initialized()) {
    clearLCD();
    printLCD(0, 0, F("BC LOAD"));
    setCursorLCD(13, 1);
    printLCDRaw(F(">"));
    printLCDNumber(14, 1, BatteryCutoffVolts, 'V', 2);
    printLCD(1, 2, F("Adj->"));
    printLCD(13, 2, F("A"));
    timer_reset();
    BatteryLife = 0;
    BatteryLifePrevious = 0;
    printLCDNumber(6, 3, BatteryLife, ' ', 0);
    printLCDRaw(F("mAh"));
    printLCD_S(14, 3, BatteryType);
    Encoder_Status(true, app_limits_current_cutoff());
    app_mode_state_set_initialized(true);
  }

  if (BatteryLife > BatteryLifePrevious) {
    printLCDNumber(6, 3, BatteryLife, ' ', 0);
    printLCDRaw(F("mAh"));
    Print_Spaces(16, 2, 4);
    BatteryLifePrevious = BatteryLife;
  }

  if (legacy_battery_capacity()) {
    printLCD(16, 2, F("Done"));
  }

  Cursor_Position();
}

void legacy_battery_type_selec() {
  clearLCD();
  printLCD(2, 0, F("Set Task & Batt"));
  printLCD(0, 1, F("Stor. 1)LiPo 2)LiIOn"));
  printLCD(0, 2, F("Disc. 3)LiPo 4)LiIOn"));
  printLCD(2, 3, F("5)Cutoff Voltage"));

  while (true) {
    const char key = app_wait_key_pressed();

    if (!app_handle_msc_keys(key)) {
      return;
    }

    switch (key) {
      case '1':
        BatteryCutoffVolts = LIPO_STOR_CELL_VLTG;
        BatteryType = "Li-Po";
        break;
      case '2':
        BatteryCutoffVolts = LION_STOR_CELL_VLTG;
        BatteryType = "Li-Ion";
        break;
      case '3':
        BatteryCutoffVolts = LIPO_DISC_CELL_VLTG;
        BatteryType = "Li-Po";
        break;
      case '4':
        BatteryCutoffVolts = LION_DISC_CELL_VLTG;
        BatteryType = "Li-Ion";
        break;
      case '5':
        BatteryType = "Custom";
        break;
      default:
        continue;
    }
    break;
  }

  if (BatteryType == "Custom") {
    clearLCD();
    printLCD_S(3, 0, BatteryType + " Batt");
    printLCD(2, 1, F("Voltage Cutoff?"));
    printLCD(5, 2, F("(0.1-25)V"));

    float inputValue = 0.0f;
    do {
      const int col = 7;
      const int row = 3;
      printLCD(col - 1, row, F(">"));
      Print_Spaces(col, row, 5);

      if (!Value_Input(col, row)) {
        return;
      }
      inputValue = app_value_result_get();
    } while (inputValue > 25 || inputValue < 0.1f);

    BatteryCutoffVolts = inputValue;
  }

  if (BatteryType != "Custom") {
    clearLCD();
    printLCD_S(3, 0, BatteryType + " Batt");
    printLCD(6, 1, F("(1-6)S?"));

    float inputValue = 0.0f;
    do {
      const int col = 9;
      const int row = 2;
      printLCD(col - 1, row, F(">"));
      Print_Spaces(col, row, 5);
      if (!Value_Input(col, row, 1, false)) {
        return;
      }
      inputValue = app_value_result_get();
    } while (inputValue < 1 || inputValue > 6);

    BatteryCutoffVolts *= inputValue;
  }

  app_mode_state_set_configured(true);
  app_mode_state_set_initialized(false);
}

bool legacy_battery_capacity() {
  float LoadCurrent = 0;
  const unsigned long currentMillis = app_io_millis();
  static unsigned long lastUpdate = 0;

  if (app_load_is_enabled() && app_measurements_voltage_v() >= BatteryCutoffVolts && !mytimerStarted) {
    timer_start();
  }
  if (!app_load_is_enabled() && app_measurements_voltage_v() >= BatteryCutoffVolts && mytimerStarted) {
    timer_stop();
  }

  if (currentMillis - lastUpdate >= 500) {
    lastUpdate = currentMillis;

    printLCD_S(0, 3, timer_getTime());

    LoadCurrent = (!mytimerStarted) ? 0 : app_measurements_current_a();
    BatteryLife += (LoadCurrent * 1000) / 7200;
  }

  float readingValue = app_runtime_encoder_position() / 1000.0f;
  readingValue = min(app_setpoint_max_reading(), max(0.0f, readingValue));
  app_setpoint_set_reading(readingValue);
  app_runtime_set_encoder_position(readingValue * 1000.0f);

  if (!app_load_is_enabled()) {
    return false;
  }

  app_load_set_set_current_mA(readingValue * 1000.0f);

  if (app_measurements_voltage_v() <= BatteryCutoffVolts) {
    const float nextCurrent = max(app_load_set_current_mA() - CRR_STEP_RDCTN, MIN_DISC_CURR);
    app_load_set_set_current_mA(nextCurrent);
    readingValue = app_load_set_current_mA() / 1000.0f;
    app_setpoint_set_reading(readingValue);
    app_runtime_set_encoder_position(readingValue * 1000.0f);
  }

  if (app_measurements_voltage_v() <= (BatteryCutoffVolts - VLTG_DROP_MARGIN)) {
    BatteryCurrent = app_measurements_current_a();
    app_setpoint_set_reading(0.0f);
    app_runtime_set_encoder_position(0.0f);
    app_load_set_set_current_mA(0.0f);
    encoder.clearCount();
    Load_OFF();
    timer_stop();
    beepBuzzer();
    return true;
  }

  return false;
}

