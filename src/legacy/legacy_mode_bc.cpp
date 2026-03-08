#include "legacy_mode_bc.h"

#include "../variables.h"
#include "../ui_lcd.h"
#include "../funciones.h"
#include "../app/app_msc.h"
#include "../app/app_value_input.h"

void legacy_battery_mode() {
  if (!modeConfigured) {
    legacy_battery_type_selec();
    return;
  }

  if (!modeInitialized) {
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
    Encoder_Status(true, CurrentCutOff);
    modeInitialized = true;
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
    customKey = app_wait_key_pressed();

    if (!app_handle_msc_keys(customKey)) {
      return;
    }

    switch (customKey) {
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

    do {
      z = 7;
      r = 3;
      printLCD(z - 1, r, F(">"));
      Print_Spaces(z, r, 5);

      if (!Value_Input(z, r)) {
        return;
      }
    } while (x > 25 || x < 0.1);

    BatteryCutoffVolts = x;
  }

  if (BatteryType != "Custom") {
    clearLCD();
    printLCD_S(3, 0, BatteryType + " Batt");
    printLCD(6, 1, F("(1-6)S?"));

    do {
      z = 9;
      r = 2;
      printLCD(z - 1, r, F(">"));
      Print_Spaces(z, r, 5);
      if (!Value_Input(z, r, 1, false)) {
        return;
      }
    } while (x < 1 || x > 6);

    BatteryCutoffVolts *= x;
  }

  modeConfigured = true;
  modeInitialized = false;
}

bool legacy_battery_capacity() {
  float LoadCurrent = 0;
  unsigned long currentMillis = millis();
  static unsigned long lastUpdate = 0;

  if (toggle && voltage >= BatteryCutoffVolts && !mytimerStarted) {
    timer_start();
  }
  if (!toggle && voltage >= BatteryCutoffVolts && mytimerStarted) {
    timer_stop();
  }

  if (currentMillis - lastUpdate >= 500) {
    lastUpdate = currentMillis;

    printLCD_S(0, 3, timer_getTime());

    Seconds = timer_getTotalSeconds();
    LoadCurrent = (!mytimerStarted) ? 0 : current;
    BatteryLife += (LoadCurrent * 1000) / 7200;
  }

  reading = encoderPosition / 1000;
  reading = min(maxReading, max(0.0f, reading));
  encoderPosition = reading * 1000.0;

  if (!toggle) {
    return false;
  }

  setCurrent = reading * 1000;

  if (voltage <= BatteryCutoffVolts) {
    setCurrent = max(setCurrent - CRR_STEP_RDCTN, MIN_DISC_CURR);
    reading = setCurrent / 1000;
    encoderPosition = reading * 1000;
  }

  if (voltage <= (BatteryCutoffVolts - VLTG_DROP_MARGIN)) {
    BatteryCurrent = current;
    reading = 0;
    encoderPosition = 0;
    setCurrent = 0;
    encoder.clearCount();
    Load_OFF();
    timer_stop();
    beepBuzzer();
    return true;
  }

  return false;
}
