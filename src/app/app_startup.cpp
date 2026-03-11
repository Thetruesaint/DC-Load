#include "app_startup.h"

#include <RTClib.h>
#include <Wire.h>

#include "../config/system_constants.h"
#include "../hw/hw_objects.h"
#include "../legacy/legacy_mode_limits.h"
#include "../storage_eeprom.h"
#include "../ui_lcd.h"
#include "app_fan_context.h"
#include "app_health_context.h"
#include "app_limits_bootstrap.h"
#include "app_limits_context.h"
#include "app_measurements_context.h"
#include "app_timing_alerts.h"

namespace {
void init_io() {
  encoder.attachFullQuad(ENC_B, ENC_A);
  ESP32Encoder::useInternalWeakPullResistors = puType::up;
  encoder.clearCount();

  pinMode(ENC_BTN, INPUT_PULLUP);
  pinMode(TEMP_SNSR, INPUT);
  analogReadResolution(12);
  analogSetPinAttenuation(TEMP_SNSR, ADC_0db);
  pinMode(LOADONOFF, INPUT_PULLUP);
  pinMode(FAN_CTRL, OUTPUT);
  pinMode(BUZZER, OUTPUT);

#ifdef WOKWI_SIMULATION
  pinMode(VSIM, INPUT);
#endif
}

void init_peripherals() {
  EEPROM.begin(64);
  Serial.begin(115200);
  Wire.begin(21, 22);
  initLCD();
  app_beep_buzzer();
}

void run_peripheral_health_check() {
  app_health_set_ok(true);

#ifndef WOKWI_SIMULATION
  if (dac.begin(0x60)) {
    printLCD(0, 0, F("dac OK"));
    Serial.print("dac OK");
    ads.setGain(GAIN_TWOTHIRDS);
  } else {
    printLCD(0, 0, F("dac NDT"));
    app_health_set_ok(false);
    Serial.print("dac NDT");
  }

  if (ads.begin()) {
    ads.setGain(GAIN_TWOTHIRDS);
    printLCD(0, 1, F("ads OK"));
  } else {
    printLCD(0, 1, F("ads NDT"));
    app_health_set_ok(false);
    Serial.print("ads NDT");
  }
#endif

  if (rtc.begin()) {
    printLCD(8, 0, F("RTC OK"));
    Serial.print("RTC OK");
  } else {
    printLCD(8, 0, F("RTC NDT"));
    app_health_set_ok(false);
    Serial.print("RTC NDT");
  }

  app_measurements_set_temp_c(static_cast<int>(analogRead(TEMP_SNSR) * TEMP_CONVERSION_FACTOR));
  printLCD_S(11, 1, String(app_measurements_temp_c()) + String((char)0xDF) + "C");

  if (app_health_is_ok() && app_measurements_temp_c() <= 99) {
    printLCD(0, 2, F("Sensor Test OK"));
  } else {
    printLCD(0, 2, F("Sensor Test FAIL"));
  }

  delay(2000);
}

void ensure_rtc_running() {
  if (!rtc.isrunning()) {
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }
}

void show_startup_splash() {
  DateTime now = rtc.now();
  String date = String(now.day()) + "/" + String(now.month()) + "/" + String(now.year());
  String time = String(now.hour()) + ":" + String(now.minute()) + ":" + String(now.second());

  clearLCD();
  printLCD(0, 0, F("*DC Electronic Load*"));
  printLCD_S(0, 1, date + " - " + time);
  printLCD(0, 2, F("By Guy & Codex"));

#ifndef WOKWI_SIMULATION
  printLCD(0, 3, F("v2.11"));
  delay(2000);
#else
  printLCD(0, 3, F("v2.11 sim"));
  delay(1000);
#endif
}

void load_runtime_configuration() {
#ifndef WOKWI_SIMULATION
  app_limits_load_from_eeprom();
  app_fan_set_temp_on_c(static_cast<int>(Load_EEPROM(ADD_FAN_TEMP_ON)));
  app_fan_set_hold_ms(static_cast<unsigned long>(Load_EEPROM(ADD_FAN_HOLD_MS)));

  const bool invalidLimits = !app_limits_current_values_are_valid();

  const bool invalidFan =
      app_fan_temp_on_c() < MIN_FAN_TEMP_ON_C || app_fan_temp_on_c() > MAX_FAN_TEMP_ON_C ||
      app_fan_hold_ms() < MIN_FAN_HOLD_MS || app_fan_hold_ms() > MAX_FAN_HOLD_MS;

  if (invalidLimits) {
    legacy_config_limits();
  }

  if (invalidFan) {
    app_fan_set_temp_on_c(DEFAULT_FAN_TEMP_ON_C);
    app_fan_set_hold_ms(DEFAULT_FAN_HOLD_MS);
    Save_EEPROM(ADD_FAN_TEMP_ON, static_cast<float>(app_fan_temp_on_c()));
    Save_EEPROM(ADD_FAN_HOLD_MS, static_cast<float>(app_fan_hold_ms()));
  }
#else
  app_limits_set_current_cutoff(10);
  app_limits_set_power_cutoff(300);
  app_limits_set_temp_cutoff(80);
  app_fan_set_temp_on_c(DEFAULT_FAN_TEMP_ON_C);
  app_fan_set_hold_ms(DEFAULT_FAN_HOLD_MS);
#endif

  Load_Calibration();
  clearLCD();
}
}

void app_startup_run() {
  init_io();
  init_peripherals();
  run_peripheral_health_check();
  ensure_rtc_running();
  show_startup_splash();
  load_runtime_configuration();
}
