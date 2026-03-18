#include "app_startup.h"

#include <RTClib.h>
#include <Wire.h>

#include "../config/system_constants.h"
#include "../hw/hw_objects.h"
#include "../storage_eeprom.h"
#include "../ui_display.h"
#include "app_fan_context.h"
#include "app_health_context.h"
#include "app_limits_bootstrap.h"
#include "app_limits_context.h"
#include "app_measurements_context.h"
#include "app_timing_alerts.h"

namespace {
bool g_startupNeedsLimitsSetup = false;

void init_io() {
  encoder.attachFullQuad(ENC_B, ENC_A);
  ESP32Encoder::useInternalWeakPullResistors = puType::up;
  encoder.clearCount();

  pinMode(ENC_BTN, INPUT_PULLUP);
  pinMode(TEMP_SNSR, INPUT);
  analogReadResolution(12);
  analogSetPinAttenuation(TEMP_SNSR, ADC_0db);
  pinMode(LOADONOFF, INPUT);
  pinMode(FAN_CTRL, OUTPUT);
  digitalWrite(FAN_CTRL, LOW);
  pinMode(BUZZER, OUTPUT);
  digitalWrite(BUZZER, LOW);
  pinMode(MOSFONOFF, OUTPUT);
  digitalWrite(MOSFONOFF, HIGH);
}

void init_peripherals() {
  EEPROM.begin(64);
  Serial.begin(115200);
  Wire.begin(21, 22);
  uiDisplayInit();
  app_beep_buzzer();
}

void run_peripheral_health_check() {
  app_health_set_ok(true);

#ifndef WOKWI_SIMULATION
  if (dac.begin(0x60)) {
    uiGridPrint(0, 0, F("dac OK"));
    Serial.print("dac OK");
    ads.setGain(GAIN_TWOTHIRDS);
  } else {
    uiGridPrint(0, 0, F("dac NDT"));
    app_health_set_ok(false);
    Serial.print("dac NDT");
  }

  if (ads.begin()) {
    ads.setGain(GAIN_TWOTHIRDS);
    uiGridPrint(0, 1, F("ads OK"));
  } else {
    uiGridPrint(0, 1, F("ads NDT"));
    app_health_set_ok(false);
    Serial.print("ads NDT");
  }
#endif

  if (rtc.begin()) {
    uiGridPrint(8, 0, F("RTC OK"));
    Serial.print("RTC OK");
  } else {
    uiGridPrint(8, 0, F("RTC NDT"));
    app_health_set_ok(false);
    Serial.print("RTC NDT");
  }

  app_measurements_set_temp_c(static_cast<int>(analogRead(TEMP_SNSR) * TEMP_CONVERSION_FACTOR));
  uiGridPrintString(11, 1, String(app_measurements_temp_c()) + String((char)0xDF) + "C");

  if (app_health_is_ok() && app_measurements_temp_c() <= 99) {
    uiGridPrint(0, 2, F("Sensor Test OK"));
    digitalWrite(MOSFONOFF, LOW);
  } else {
    uiGridPrint(0, 2, F("Sensor Test FAIL"));
    digitalWrite(MOSFONOFF, HIGH);
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

  uiDisplayClear();
  uiGridPrint(0, 0, F("*DC Electronic Load*"));
  uiGridPrintString(0, 1, date + " - " + time);
  uiGridPrint(0, 2, F("By Guy & Codex"));

#ifndef WOKWI_SIMULATION
  uiGridPrint(0, 3, F("v2.12b"));
  delay(2000);
#else
  uiGridPrint(0, 3, F("v2.12b sim"));
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
    g_startupNeedsLimitsSetup = true;
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
  uiDisplayClear();
}
}

void app_startup_run() {
  g_startupNeedsLimitsSetup = false;
  init_io();
  init_peripherals();
  run_peripheral_health_check();
  ensure_rtc_running();
  show_startup_splash();
  load_runtime_configuration();
}

bool app_startup_consume_limits_setup_request() {
  const bool requested = g_startupNeedsLimitsSetup;
  g_startupNeedsLimitsSetup = false;
  return requested;
}
