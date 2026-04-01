#include "app_startup.h"

#include <RTClib.h>
#include <Wire.h>

#include "../config/system_constants.h"
#include "../hw/hw_objects.h"
#include "../storage_eeprom.h"
#include "../ui_display.h"
#include "app_fan_context.h"
#include "app_health_context.h"
#include "app_limits_storage.h"
#include "app_limits_context.h"
#include "app_load_output.h"
#include "app_measurements_context.h"
#include "app_timing_alerts.h"

namespace {
bool g_startupNeedsLimitsSetup = false;
bool g_rtcDetected = false;

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
  app_load_output_set_dac_ready(false);
  app_load_output_emergency_off();
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
  bool dacDetected = true;
  bool adsDetected = true;
  bool sensorOk = false;

#ifndef WOKWI_SIMULATION
  if (dac.begin(0x60)) {
    app_load_output_set_dac_ready(true);
    Serial.print("DAC detected");
    ads.setGain(GAIN_TWOTHIRDS);
  } else {
    app_load_output_set_dac_ready(false);
    dacDetected = false;
    app_health_set_ok(false);
    Serial.print("DAC not detected");
  }

  if (ads.begin()) {
    ads.setGain(GAIN_TWOTHIRDS);
    Serial.print(" ADS detected");
  } else {
    adsDetected = false;
    app_health_set_ok(false);
    Serial.print(" ADS not detected");
  }
#endif

  g_rtcDetected = rtc.begin();
  if (g_rtcDetected) {
    Serial.print(" RTC detected");
  } else {
    app_health_set_ok(false);
    Serial.print(" RTC not detected");
  }

  app_measurements_set_temp_c(app_measurements_read_temp_c());
  sensorOk = app_health_is_ok() && app_measurements_temp_c() <= 99;

  if (sensorOk) {
    digitalWrite(MOSFONOFF, LOW);
  } else {
    app_load_output_emergency_off();
  }

  uiDisplayRenderStartupHealthCheck(dacDetected, adsDetected, g_rtcDetected, app_measurements_temp_c(), sensorOk);

  if (!sensorOk) {
    while (true) {
      delay(50);
    }
  }

  delay(2000);
}

void ensure_rtc_running() {
  if (g_rtcDetected && !rtc.isrunning()) {
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }
}

void show_startup_splash() {
  app_measurements_set_temp_c(app_measurements_read_temp_c());
  uiDisplayRenderStartupSplash(false, app_measurements_temp_c());

#ifndef WOKWI_SIMULATION
  delay(2000);
#else
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
  g_rtcDetected = false;
  init_io();
  init_peripherals();
  show_startup_splash();
  run_peripheral_health_check();
  ensure_rtc_running();
  load_runtime_configuration();
}

bool app_startup_consume_limits_setup_request() {
  const bool requested = g_startupNeedsLimitsSetup;
  g_startupNeedsLimitsSetup = false;
  return requested;
}
