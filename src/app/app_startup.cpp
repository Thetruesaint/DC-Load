#include "app_startup.h"

#include <RTClib.h>
#include <Wire.h>

#include "../config/system_constants.h"
#include "../hw/hw_objects.h"
#include "../storage_eeprom.h"
#include "../ui_display.h"
#include "app_fan_context.h"
#include "app_health_context.h"
#include "app_input_buffer.h"
#include "app_limits_bootstrap.h"
#include "app_limits_context.h"
#include "app_measurements_context.h"
#include "app_timing_alerts.h"

namespace {
bool g_startupNeedsLimitsSetup = false;

constexpr uint16_t kUiBg = TFT_BLACK;
constexpr uint16_t kUiAccent = TFT_DARKGREY;
constexpr uint16_t kUiText = TFT_WHITE;
constexpr uint16_t kUiDark = TFT_BLACK;
constexpr uint16_t kUiBorder = TFT_WHITE;
constexpr uint16_t kUiModeAreaBg = TFT_BLUE;
constexpr uint16_t kUiLoadOn = TFT_RED;
constexpr uint16_t kUiLoadOff = TFT_GREEN;
constexpr uint16_t kUiHighlight = TFT_YELLOW;
constexpr uint16_t kUiSetColor = TFT_MAGENTA;

void draw_degree_c_symbol(int x, int y, uint16_t color, uint16_t bg, uint8_t textSize, uint8_t textFont) {
  const int radius = (textSize >= 2) ? 3 : 2;
  uiDisplayDrawCircle(x, y + radius + 1, radius, color);
  uiDisplayPrintStyledAt(x + 5, y - 1, "C", color, bg, textSize, textFont);
}

void show_static_cc_template_preview() {
  const int displayW = uiDisplayWidthPx();
  const int displayH = uiDisplayHeightPx();
  const bool isLargeDisplay = displayW >= 400;
  const int topBarH = (displayH * 10) / 100;
  const int metricsH = (displayH * 20) / 100;
  const int setZoneH = (displayH * 10) / 100;
  const int bottomBarH = (displayH * 10) / 100;
  const int footerY = displayH - bottomBarH;
  const int setZoneY = footerY - setZoneH;
  const int contentY = topBarH + metricsH;
  const int contentH = setZoneY - contentY;
  const int metricsY = topBarH;
  const int metricsBoxX = 0;
  const int metricsBoxY = metricsY;
  const int metricsBoxW = displayW;
  const int metricsBoxH = metricsH;
  const uint8_t barTextFont = 2;
  const uint8_t barTextSize = isLargeDisplay ? 2 : 1;
  const uint8_t metricTextFont = 1;
  const uint8_t sectionTextFont = 2;
  const uint8_t sectionTextSize = isLargeDisplay ? 2 : 1;
  const int topBarTextY = ((topBarH - uiDisplayFontHeight(barTextSize, barTextFont)) / 2) + (isLargeDisplay ? 1 : 0);
  const String metric1 = "10.000a";
  const String metric2 = "32.00v";
  const String metric3 = "320.00w";
  uint8_t metricTextSize = isLargeDisplay ? 4 : 3;
  int metric1W = 0;
  int metric2W = 0;
  int metric3W = 0;
  int metricGap = 0;
  const int metricsMargin = isLargeDisplay ? 12 : 8;
  const int metricsAvailableW = metricsBoxW - (metricsMargin * 2);

  for (; metricTextSize > 0; --metricTextSize) {
    metric1W = uiDisplayTextWidth(metric1, metricTextSize, metricTextFont);
    metric2W = uiDisplayTextWidth(metric2, metricTextSize, metricTextFont);
    metric3W = uiDisplayTextWidth(metric3, metricTextSize, metricTextFont);
    metricGap = isLargeDisplay ? uiDisplayTextWidth(" ", metricTextSize, metricTextFont)
                               : (uiDisplayTextWidth(" ", metricTextSize, metricTextFont) / 2);

    const int totalW = metric1W + metric2W + metric3W + (metricGap * 2);
    if (totalW <= metricsAvailableW || metricTextSize == 1) {
      break;
    }
  }

  const int metricsTotalW = metric1W + metric2W + metric3W + (metricGap * 2);
  const int metricsStartX = (displayW - metricsTotalW) / 2;
  const int metricsTextY = metricsBoxY + ((metricsBoxH - uiDisplayFontHeight(metricTextSize, metricTextFont)) / 2) - (isLargeDisplay ? 2 : 0);
  const int setTextY = setZoneY + ((setZoneH - uiDisplayFontHeight(sectionTextSize, sectionTextFont)) / 2);
  const int contentLabelY = contentY + (contentH / 3);
  const String footerVersion = "v2.12";
  const String footerDateTime = "17/03/26 12:34:56";
  const uint8_t footerTextFont = 2;
  const uint8_t footerTextSize = isLargeDisplay ? 2 : 1;
  const int footerTextY = footerY + ((bottomBarH - uiDisplayFontHeight(footerTextSize, footerTextFont)) / 2);

  uiDisplayClear();

  uiDisplayFillRect(0, 0, displayW, topBarH, kUiAccent);
  uiDisplayPrintStyledAt(displayW / 50, topBarTextY, "CC", kUiHighlight, kUiAccent, barTextSize, barTextFont);

  const int indicatorRadius = isLargeDisplay ? 10 : ((topBarH >= 30) ? 8 : 6);
  const int indicatorX = (displayW / 6) + 9;
  const int indicatorY = topBarH / 2;
  uiDisplayFillCircle(indicatorX, indicatorY, indicatorRadius, kUiLoadOn);
  uiDisplayDrawCircle(indicatorX, indicatorY, indicatorRadius, kUiDark);
  uiDisplayPrintStyledAt(indicatorX + indicatorRadius + 8, topBarTextY, "ON", kUiHighlight, kUiAccent, barTextSize, barTextFont);

  const int tempBlockX = displayW - (isLargeDisplay ? 86 : 38);
  uiDisplayPrintStyledAt(tempBlockX, topBarTextY, "99", kUiHighlight, kUiAccent, barTextSize, barTextFont);
  draw_degree_c_symbol(tempBlockX + uiDisplayTextWidth("99", barTextSize, barTextFont) + (isLargeDisplay ? 8 : 6), topBarTextY + (isLargeDisplay ? 3 : 1), kUiHighlight, kUiAccent, barTextSize, barTextFont);

  uiDisplayDrawRect(metricsBoxX, metricsBoxY, metricsBoxW, metricsBoxH, kUiAccent);
  uiDisplayPrintStyledAt(metricsStartX - uiDisplayTextWidth(" ", metricTextSize, metricTextFont), metricsTextY, metric1, kUiText, kUiBg, metricTextSize, metricTextFont);
  uiDisplayPrintStyledAt(metricsStartX + metric1W + metricGap, metricsTextY, metric2, kUiText, kUiBg, metricTextSize, metricTextFont);
  uiDisplayPrintStyledAt(metricsStartX + metric1W + metricGap + metric2W + metricGap + uiDisplayTextWidth(" ", metricTextSize, metricTextFont), metricsTextY, metric3, kUiText, kUiBg, metricTextSize, metricTextFont);

  uiDisplayDrawRect(0, setZoneY, displayW, setZoneH, kUiAccent);
  uiDisplayPrintStyledAt(displayW / 50, setTextY, "SET> 10.000A", kUiSetColor, kUiBg, sectionTextSize, sectionTextFont);

  uiDisplayDrawRect(0, contentY, displayW, contentH, kUiAccent);
  uiDisplayFillRect(1, contentY + 1, displayW - 2, contentH - 2, kUiModeAreaBg);
  uiDisplayDrawRect(0, contentY, displayW, contentH, kUiAccent);
  uiDisplayPrintStyledAt(displayW / 50, contentLabelY, "MODE AREA / CC SPECIFIC INFO", kUiAccent, kUiBg, sectionTextSize, sectionTextFont);
  uiDisplayPrintStyledAt(displayW / 50, contentLabelY, "MODE AREA / CC SPECIFIC INFO", kUiAccent, kUiModeAreaBg, sectionTextSize, sectionTextFont);
  uiDisplayPrintStyledAt(displayW / 50, contentLabelY + uiDisplayFontHeight(sectionTextSize, sectionTextFont) + 4, "Static mock preview - press E", kUiText, kUiModeAreaBg, sectionTextSize, sectionTextFont);

  uiDisplayFillRect(0, footerY, displayW, bottomBarH, kUiAccent);
  uiDisplayPrintStyledAt(4, footerTextY, footerVersion, kUiText, kUiAccent, footerTextSize, footerTextFont);
  uiDisplayPrintStyledAt(displayW - uiDisplayTextWidth(footerDateTime, footerTextSize, footerTextFont) - 4, footerTextY, footerDateTime, kUiText, kUiAccent, footerTextSize, footerTextFont);

  while (true) {
    const char key = app_input_read_key();
    if (!app_input_is_no_key(key) && key == 'E') {
      break;
    }
    delay(25);
  }

  uiDisplayClear();
}

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
  show_static_cc_template_preview();
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
