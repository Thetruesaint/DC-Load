#include "app_ota.h"
#include "ui_display.h"

#include <Arduino.h>
#include <WiFi.h>
#include <ArduinoOTA.h>
#include <cstdio>
#include <cstring>

#if __has_include("app_ota_secrets.local.h")
#include "app_ota_secrets.local.h"
#else
#define APP_OTA_WIFI_SSID ""
#define APP_OTA_WIFI_PASSWORD ""
#define APP_OTA_HOSTNAME "DC_LOAD"
#endif

namespace {
constexpr unsigned long WIFI_CONNECT_TIMEOUT_MS = 15000UL;

bool g_active = false;
bool g_otaBegun = false;
bool g_uploading = false;
bool g_error = false;
unsigned long g_wifiStartMs = 0;
char g_status[21] = "Inactive";
char g_detail[21] = "";
char g_hint[21] = "";

void refresh_ota_ui_if_active() {
  if (!g_active) return;
  uiDisplayRenderFwUpdateScreen(g_status, g_detail, g_hint);
}

void set_texts(const char *status, const char *detail, const char *hint) {
  std::snprintf(g_status, sizeof(g_status), "%s", status);
  std::snprintf(g_detail, sizeof(g_detail), "%s", detail);
  std::snprintf(g_hint, sizeof(g_hint), "%s", hint);
  refresh_ota_ui_if_active();
}

void begin_wifi_connect() {
  WiFi.mode(WIFI_STA);
  WiFi.setHostname(APP_OTA_HOSTNAME);
  WiFi.begin(APP_OTA_WIFI_SSID, APP_OTA_WIFI_PASSWORD);
  g_wifiStartMs = millis();
  g_active = true;
  g_otaBegun = false;
  g_uploading = false;
  g_error = false;
  set_texts("Connecting WiFi...", "Host: DC_LOAD", "CLR-Cancel");
}

void ensure_ota_started() {
  if (g_otaBegun) return;

  ArduinoOTA.setHostname(APP_OTA_HOSTNAME);
  ArduinoOTA.onStart([]() {
    g_uploading = true;
    g_error = false;
    set_texts("Uploading...", "0%", "Wait...");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    char detail[21];
    const unsigned int percent = (total == 0U) ? 0U : static_cast<unsigned int>((progress * 100U) / total);
    std::snprintf(detail, sizeof(detail), "%u%%", percent);
    set_texts("Uploading...", detail, "Wait...");
  });
  ArduinoOTA.onEnd([]() {
    set_texts("Update complete", "Rebooting...", "");
  });
  ArduinoOTA.onError([](ota_error_t error) {
    char detail[21];
    g_uploading = false;
    g_error = true;
    std::snprintf(detail, sizeof(detail), "Err %d", static_cast<int>(error));
    set_texts("Upload failed", detail, "E-Retry   CLR-Cancel");
  });
  ArduinoOTA.begin();
  g_otaBegun = true;

  char ipLine[21];
  std::snprintf(ipLine, sizeof(ipLine), "IP: %s", WiFi.localIP().toString().c_str());
  set_texts(ipLine, "Host: DC_LOAD", "CLR-Cancel");
}
}

void app_ota_begin() {
#ifdef WOKWI_SIMULATION
  g_active = true;
  g_otaBegun = false;
  g_uploading = false;
  g_error = true;
  set_texts("OTA not in sim", "", "CLR-Cancel");
  return;
#endif
  if (g_active) return;
  if (std::strlen(APP_OTA_WIFI_SSID) == 0U) {
    g_active = true;
    g_error = true;
    set_texts("OTA not config'd", "", "CLR-Cancel");
    return;
  }
  begin_wifi_connect();
}

void app_ota_handle() {
#ifdef WOKWI_SIMULATION
  return;
#endif
  if (!g_active) return;

  if (WiFi.status() != WL_CONNECTED) {
    if ((millis() - g_wifiStartMs) >= WIFI_CONNECT_TIMEOUT_MS) {
      g_error = true;
      g_uploading = false;
      set_texts("WiFi failed", "", "E-Retry   CLR-Cancel");
    }
    return;
  }

  ensure_ota_started();
  ArduinoOTA.handle();
}

void app_ota_stop() {
  if (g_uploading) return;
  g_active = false;
  g_otaBegun = false;
  g_uploading = false;
  g_error = false;
#ifndef WOKWI_SIMULATION
  WiFi.disconnect(true, true);
  WiFi.mode(WIFI_OFF);
#endif
  set_texts("Inactive", "", "");
}

void app_ota_restart() {
  app_ota_stop();
  app_ota_begin();
}

bool app_ota_is_active() { return g_active; }
bool app_ota_is_uploading() { return g_uploading; }
bool app_ota_has_error() { return g_error; }
const char *app_ota_status_text() { return g_status; }
const char *app_ota_detail_text() { return g_detail; }
const char *app_ota_hint_text() { return g_hint; }
