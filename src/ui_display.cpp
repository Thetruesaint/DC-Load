#include "ui_display.h"

#include "app/app_input_buffer.h"
#include "app/app_calibration_context.h"
#include "app/app_fan_context.h"
#include "app/app_limits_context.h"
#include "app/app_measurements_context.h"
#include "app/app_msc.h"
#include "app/app_load_context.h"
#include "app/app_mode_state_context.h"
#include "app/app_setpoint_context.h"
#include "app/app_ota.h"
#include "app/app_trace_context.h"
#include "app/app_timing_alerts.h"
#include "app/app_ui_context.h"
#include "config/system_constants.h"
#include "hw/hw_objects.h"
#include "ui/ui_view_state.h"
#include "ui/ui_state_cache.h"
#include "ui/ui_state_machine.h"

#include <cstring>

namespace {
constexpr uint16_t TFT_TEXT_COLOR = TFT_CYAN;
constexpr uint16_t TFT_BG_COLOR = TFT_BLACK;
constexpr uint16_t kUiBg = TFT_BLACK;
constexpr uint16_t kUiAccent = TFT_DARKGREY;
constexpr uint16_t kUiBorder = TFT_WHITE;
constexpr uint16_t kUiText = TFT_WHITE;
constexpr uint16_t kUiDark = TFT_BLACK;
constexpr uint16_t kUiModeAreaBg = TFT_BLUE;
constexpr uint16_t kUiLoadOn = TFT_RED;
constexpr uint16_t kUiLoadOff = TFT_GREEN;
constexpr uint16_t kUiHighlight = TFT_YELLOW;
constexpr uint16_t kUiSetColor = TFT_MAGENTA;
constexpr uint16_t kUiAlertBg = TFT_RED;
constexpr uint16_t kUiAlertText = TFT_WHITE;
constexpr const char *kFirmwareVersion = "v2.14b";

int footer_bar_height_px(int displayH) {
  return max(16, (displayH * 8) / 100);
}

constexpr uint8_t kFooterTextFont = 1;
#ifdef WOKWI_SIMULATION
constexpr uint8_t kFooterTextSize = 1;
#else
constexpr uint8_t kFooterTextSize = 2;
#endif

void draw_horizontal_separator(int y, int width, uint16_t color) {
  tft.drawLine(0, y, width - 1, y, color);
}

bool g_ccLayoutDrawn = false;
int g_ccLastDisplayW = 0;
int g_ccLastDisplayH = 0;
bool g_ccLastLoadEnabled = false;
uint8_t g_ccLastMode = 0xFF;
bool g_ccLastCursorVisible = true;
int g_ccLastCursorPosition = -1;
String g_ccLastMetric1;
String g_ccLastMetric2;
String g_ccLastMetric3;
String g_ccLastSetText;
String g_ccLastFooterText;
String g_ccLastTempText;
String g_ccLastModeLine1;
String g_ccLastModeLine2;
String g_ccLastModeLine3;
String g_ccLastModeLine4;
String g_ccLastModeLine5;
String g_ccLastBatteryStatus;
bool g_ccLastShiftActive = false;
uint32_t g_ccLastTraceToken = 0;
size_t g_ccLastTraceSampleCount = 0;
float g_ccLastTraceTimeScaleSeconds = 0.0f;
float g_ccLastTraceCurrentScaleMax = 0.0f;
float g_ccLastTraceVoltageScaleMax = 0.0f;
bool g_ccLastTraceOverlayActive = false;
uint16_t g_ccLastBatteryStatusColor = 0;
bool g_setupMetricsValid = false;
int g_setupMetricsDisplayW = 0;
int g_setupMetricsDisplayH = 0;
uint8_t g_setupMetricsMode = 0xFF;
String g_setupLastMetric1;
String g_setupLastMetric2;
String g_setupLastMetric3;
bool g_fwUpdateLayoutDrawn = false;
int g_fwUpdateLastDisplayW = 0;
int g_fwUpdateLastDisplayH = 0;
String g_fwUpdateLastStatus;
String g_fwUpdateLastDetail;
String g_fwUpdateLastHint;

struct DisplayZones {
  int displayW;
  int displayH;
  int topBarH;
  int metricsH;
  int setZoneH;
  int bottomBarH;
  int footerY;
  int setZoneY;
  int contentY;
  int contentH;
  bool isLargeDisplay;
};

using ManagedZoneLayout = DisplayZones;

struct TopStatusZoneState {
  String label;
  bool loadEnabled;
  float tempC;
  float fanTempOnC;
  float tempCutOffC;
};

struct MetricsZoneState {
  String metric1;
  String metric2;
  String metric3;
};

struct MetricsZoneRenderLayout {
  uint8_t textFont;
  uint8_t textSize;
  int metric1W;
  int metric2W;
  int metric3W;
  int metricGap;
  int metricsTextY;
  int metricPadding;
  int metric1X;
  int metric2X;
  int metric3X;
  int metricAreaY;
  int metricAreaH;
};

struct InputZoneRenderLayout {
  uint8_t textFont;
  uint8_t textSize;
  int textX;
  int textY;
  int shiftX;
};

struct ContentTextPanelLayout {
  uint8_t titleFont;
  uint8_t titleSize;
  uint8_t textFont;
  uint8_t textSize;
  int titleH;
  int textH;
  int verticalGap;
  int titleY;
  int row1Y;
  int row2Y;
  int row3Y;
};

struct TracePlotLayout {
  int leftPad;
  int rightPad;
  int topPad;
  int bottomPad;
  int plotX;
  int plotY;
  int plotW;
  int plotH;
};

struct FooterZoneState {
  String leftText;
  String rightText;
};

ManagedZoneLayout managed_zone_layout();
void draw_zone_borders(const ManagedZoneLayout &layout);
MetricsZoneState metrics_zone_stub_state();
TopStatusZoneState current_top_status_zone_state(const String &label, bool loadEnabled);
FooterZoneState current_footer_zone_state();
void draw_footer_zone(const ManagedZoneLayout &layout, const FooterZoneState &state);
void draw_top_status_zone(const ManagedZoneLayout &layout, const TopStatusZoneState &state);
void draw_metrics_zone(const ManagedZoneLayout &layout, const MetricsZoneState &state);
MetricsZoneRenderLayout compute_metrics_zone_render_layout(const ManagedZoneLayout &layout, const MetricsZoneState &state);
void update_metrics_zone_values(const ManagedZoneLayout &layout,
                                const MetricsZoneState &state,
                                bool layoutChanged,
                                String &lastMetric1,
                                String &lastMetric2,
                                String &lastMetric3);
InputZoneRenderLayout compute_input_zone_render_layout(const ManagedZoneLayout &layout);
void clear_input_zone(const ManagedZoneLayout &layout);
void draw_input_zone_value(const ManagedZoneLayout &layout, const String &prefix, const String &value);
void draw_input_zone_shift_hint(const ManagedZoneLayout &layout);
int draw_input_zone_highlighted_value(const ManagedZoneLayout &layout,
                                      int startX,
                                      const String &valueText,
                                      int highlightedIndex,
                                      bool cursorVisible);
ContentTextPanelLayout compute_content_text_panel_layout(const ManagedZoneLayout &layout);
int centered_content_text_x(const ManagedZoneLayout &layout, const String &text, uint8_t textSize, uint8_t textFont);
void clear_blue_content_zone(const ManagedZoneLayout &layout);
void draw_content_panel_title(const ManagedZoneLayout &layout, const ContentTextPanelLayout &panel, const String &title);
void draw_content_stage_line(const ContentTextPanelLayout &panel, const String &text, int x, int y, bool selected);
void render_bc_mode_content(const ManagedZoneLayout &layout,
                            bool refreshAll,
                            const String &title,
                            const String &statusLabel,
                            const String &statusValue,
                            uint16_t statusColor,
                            const String &row2Left,
                            const String &row2Right,
                            const String &row3Left,
                            const String &row3Right);
void render_transient_mode_content(const ManagedZoneLayout &layout,
                                   bool refreshAll,
                                   const String &title,
                                   const String &row1,
                                   const String &row2Left,
                                   const String &row2Right,
                                   const String &row3);
void render_calibration_mode_content(const ManagedZoneLayout &layout,
                                     bool refreshAll,
                                     const String &title,
                                     const String &row1,
                                     const String &row2,
                                     const String &row3);
TracePlotLayout compute_trace_plot_layout(const ManagedZoneLayout &layout);
void update_trace_overlay_content(const ManagedZoneLayout &layout, bool layoutChanged, bool traceOverlayChanged, uint32_t traceToken);

void restore_tft_text_style() {
  tft.setTextColor(TFT_TEXT_COLOR, TFT_BG_COLOR);
  tft.setTextFont(1);
  tft.setTextSize(1);
}

void draw_centered_load_status(int displayW, int topBarH, bool enabled, bool isLargeDisplay, uint8_t textSize, uint8_t textFont) {
  const int indicatorRadius = isLargeDisplay ? 10 : ((topBarH >= 30) ? 8 : 6);
  const int indicatorX = (displayW / 2) - 26;
  const int indicatorY = topBarH / 2;
  const int textY = ((topBarH - uiDisplayFontHeight(textSize, textFont)) / 2) + (isLargeDisplay ? 1 : 0);
  const uint16_t loadColor = enabled ? kUiLoadOn : kUiLoadOff;
  const uint16_t statusTextColor = enabled ? kUiText : kUiHighlight;

  uiDisplayFillCircle(indicatorX, indicatorY, indicatorRadius, loadColor);
  uiDisplayDrawCircle(indicatorX, indicatorY, indicatorRadius, kUiDark);
  uiDisplayPrintStyledAt(indicatorX + indicatorRadius + 8,
                         textY,
                         enabled ? "ON" : "OFF",
                         statusTextColor,
                         kUiAccent,
                         textSize,
                         textFont);
}

uint16_t temperature_status_color(float tempC, float fanOnC, float tempCutoffC) {
  const float safeFanOn = max(0.0f, fanOnC);
  const float safeCutoff = max(safeFanOn, tempCutoffC);

  if (tempC <= safeFanOn) return kUiText;
  if (tempC >= safeCutoff) return kUiAlertBg;

  const float range = max(0.1f, safeCutoff - safeFanOn);
  const float progress = constrain((tempC - safeFanOn) / range, 0.0f, 1.0f);

  const uint8_t red = 255;
  const uint8_t green = static_cast<uint8_t>(255.0f * (1.0f - progress));
  const uint8_t blue = 0;
  return tft.color565(red, green, blue);
}

String format_fixed(float value, int decimals) {
  return String(value, decimals);
}

float quantize_display_value(float value, float step) {
  if (step <= 0.0f) return value;
  return roundf(value / step) * step;
}

String format_measured_current_raw(float currentA) {
  return format_fixed(currentA, (currentA <= 9.999f) ? 3 : 2) + "a";
}

String format_measured_voltage_raw(float voltageV) {
  const int decimals = (voltageV <= 9.999f) ? 3 : (voltageV <= 99.99f) ? 2 : 1;
  return format_fixed(voltageV, decimals) + "v";
}

String format_measured_power_raw(float powerW) {
  const int decimals = (powerW < 100.0f) ? 2 : 1;
  return format_fixed(powerW, decimals) + "w";
}

String format_measured_current_display(float currentA) {
  currentA = quantize_display_value(currentA, (currentA <= 9.999f) ? 0.002f : 0.01f);
  return format_measured_current_raw(currentA);
}

String format_measured_voltage_display(float voltageV) {
  voltageV = quantize_display_value(voltageV, (voltageV <= 9.999f) ? 0.005f : (voltageV <= 99.99f) ? 0.01f : 0.1f);
  return format_measured_voltage_raw(voltageV);
}

String format_measured_power_display(float powerW) {
  powerW = quantize_display_value(powerW, (powerW < 100.0f) ? 0.05f : 0.1f);
  return format_measured_power_raw(powerW);
}

float trace_time_scale_seconds(float elapsedSeconds) {
  constexpr float kTraceTimeScales[] = {
      5.0f,   10.0f,   20.0f,   40.0f,   60.0f,   120.0f,
      300.0f, 600.0f,  1200.0f, 2400.0f, 3600.0f};

  float timeScaleSeconds = kTraceTimeScales[0];
  for (float candidate : kTraceTimeScales) {
    timeScaleSeconds = candidate;
    if (elapsedSeconds < (candidate * 0.8f)) {
      break;
    }
  }
  return timeScaleSeconds;
}

String trace_time_legend(float timeScaleSeconds) {
  if (timeScaleSeconds < 60.0f) {
    return String(static_cast<int>(timeScaleSeconds)) + "s";
  }
  const int minutes = static_cast<int>(timeScaleSeconds / 60.0f);
  if (minutes < 60) {
    return String(minutes) + "m";
  }
  return "1h";
}

int trace_time_tick_step_seconds(float timeScaleSeconds) {
  if (timeScaleSeconds <= 40.0f) return 1;
  if (timeScaleSeconds <= 60.0f) return 5;
  if (timeScaleSeconds <= 120.0f) return 10;
  if (timeScaleSeconds <= 300.0f) return 30;
  if (timeScaleSeconds <= 600.0f) return 60;
  if (timeScaleSeconds <= 1200.0f) return 120;
  if (timeScaleSeconds <= 2400.0f) return 300;
  return 600;
}

float trace_current_scale_max(float maxCurrentA) {
  float currentScaleMax = 1.0f;
  while (currentScaleMax < 10.0f && maxCurrentA > currentScaleMax) {
    currentScaleMax += 1.0f;
  }
  return constrain(currentScaleMax, 1.0f, 10.0f);
}

float trace_voltage_scale_max(float maxVoltageV) {
  if (maxVoltageV > 1.0f && maxVoltageV <= 5.0f) {
    return 5.0f;
  }
  if (maxVoltageV > 5.0f && maxVoltageV <= 10.0f) {
    return 10.0f;
  }
  if (maxVoltageV > 10.0f && maxVoltageV <= 15.0f) {
    return 15.0f;
  }
  if (maxVoltageV > 15.0f && maxVoltageV <= 20.0f) {
    return 20.0f;
  }
  if (maxVoltageV > 20.0f && maxVoltageV <= 25.0f) {
    return 25.0f;
  }
  if (maxVoltageV > 25.0f && maxVoltageV <= 30.0f) {
    return 30.0f;
  }
  if (maxVoltageV > 30.0f) {
    return 35.0f;
  }
  return 1.0f;
}

void draw_cc_trace_segment_range(int plotX,
                                 int plotY,
                                 int plotW,
                                 int plotH,
                                 unsigned long sampleIntervalMs,
                                 float timeScaleSeconds,
                                 float currentScaleMax,
                                 float voltageScaleMax,
                                 uint16_t currentColor,
                                 uint16_t voltageColor,
                                 size_t startIndex,
                                 size_t endIndex) {
  if (endIndex == 0 || startIndex > endIndex) return;

  float prevCurrentA = 0.0f;
  float prevVoltageV = 0.0f;
  if (!app_trace_read_sample(startIndex, &prevCurrentA, &prevVoltageV)) return;

  const float prevTime = static_cast<float>(startIndex * sampleIntervalMs) / 1000.0f;
  int prevX = plotX + static_cast<int>(constrain(prevTime / timeScaleSeconds, 0.0f, 1.0f) * static_cast<float>(plotW));
  int prevCurrentY = plotY + plotH - static_cast<int>((prevCurrentA / currentScaleMax) * static_cast<float>(plotH));
  int prevVoltageY = plotY + plotH - static_cast<int>((prevVoltageV / voltageScaleMax) * static_cast<float>(plotH));

  if (startIndex == endIndex) {
    uiDisplayFillCircle(prevX, prevCurrentY, 2, currentColor);
    uiDisplayFillCircle(prevX, prevVoltageY, 2, voltageColor);
    return;
  }

  for (size_t i = startIndex + 1; i <= endIndex; ++i) {
    float currentA = 0.0f;
    float voltageV = 0.0f;
    if (!app_trace_read_sample(i, &currentA, &voltageV)) continue;
    const float sampleTime = static_cast<float>(i * sampleIntervalMs) / 1000.0f;
    const int sampleX = plotX + static_cast<int>(constrain(sampleTime / timeScaleSeconds, 0.0f, 1.0f) * static_cast<float>(plotW));
    const int currentY = plotY + plotH - static_cast<int>((currentA / currentScaleMax) * static_cast<float>(plotH));
    const int voltageY = plotY + plotH - static_cast<int>((voltageV / voltageScaleMax) * static_cast<float>(plotH));
    tft.drawLine(prevX, prevCurrentY, sampleX, currentY, currentColor);
    tft.drawLine(prevX, prevVoltageY, sampleX, voltageY, voltageColor);
    prevX = sampleX;
    prevCurrentY = currentY;
    prevVoltageY = voltageY;
  }
}

void draw_cc_trace_graph(int x, int y, int w, int h, bool isLargeDisplay) {
  const uint16_t panelBg = kUiAccent;
  const uint16_t axisColor = TFT_WHITE;
  const uint16_t currentColor = tft.color565(0, 180, 0);
  const uint16_t voltageColor = TFT_RED;
  const uint16_t textBg = panelBg;
  const uint8_t textFont = 1;
#ifdef WOKWI_SIMULATION
  const uint8_t textSize = 1;
#else
  const uint8_t textSize = 2;
#endif
  const size_t sampleCount = app_trace_sample_count();
  const unsigned long sampleIntervalMs = app_trace_effective_interval_ms();
  const int leftPad = isLargeDisplay ? 24 : 18;
  const int rightPad = leftPad;
  const int topPad = isLargeDisplay ? 18 : 14;
  const int bottomPad = isLargeDisplay ? 18 : 14;
  const int plotX = x + leftPad;
  const int plotY = y + topPad;
  const int plotW = max(10, w - leftPad - rightPad);
  const int plotH = max(10, h - topPad - bottomPad);

  const int panelInsetX = isLargeDisplay ? 8 : 6;
  const int panelInsetY = isLargeDisplay ? 6 : 4;
  const int panelX = x + panelInsetX;
  const int panelY = y + panelInsetY;
  const int panelW = max(8, w - (panelInsetX * 2));
  const int panelH = max(8, h - (panelInsetY * 2));

  uiDisplayFillRect(x, y, w, h, kUiModeAreaBg);
  uiDisplayFillRect(panelX, panelY, panelW, panelH, panelBg);
  uiDisplayDrawRect(panelX, panelY, panelW, panelH, kUiBorder);
  tft.drawLine(plotX, plotY, plotX, plotY + plotH, axisColor);
  tft.drawLine(plotX - (isLargeDisplay ? 4 : 3), plotY + plotH, plotX + plotW, plotY + plotH, axisColor);
  tft.drawLine(plotX - (isLargeDisplay ? 4 : 3), plotY, plotX + (isLargeDisplay ? 4 : 3), plotY, axisColor);

  float maxCurrentA = 0.0f;
  float maxVoltageV = 0.0f;
  float currentA = 0.0f;
  float voltageV = 0.0f;
  for (size_t i = 0; i < sampleCount; ++i) {
    if (!app_trace_read_sample(i, &currentA, &voltageV)) continue;
    maxCurrentA = max(maxCurrentA, currentA);
    maxVoltageV = max(maxVoltageV, voltageV);
  }
  const float elapsedSeconds = app_trace_duration_seconds();
  const float timeScaleSeconds = trace_time_scale_seconds(elapsedSeconds);
  const float currentScaleMax = trace_current_scale_max(maxCurrentA);
  const float voltageScaleMax = trace_voltage_scale_max(maxVoltageV);

  const String currentLegend = "Imax:" + String(currentScaleMax, currentScaleMax < 10.0f ? 0 : 0);
  const String voltageLegend = "Vmax:" + String(voltageScaleMax, voltageScaleMax < 10.0f ? 0 : 0);
  const String timeLegend = trace_time_legend(timeScaleSeconds);
  const float currentTickStep = (currentScaleMax <= 1.0f) ? 0.1f : (currentScaleMax <= 2.0f ? 0.5f : 1.0f);
  const float voltageTickStep = (voltageScaleMax <= 1.0f) ? 0.5f : ((voltageScaleMax <= 10.0f) ? 1.0f : (voltageScaleMax <= 20.0f ? 2.0f : 5.0f));

  const int legendY = y + 8;
  const int legendStartX = panelX + (panelW / 2) - uiDisplayTextWidth(currentLegend + "   " + voltageLegend, textSize, textFont) / 2;
  uiDisplayPrintStyledAt(legendStartX,
                         legendY,
                         currentLegend,
                         currentColor,
                         textBg,
                         textSize,
                         textFont);
  uiDisplayPrintStyledAt(legendStartX + uiDisplayTextWidth(currentLegend + "   ", textSize, textFont),
                         legendY,
                         voltageLegend,
                         voltageColor,
                         textBg,
                         textSize,
                         textFont);
  uiDisplayPrintStyledAt(plotX + plotW - uiDisplayTextWidth(timeLegend, textSize, textFont),
                         plotY + plotH - uiDisplayFontHeight(textSize, textFont) - (isLargeDisplay ? 2 : 1),
                         timeLegend,
                         axisColor,
                         textBg,
                         textSize,
                         textFont);

  for (float tick = currentTickStep; tick < currentScaleMax; tick += currentTickStep) {
    const int tickY = plotY + plotH - static_cast<int>((tick / currentScaleMax) * static_cast<float>(plotH));
    const bool majorTick = fabsf(fmodf(tick, 1.0f)) < 0.01f;
    const int tickLen = majorTick ? (isLargeDisplay ? 6 : 4) : (isLargeDisplay ? 4 : 3);
    tft.drawLine(plotX - tickLen, tickY, plotX - 1, tickY, currentColor);
  }

  for (float tick = voltageTickStep; tick < voltageScaleMax; tick += voltageTickStep) {
    const int tickY = plotY + plotH - static_cast<int>((tick / voltageScaleMax) * static_cast<float>(plotH));
    const bool majorTick = fabsf(fmodf(tick, 1.0f)) < 0.01f;
    const int tickLen = majorTick ? (isLargeDisplay ? 6 : 4) : (isLargeDisplay ? 4 : 3);
    tft.drawLine(plotX + 1, tickY, plotX + tickLen, tickY, voltageColor);
  }

  const int timeTickStep = trace_time_tick_step_seconds(timeScaleSeconds);
  int lastTickX = -1;
  for (int second = 0; second <= static_cast<int>(timeScaleSeconds); second += timeTickStep) {
    const int tickX = plotX + static_cast<int>((static_cast<float>(second) / timeScaleSeconds) * static_cast<float>(plotW));
    if (tickX == lastTickX) continue;
    const bool majorTick = (timeTickStep < 60) ? ((second % (timeTickStep * 5)) == 0) : ((second % (timeTickStep * 2)) == 0);
    const int tickH = majorTick ? (isLargeDisplay ? 6 : 4) : (isLargeDisplay ? 4 : 3);
    tft.drawLine(tickX, plotY + plotH, tickX, plotY + plotH + tickH, axisColor);
    lastTickX = tickX;
  }

  if (sampleCount > 0) {
    draw_cc_trace_segment_range(plotX,
                                plotY,
                                plotW,
                                plotH,
                                sampleIntervalMs,
                                timeScaleSeconds,
                                currentScaleMax,
                                voltageScaleMax,
                                currentColor,
                                voltageColor,
                                0,
                                sampleCount - 1);
  }

  g_ccLastTraceSampleCount = sampleCount;
  g_ccLastTraceTimeScaleSeconds = timeScaleSeconds;
  g_ccLastTraceCurrentScaleMax = currentScaleMax;
  g_ccLastTraceVoltageScaleMax = voltageScaleMax;
}

String format_cc_setpoint(float readingValue) {
  return String("SET> ") + String(readingValue, 3) + "A";
}

String format_cp_value(float readingValue) {
  String valueText = String(readingValue, 1);
  if (readingValue < 100.0f) valueText = "0" + valueText;
  if (readingValue < 10.0f) valueText = "0" + valueText;
  return valueText;
}

String format_cr_value(float readingValue) {
  return String(readingValue, 1);
}

String format_tc_period_value(float periodMs) {
  const unsigned long period = static_cast<unsigned long>(constrain(periodMs, 100.0f, 10000.0f));
  char buffer[6];
  snprintf(buffer, sizeof(buffer), "%05lu", period);
  return String(buffer);
}

String format_transient_current(float currentA) {
  return String(currentA, 3) + "A";
}

String format_transient_period_label(float periodMs) {
  return String(static_cast<unsigned long>(max(0.0f, periodMs))) + " mS";
}

const char *mode_label_for_value(uint8_t mode) {
  return (mode == CC) ? "CC" : (mode == CP) ? "CP" : (mode == CR) ? "CR" : (mode == BC) ? "BC" : (mode == TC) ? "TC" : (mode == TL) ? "TL" : "CA";
}

String two_digits(int value) {
  if (value < 10) return String("0") + String(value);
  return String(value);
}

String rtc_timestamp_text() {
  const DateTime now = rtc.now();
  return two_digits(now.day()) + "/" + two_digits(now.month()) + "/" +
         two_digits(now.year() % 100) + " " + two_digits(now.hour()) + ":" +
         two_digits(now.minute());
}

void draw_degree_c_symbol(int x, int y, uint16_t color, uint16_t bg, uint8_t textSize, uint8_t textFont);

struct StartupScreenLayout {
  int displayW;
  int displayH;
  int topBarH;
  int bottomBarH;
  int footerY;
  int contentY;
  int contentH;
  bool isLargeDisplay;
};

StartupScreenLayout startup_screen_layout() {
  const int displayW = uiDisplayWidthPx();
  const int displayH = uiDisplayHeightPx();
  const int topBarH = (displayH * 10) / 100;
  const int bottomBarH = footer_bar_height_px(displayH);
  const int footerY = displayH - bottomBarH;
  return {displayW, displayH, topBarH, bottomBarH, footerY, topBarH, footerY - topBarH, displayW >= 400};
}

void draw_startup_borders(const StartupScreenLayout &layout) {
  uiDisplayDrawRect(0, 0, layout.displayW, layout.displayH, kUiBorder);
  draw_horizontal_separator(layout.topBarH, layout.displayW, kUiBorder);
  draw_horizontal_separator(layout.footerY, layout.displayW, kUiBorder);
}

void draw_startup_base(const StartupScreenLayout &layout, const char *title, bool rtcDetected) {
  const uint8_t barTextFont = 2;
  const uint8_t barTextSize = layout.isLargeDisplay ? 2 : 1;
  const uint8_t footerTextFont = kFooterTextFont;
  const uint8_t footerTextSize = kFooterTextSize;
  const int topBarTextY =
      ((layout.topBarH - uiDisplayFontHeight(barTextSize, barTextFont)) / 2) + (layout.isLargeDisplay ? 1 : 0);
  const int footerTextY = layout.footerY + ((layout.bottomBarH - uiDisplayFontHeight(footerTextSize, footerTextFont)) / 2);

  uiDisplayClear();
  uiDisplayFillRect(0, 0, layout.displayW, layout.topBarH, kUiAccent);
  uiDisplayFillRect(1, layout.contentY + 1, layout.displayW - 2, layout.contentH - 1, kUiModeAreaBg);
  uiDisplayFillRect(0, layout.footerY, layout.displayW, layout.bottomBarH, kUiAccent);
  draw_startup_borders(layout);

  uiDisplayPrintStyledAt(layout.displayW / 50, topBarTextY, title, kUiText, kUiAccent, barTextSize, barTextFont);
  uiDisplayPrintStyledAt(4, footerTextY, kFirmwareVersion, kUiText, kUiAccent, footerTextSize, footerTextFont);
  if (rtcDetected) {
    const String footerDateTime = rtc_timestamp_text();
    uiDisplayPrintStyledAt(layout.displayW - uiDisplayTextWidth(footerDateTime, footerTextSize, footerTextFont) - 4,
                           footerTextY,
                           footerDateTime,
                           kUiText,
                           kUiAccent,
                           footerTextSize,
                           footerTextFont);
  }
}

String startup_status_text(bool detected) {
  return detected ? "Detected" : "Not detected";
}

int cc_cursor_text_index(const String &valueText, int cursorPosition) {
  const int decimal = valueText.indexOf('.');
  switch (cursorPosition) {
    case 8: return (decimal > 0) ? (decimal - 1) : 0;
    case 10: return (decimal >= 0 && decimal + 1 < valueText.length()) ? (decimal + 1) : -1;
    case 11: return (decimal >= 0 && decimal + 2 < valueText.length()) ? (decimal + 2) : -1;
    case 12: return (decimal >= 0 && decimal + 3 < valueText.length()) ? (decimal + 3) : -1;
    default: return -1;
  }
}

int cp_cursor_text_index(const String &valueText, int cursorPosition) {
  const int decimal = valueText.indexOf('.');
  switch (cursorPosition) {
    case 6: return 0;
    case 7: return 1;
    case 8: return (decimal > 0) ? (decimal - 1) : 2;
    case 10: return (decimal >= 0 && decimal + 1 < valueText.length()) ? (decimal + 1) : -1;
    default: return -1;
  }
}

int cr_cursor_text_index(const String &valueText, int cursorPosition) {
  const int decimal = valueText.indexOf('.');
  switch (cursorPosition) {
    case 6: return 0;
    case 7: return 1;
    case 8: return 2;
    case 10: return (decimal >= 0 && decimal + 1 < valueText.length()) ? (decimal + 1) : -1;
    default: return -1;
  }
}

int bc_cursor_text_index(const String &valueText, int cursorPosition) {
  return cc_cursor_text_index(valueText, cursorPosition);
}

int tc_cursor_text_index(const String &valueText, int cursorPosition) {
  (void)valueText;
  switch (cursorPosition) {
    case 8: return 0;
    case 9: return 1;
    case 10: return 2;
    case 11: return 3;
    case 12: return 4;
    default: return -1;
  }
}

void draw_degree_c_symbol(int x, int y, uint16_t color, uint16_t bg, uint8_t textSize, uint8_t textFont) {
  const int radius = (textSize >= 2) ? 3 : 2;
  uiDisplayDrawCircle(x, y + radius + 1, radius, color);
  uiDisplayPrintStyledAt(x + 5, y - 1, "C", color, bg, textSize, textFont);
}

void draw_setup_screen_base(const UiViewState &state, const char *modeLabel) {
  const ManagedZoneLayout layout = managed_zone_layout();
  const MetricsZoneState metricsState = {format_measured_current_display(max(0.0f, state.measuredCurrent_A)),
                                         format_measured_voltage_display(max(0.0f, state.measuredVoltage_V)),
                                         format_measured_power_display(max(0.0f, state.measuredPower_W))};
  const TopStatusZoneState topState = {String(modeLabel), state.loadEnabled, state.tempC, state.fanTempOnC, state.tempCutOffC};
  const FooterZoneState footerState = current_footer_zone_state();

  uiDisplayClear();
  draw_top_status_zone(layout, topState);
  draw_metrics_zone(layout, metricsState);
  uiDisplayFillRect(1, layout.contentY + 1, layout.displayW - 2, layout.contentH - 1, kUiModeAreaBg);
  draw_footer_zone(layout, footerState);
  draw_zone_borders(layout);
  g_setupMetricsValid = true;
  g_setupMetricsDisplayW = layout.displayW;
  g_setupMetricsDisplayH = layout.displayH;
  g_setupMetricsMode = state.mode;
  g_setupLastMetric1 = metricsState.metric1;
  g_setupLastMetric2 = metricsState.metric2;
  g_setupLastMetric3 = metricsState.metric3;
}

void update_setup_metrics_internal(const UiViewState &state) {
  const ManagedZoneLayout layout = managed_zone_layout();
  const MetricsZoneState metricsState = {format_measured_current_display(max(0.0f, state.measuredCurrent_A)),
                                         format_measured_voltage_display(max(0.0f, state.measuredVoltage_V)),
                                         format_measured_power_display(max(0.0f, state.measuredPower_W))};
  const bool layoutChanged =
      !g_setupMetricsValid || g_setupMetricsDisplayW != layout.displayW || g_setupMetricsDisplayH != layout.displayH || g_setupMetricsMode != state.mode;

  update_metrics_zone_values(layout, metricsState, layoutChanged, g_setupLastMetric1, g_setupLastMetric2, g_setupLastMetric3);
  g_setupMetricsValid = true;
  g_setupMetricsDisplayW = layout.displayW;
  g_setupMetricsDisplayH = layout.displayH;
  g_setupMetricsMode = state.mode;
}

void draw_battery_setup_set_zone(const String &prefix, const String &valueText) {
  const ManagedZoneLayout layout = managed_zone_layout();
  clear_input_zone(layout);
  draw_input_zone_value(layout, prefix, valueText);
  if (app_msc_shift_active()) {
    draw_input_zone_shift_hint(layout);
  }
}

bool render_managed_home(const UiViewState &state, bool cursorVisible) {
  const ManagedZoneLayout layout = managed_zone_layout();
  const int displayW = layout.displayW;
  const int displayH = layout.displayH;
  const bool isLargeDisplay = layout.isLargeDisplay;
  const bool isCcMode = state.mode == CC;
  const bool isCpMode = state.mode == CP;
  const bool isCrMode = state.mode == CR;
  const bool isBcMode = state.mode == BC;
  const bool isTcMode = state.mode == TC;
  const bool isTlMode = state.mode == TL;
  const bool isCaMode = state.mode == CA;
  const bool traceOverlaySupportedMode = isCcMode || isCpMode || isCrMode || isBcMode;
  const bool traceOverlayActive = state.traceOverlayActive && traceOverlaySupportedMode;
  const bool isTransientMode = isTcMode || isTlMode;
  const int topBarH = layout.topBarH;
  const int metricsH = layout.metricsH;
  const int setZoneH = layout.setZoneH;
  const int bottomBarH = layout.bottomBarH;
  const int footerY = layout.footerY;
  const int setZoneY = layout.setZoneY;
  const int contentY = layout.contentY;
  const int contentH = layout.contentH;
  const int metricsBoxY = topBarH;
  const int metricsBoxH = metricsH;
  const uint8_t barTextFont = 2;
  const uint8_t barTextSize = isLargeDisplay ? 2 : 1;
  const uint8_t metricTextFont = 1;
  const uint8_t sectionTextFont = 2;
  const uint8_t sectionTextSize = isLargeDisplay ? 2 : 1;
  const uint8_t footerTextFont = kFooterTextFont;
  const uint8_t footerTextSize = kFooterTextSize;

  const int topBarTextY = ((topBarH - uiDisplayFontHeight(barTextSize, barTextFont)) / 2) + (isLargeDisplay ? 1 : 0);
  const bool calibrationVoltageMode = app_calibration_is_voltage_mode();
  const bool calibrationFirstPointTaken = app_calibration_first_point_taken();
  const char *modeLabel = isCcMode ? "CC" : isCpMode ? "CP" : isCrMode ? "CR" : isBcMode ? "BC" : isTcMode ? "TC" : isTlMode ? "TL" : "CA";
  const bool useRawMetrics = isCaMode;
  const MetricsZoneState metricsState = {
      useRawMetrics ? format_measured_current_raw(max(0.0f, state.measuredCurrent_A))
                    : format_measured_current_display(max(0.0f, state.measuredCurrent_A)),
      useRawMetrics ? format_measured_voltage_raw(max(0.0f, state.measuredVoltage_V))
                    : format_measured_voltage_display(max(0.0f, state.measuredVoltage_V)),
      useRawMetrics ? format_measured_power_raw(max(0.0f, state.measuredPower_W))
                    : format_measured_power_display(max(0.0f, state.measuredPower_W))};
  const int setTextY = setZoneY + ((setZoneH - uiDisplayFontHeight(sectionTextSize, sectionTextFont)) / 2);
  const String footerDateTime = rtc_timestamp_text();
  const char *liveInput = app_input_text();
  const bool supportsLiveInput = isCcMode || isCpMode || isCrMode || isBcMode || isTcMode;
  const bool hasLiveInput = supportsLiveInput && (liveInput != nullptr) && (liveInput[0] != '\0');
  const bool hasCalibrationRealInput = isCaMode && (liveInput != nullptr) && (liveInput[0] != '\0');
  const int transientVisibleTotalSteps = (state.transientListTotalSteps > 0) ? state.transientListTotalSteps : 1;
  const String transientStepText = String(state.transientListActiveStep + 1) + "/" + String(transientVisibleTotalSteps);
  const String setValueText = hasLiveInput
                                  ? String(liveInput)
                                  : ((isCcMode || isBcMode) ? String(state.readingValue, 3)
                                              : isCaMode ? String(state.readingValue, 3)
                                              : isCpMode ? format_cp_value(state.readingValue)
                                              : isCrMode ? format_cr_value(state.readingValue)
                                              : isTcMode ? format_tc_period_value(state.transientPeriodMs)
                                                         : transientStepText);
  const String setPrefix = isBcMode ? "CURR> " : isTcMode ? "PER> " : isTlMode ? "STEP> " : "SET> ";
  const String setUnit = isCrMode ? " ohms" : isTcMode ? " ms" : ((isCcMode || isBcMode || isCaMode) ? "A" : isCpMode ? "W" : "");
  const String calibrationRealPrefix = "REAL> ";
  const String calibrationRealUnit = calibrationVoltageMode ? "V" : "A";
  const String calibrationRealValue = hasCalibrationRealInput ? String(liveInput) : "";
  const String setText = setPrefix + setValueText + setUnit;
  const bool bcWasRunning = (g_ccLastBatteryStatus == "Discharging...") || (g_ccLastBatteryStatus == "Pause");
#ifdef WOKWI_SIMULATION
  const bool bcVoltageDropped = state.measuredVoltage_V < (SIM_DEFAULT_VOLTAGE - 0.001f);
#else
  const bool bcVoltageDropped = false;
#endif
  const bool bcPaused = app_mode_state_configured() &&
                        !state.batteryDone &&
                        !state.loadEnabled &&
                        ((state.batteryLife > 0.0f) || bcVoltageDropped || bcWasRunning);
  const String bcStatus = state.batteryDone ? "Done!" : state.loadEnabled ? "Discharging..." : bcPaused ? "Pause" : "Ready";
  const uint16_t bcStatusColor = state.batteryDone ? TFT_GREEN : state.loadEnabled ? TFT_RED : kUiHighlight;
  const String modeTitle = isBcMode ? "BATTERY CAPACITY"
                                    : isTcMode ? "TRANSIENT CONT"
                                    : isTlMode ? "TRANSIENT LIST"
                                               : (calibrationVoltageMode ? "CAL VOLTAGE" : "CAL CURRENT");
  const String modeLine1 = isBcMode ? String("Status:")
                                    : isTcMode ? String(state.loadEnabled ? "Output toggling" : "Enable load to start")
                                    : isCaMode ? String("Point: ") + (calibrationFirstPointTaken ? "P2" : "P1")
                                    : isTlMode ? String(state.loadEnabled ? "Sequence running" : "Enable load to start")
                                               : "";
  const String modeLine2 = isBcMode ? String("Cutoff: ") + String(state.batteryCutoffVolts, 2) + "v"
                                    : isTcMode ? String("I2: ") + format_transient_current(state.transientHighCurrentA)
                                               : isCaMode ? String("Target with encoder")
                                               : String("Set: ") + format_transient_current(max(0.0f, state.setCurrent_mA / 1000.0f));
  const String modeLine3 = isBcMode ? String("Type: ") + String(state.batteryType)
                                    : isTcMode ? String("I1: ") + format_transient_current(state.transientLowCurrentA)
                                               : isCaMode ? String("Enter REAL at right")
                                               : String("Total: ") + String(transientVisibleTotalSteps);
  const String modeLine4 = isBcMode ? String("Capacity: ") + String(state.batteryLife, 0) + " mAh"
                                    : isTcMode ? String("dt: ") + format_transient_period_label(state.transientPeriodMs)
                                               : isCaMode ? ""
                                               : String("dt: ") + format_transient_period_label(state.transientPeriodMs);
  const String modeLine5 = isBcMode ? String("Elapsed: ") + app_timer_get_time()
                                    : isTcMode ? ""
                                               : isCaMode ? ""
                                               : String("Total: ") + String(transientVisibleTotalSteps);

  const bool modeChanged = g_ccLastMode != state.mode;
  const bool layoutChanged = !g_ccLayoutDrawn || g_ccLastDisplayW != displayW || g_ccLastDisplayH != displayH || modeChanged;
  if (layoutChanged) {
    uiDisplayClear();
    uiDisplayFillRect(0, 0, displayW, topBarH, kUiAccent);
    uiDisplayFillRect(1, contentY + 1, displayW - 2, contentH - 1, kUiModeAreaBg);
    draw_footer_zone(layout, current_footer_zone_state());
    draw_zone_borders(layout);
    g_ccLayoutDrawn = true;
    g_ccLastDisplayW = displayW;
    g_ccLastDisplayH = displayH;
    g_ccLastMode = state.mode;
    g_ccLastLoadEnabled = !state.loadEnabled;
    g_ccLastCursorVisible = !cursorVisible;
    g_ccLastCursorPosition = -1;
    g_ccLastMetric1 = "";
    g_ccLastMetric2 = "";
    g_ccLastMetric3 = "";
    g_ccLastSetText = "";
    g_ccLastFooterText = "";
    g_ccLastTempText = "";
    g_ccLastModeLine1 = "";
    g_ccLastModeLine2 = "";
    g_ccLastModeLine3 = "";
    g_ccLastModeLine4 = "";
    g_ccLastModeLine5 = "";
    g_ccLastBatteryStatus = "";
    g_ccLastShiftActive = false;
    g_ccLastTraceToken = 0;
    g_ccLastTraceSampleCount = 0;
    g_ccLastTraceTimeScaleSeconds = 0.0f;
    g_ccLastTraceCurrentScaleMax = 0.0f;
    g_ccLastTraceVoltageScaleMax = 0.0f;
    g_ccLastTraceOverlayActive = false;
    g_ccLastBatteryStatusColor = 0;
  }

  const bool traceOverlayChanged = layoutChanged || (g_ccLastTraceOverlayActive != traceOverlayActive);
  if (traceOverlayChanged) {
    uiDisplayFillRect(1, contentY + 1, displayW - 2, contentH - 1, kUiModeAreaBg);
    g_ccLastModeLine1 = "";
    g_ccLastModeLine2 = "";
    g_ccLastModeLine3 = "";
    g_ccLastModeLine4 = "";
    g_ccLastModeLine5 = "";
    g_ccLastTraceToken = 0;
    g_ccLastTraceSampleCount = 0;
    g_ccLastTraceTimeScaleSeconds = 0.0f;
    g_ccLastTraceCurrentScaleMax = 0.0f;
    g_ccLastTraceVoltageScaleMax = 0.0f;
    g_ccLastTraceOverlayActive = traceOverlayActive;
  }

  const String tempText = String(constrain(static_cast<int>(state.tempC), 0, 99));
  const uint16_t tempColor = temperature_status_color(state.tempC, state.fanTempOnC, state.tempCutOffC);
  if (layoutChanged || g_ccLastLoadEnabled != state.loadEnabled || g_ccLastTempText != tempText) {
    draw_top_status_zone(layout, {String(modeLabel), state.loadEnabled, state.tempC, state.fanTempOnC, state.tempCutOffC});
    g_ccLastLoadEnabled = state.loadEnabled;
    g_ccLastTempText = tempText;
    draw_zone_borders(layout);
  }

  update_metrics_zone_values(layout, metricsState, layoutChanged, g_ccLastMetric1, g_ccLastMetric2, g_ccLastMetric3);

  if (!traceOverlayActive &&
      isBcMode &&
      (layoutChanged || traceOverlayChanged || g_ccLastModeLine1 != modeLine1 || g_ccLastModeLine2 != modeLine2 ||
       g_ccLastModeLine3 != modeLine3 || g_ccLastModeLine4 != modeLine4 || g_ccLastModeLine5 != modeLine5 ||
       g_ccLastBatteryStatus != bcStatus || g_ccLastBatteryStatusColor != bcStatusColor)) {
    render_bc_mode_content(layout,
                           layoutChanged || traceOverlayChanged,
                           modeTitle,
                           modeLine1,
                           bcStatus,
                           bcStatusColor,
                           modeLine3,
                           modeLine2,
                           modeLine4,
                           modeLine5);
  }

  if (!traceOverlayActive &&
      isTransientMode &&
      (layoutChanged || traceOverlayChanged || g_ccLastModeLine1 != modeLine1 || g_ccLastModeLine2 != modeLine2 ||
       g_ccLastModeLine3 != modeLine3 || g_ccLastModeLine4 != modeLine4)) {
    render_transient_mode_content(layout,
                                  layoutChanged || traceOverlayChanged,
                                  modeTitle,
                                  modeLine1,
                                  modeLine3,
                                  modeLine2,
                                  modeLine4);
  }

  if (!traceOverlayActive &&
      isCaMode &&
      (layoutChanged || g_ccLastModeLine1 != modeLine1 || g_ccLastModeLine2 != modeLine2 ||
       g_ccLastModeLine3 != modeLine3 || g_ccLastModeLine4 != modeLine4 || g_ccLastModeLine5 != modeLine5)) {
    render_calibration_mode_content(layout, layoutChanged, modeTitle, modeLine1, modeLine2, modeLine3);
  }

  const uint32_t traceToken = app_trace_update_token();
  if (traceOverlayActive && (layoutChanged || traceOverlayChanged || g_ccLastTraceToken != traceToken)) {
    update_trace_overlay_content(layout, layoutChanged, traceOverlayChanged, traceToken);
  }

  const bool supportsCursorHighlight = isCcMode || isCpMode || isCrMode || isBcMode || isTcMode || isCaMode;
  const bool shiftActive = app_msc_shift_active();
  const bool cursorRefreshBlocked = hasLiveInput || (!isCaMode && hasCalibrationRealInput);
  if (layoutChanged || g_ccLastSetText != (isCaMode ? (setText + "|" + calibrationRealValue) : setText) || g_ccLastShiftActive != shiftActive ||
      (supportsCursorHighlight && !cursorRefreshBlocked &&
       (g_ccLastCursorVisible != cursorVisible || g_ccLastCursorPosition != state.cursorPosition))) {
    clear_input_zone(layout);
    const InputZoneRenderLayout inputLayout = compute_input_zone_render_layout(layout);
    if (isCaMode) {
      const int realBlockWidth = uiDisplayTextWidth(calibrationRealPrefix + String("0.000") + calibrationRealUnit,
                                                    inputLayout.textSize,
                                                    inputLayout.textFont);
      const int realX = displayW - realBlockWidth - max(24, displayW / 25);
      const int prefixWidth = uiDisplayTextWidth(setPrefix, inputLayout.textSize, inputLayout.textFont);
      uiDisplayPrintStyledAt(inputLayout.textX, inputLayout.textY, setPrefix, kUiSetColor, kUiBg, inputLayout.textSize, inputLayout.textFont);
      const int highlightedIndex = cc_cursor_text_index(setValueText, state.cursorPosition);
      int currentX = draw_input_zone_highlighted_value(layout, inputLayout.textX + prefixWidth, setValueText, highlightedIndex, cursorVisible);
      uiDisplayPrintStyledAt(currentX,
                             inputLayout.textY,
                             setUnit,
                             kUiSetColor,
                             kUiBg,
                             inputLayout.textSize,
                             inputLayout.textFont);
      uiDisplayPrintStyledAt(realX,
                             inputLayout.textY,
                             calibrationRealPrefix,
                             kUiSetColor,
                             kUiBg,
                             inputLayout.textSize,
                             inputLayout.textFont);
      uiDisplayPrintStyledAt(realX + uiDisplayTextWidth(calibrationRealPrefix, inputLayout.textSize, inputLayout.textFont),
                             inputLayout.textY,
                             calibrationRealValue + calibrationRealUnit,
                             kUiSetColor,
                             kUiBg,
                             inputLayout.textSize,
                             inputLayout.textFont);
    } else {
      const int prefixWidth = uiDisplayTextWidth(setPrefix, inputLayout.textSize, inputLayout.textFont);
      uiDisplayPrintStyledAt(inputLayout.textX, inputLayout.textY, setPrefix, kUiSetColor, kUiBg, inputLayout.textSize, inputLayout.textFont);

      if (hasLiveInput) {
        uiDisplayPrintStyledAt(inputLayout.textX + prefixWidth,
                               inputLayout.textY,
                               setText.substring(5),
                               kUiSetColor,
                               kUiBg,
                               inputLayout.textSize,
                               inputLayout.textFont);
      } else if (supportsCursorHighlight) {
        const int highlightedIndex = isCcMode
                                         ? cc_cursor_text_index(setValueText, state.cursorPosition)
                                         : isCpMode ? cp_cursor_text_index(setValueText, state.cursorPosition)
                                         : isCrMode ? cr_cursor_text_index(setValueText, state.cursorPosition)
                                         : isTcMode ? tc_cursor_text_index(setValueText, state.cursorPosition)
                                                    : bc_cursor_text_index(setValueText, state.cursorPosition);
        int currentX =
            draw_input_zone_highlighted_value(layout, inputLayout.textX + prefixWidth, setValueText, highlightedIndex, cursorVisible);
        uiDisplayPrintStyledAt(currentX,
                               inputLayout.textY,
                               setUnit,
                               kUiSetColor,
                               kUiBg,
                               inputLayout.textSize,
                               inputLayout.textFont);
      } else {
        uiDisplayPrintStyledAt(inputLayout.textX + prefixWidth,
                               inputLayout.textY,
                               setValueText + setUnit,
                               kUiSetColor,
                               kUiBg,
                               inputLayout.textSize,
                               inputLayout.textFont);
      }
    }
    if (shiftActive) {
      draw_input_zone_shift_hint(layout);
    }
    g_ccLastSetText = isCaMode ? (setText + "|" + calibrationRealValue) : setText;
    g_ccLastCursorVisible = cursorVisible;
    g_ccLastCursorPosition = state.cursorPosition;
    g_ccLastShiftActive = shiftActive;
  }

  if (layoutChanged || g_ccLastFooterText != footerDateTime) {
    draw_footer_zone(layout, current_footer_zone_state());
    g_ccLastFooterText = footerDateTime;
    draw_zone_borders(layout);
  }
  draw_zone_borders(layout);
  return true;
}
}

int uiDisplayWidthPx() { return tft.width(); }

int uiDisplayHeightPx() { return tft.height(); }

void uiDisplayUpdateSetupMetrics(const UiViewState &state) {
  update_setup_metrics_internal(state);
}

void uiDisplayUpdateSetupTopStatus(const UiViewState &state) {
  const ManagedZoneLayout layout = managed_zone_layout();
  draw_top_status_zone(layout, {String(mode_label_for_value(state.mode)), state.loadEnabled, state.tempC, state.fanTempOnC, state.tempCutOffC});
  draw_zone_borders(layout);
}

void uiDisplayUpdateSetupFooterTime(void) {
  const ManagedZoneLayout layout = managed_zone_layout();
  draw_footer_zone(layout, current_footer_zone_state());
  draw_zone_borders(layout);
}

int uiDisplayTextWidth(const char *text, uint8_t textSize, uint8_t textFont) {
  tft.setTextFont(textFont);
  tft.setTextSize(textSize);
  const int width = tft.textWidth(text);
  restore_tft_text_style();
  return width;
}

int uiDisplayTextWidth(const String &text, uint8_t textSize, uint8_t textFont) {
  return uiDisplayTextWidth(text.c_str(), textSize, textFont);
}

int uiDisplayFontHeight(uint8_t textSize, uint8_t textFont) {
  tft.setTextFont(textFont);
  tft.setTextSize(textSize);
  const int height = tft.fontHeight();
  restore_tft_text_style();
  return height;
}

void uiDisplayInit(void) {
  tft.init();
  tft.setRotation(1);
  tft.fillScreen(TFT_BG_COLOR);
  restore_tft_text_style();
  tft.setTextWrap(false);
}

void uiDisplayClear(void) {
  tft.fillScreen(TFT_BG_COLOR);
  tft.setCursor(0, 0);
  restore_tft_text_style();
}

void uiDisplayInvalidateHomeLayout(void) {
  g_ccLayoutDrawn = false;
  g_ccLastMode = 0xFF;
  g_ccLastCursorPosition = -1;
  g_ccLastMetric1 = "";
  g_ccLastMetric2 = "";
  g_ccLastMetric3 = "";
  g_ccLastSetText = "";
  g_ccLastFooterText = "";
  g_ccLastTempText = "";
  g_ccLastModeLine1 = "";
  g_ccLastModeLine2 = "";
  g_ccLastModeLine3 = "";
  g_ccLastModeLine4 = "";
  g_ccLastModeLine5 = "";
}

void uiDisplayInvalidateFwUpdateLayout(void) {
  g_fwUpdateLayoutDrawn = false;
  g_fwUpdateLastDisplayW = 0;
  g_fwUpdateLastDisplayH = 0;
  g_fwUpdateLastStatus = "";
  g_fwUpdateLastDetail = "";
  g_fwUpdateLastHint = "";
}

void uiDisplayFillRect(int x, int y, int w, int h, uint16_t color) {
  tft.fillRect(x, y, w, h, color);
}

void uiDisplayDrawRect(int x, int y, int w, int h, uint16_t color) {
  tft.drawRect(x, y, w, h, color);
}

void uiDisplayFillCircle(int x, int y, int r, uint16_t color) {
  tft.fillCircle(x, y, r, color);
}

void uiDisplayDrawCircle(int x, int y, int r, uint16_t color) {
  tft.drawCircle(x, y, r, color);
}

void uiDisplayPrintAt(int x, int y, const char *text, uint16_t fg, uint16_t bg, uint8_t textSize) {
  uiDisplayPrintStyledAt(x, y, text, fg, bg, textSize, 1);
}

void uiDisplayPrintStyledAt(int x, int y, const char *text, uint16_t fg, uint16_t bg, uint8_t textSize, uint8_t textFont) {
  tft.setTextColor(fg, bg);
  tft.setTextFont(textFont);
  tft.setTextSize(textSize);
  tft.setCursor(x, y);
  tft.print(text);
  restore_tft_text_style();
}

void uiDisplayPrintAt(int x, int y, const String &text, uint16_t fg, uint16_t bg, uint8_t textSize) {
  uiDisplayPrintStyledAt(x, y, text, fg, bg, textSize, 1);
}

void uiDisplayPrintStyledAt(int x, int y, const String &text, uint16_t fg, uint16_t bg, uint8_t textSize, uint8_t textFont) {
  tft.setTextColor(fg, bg);
  tft.setTextFont(textFont);
  tft.setTextSize(textSize);
  tft.setCursor(x, y);
  tft.print(text);
  restore_tft_text_style();
}

void uiDisplayRenderBatterySetupTask(const UiViewState &state) {
  const ManagedZoneLayout layout = managed_zone_layout();
  const ContentTextPanelLayout panel = compute_content_text_panel_layout(layout);
  const String title = "BATTERY SETUP";
  const int gap = layout.isLargeDisplay ? 34 : 18;
  const int leftColW = max(uiDisplayTextWidth("1 Stor Li-Po", panel.textSize, panel.textFont),
                           max(uiDisplayTextWidth("3 Disc Li-Po", panel.textSize, panel.textFont),
                               uiDisplayTextWidth("5 Custom Cutoff", panel.textSize, panel.textFont)));
  const int rightColW = max(uiDisplayTextWidth("2 Stor Li-Ion", panel.textSize, panel.textFont),
                            uiDisplayTextWidth("4 Disc Li-Ion", panel.textSize, panel.textFont));
  const int blockW = leftColW + gap + rightColW;
  const int leftX = (layout.displayW - blockW) / 2;
  const int rightX = leftX + leftColW + gap;

  draw_setup_screen_base(state, "BC");
  draw_content_panel_title(layout, panel, title);
  uiDisplayPrintStyledAt(leftX, panel.row1Y, "1 Stor Li-Po", kUiText, kUiModeAreaBg, panel.textSize, panel.textFont);
  uiDisplayPrintStyledAt(rightX, panel.row1Y, "2 Stor Li-Ion", kUiText, kUiModeAreaBg, panel.textSize, panel.textFont);
  uiDisplayPrintStyledAt(leftX, panel.row2Y, "3 Disc Li-Po", kUiText, kUiModeAreaBg, panel.textSize, panel.textFont);
  uiDisplayPrintStyledAt(rightX, panel.row2Y, "4 Disc Li-Ion", kUiText, kUiModeAreaBg, panel.textSize, panel.textFont);
  uiDisplayPrintStyledAt(leftX, panel.row3Y, "5 Custom Cutoff", kUiText, kUiModeAreaBg, panel.textSize, panel.textFont);
  draw_battery_setup_set_zone("SEL> ", "1..5");
  draw_zone_borders(layout);
}

void uiDisplayRenderBatterySetupCustom(const UiViewState &state) {
  const ManagedZoneLayout layout = managed_zone_layout();
  const ContentTextPanelLayout panel = compute_content_text_panel_layout(layout);
  const String title = "CUSTOM CUTOFF";
  const String line1 = String("Battery: ") + String(state.batteryType);
  const String line2 = "Enter cutoff voltage";
  const String line3 = "Range: 0.1v to 25.0v";
  draw_setup_screen_base(state, "BC");
  draw_content_panel_title(layout, panel, title);
  uiDisplayPrintStyledAt(centered_content_text_x(layout, line1, panel.textSize, panel.textFont), panel.row1Y, line1, kUiText, kUiModeAreaBg, panel.textSize, panel.textFont);
  uiDisplayPrintStyledAt(centered_content_text_x(layout, line2, panel.textSize, panel.textFont), panel.row2Y, line2, kUiText, kUiModeAreaBg, panel.textSize, panel.textFont);
  uiDisplayPrintStyledAt(centered_content_text_x(layout, line3, panel.textSize, panel.textFont), panel.row3Y, line3, kUiText, kUiModeAreaBg, panel.textSize, panel.textFont);
  uiDisplayUpdateBatterySetupCustomValue(state);
  draw_zone_borders(layout);
}

void uiDisplayRenderBatterySetupCells(const UiViewState &state) {
  const ManagedZoneLayout layout = managed_zone_layout();
  const ContentTextPanelLayout panel = compute_content_text_panel_layout(layout);
  const String title = "CELL COUNT";
  const String line1 = String("Battery: ") + String(state.batteryType);
  const String line2 = "Enter cells in series";
  const String line3 = "Range: 1 to 6";
  draw_setup_screen_base(state, "BC");
  draw_content_panel_title(layout, panel, title);
  uiDisplayPrintStyledAt(centered_content_text_x(layout, line1, panel.textSize, panel.textFont), panel.row1Y, line1, kUiText, kUiModeAreaBg, panel.textSize, panel.textFont);
  uiDisplayPrintStyledAt(centered_content_text_x(layout, line2, panel.textSize, panel.textFont), panel.row2Y, line2, kUiText, kUiModeAreaBg, panel.textSize, panel.textFont);
  uiDisplayPrintStyledAt(centered_content_text_x(layout, line3, panel.textSize, panel.textFont), panel.row3Y, line3, kUiText, kUiModeAreaBg, panel.textSize, panel.textFont);
  uiDisplayUpdateBatterySetupCellsValue(state);
  draw_zone_borders(layout);
}

void uiDisplayUpdateBatterySetupCustomValue(const UiViewState &state) {
  const String setValue = (state.batteryInputText[0] != '\0') ? String(state.batteryInputText) + "v" : "";
  draw_battery_setup_set_zone("SET> ", setValue);
}

void uiDisplayUpdateBatterySetupCellsValue(const UiViewState &state) {
  const String setValue = (state.batteryInputText[0] != '\0') ? String(state.batteryInputText) + "S" : "";
  draw_battery_setup_set_zone("SEL> ", setValue);
}

void uiDisplayRenderTransientContSetup(const UiViewState &state) {
  const ManagedZoneLayout layout = managed_zone_layout();
  const ContentTextPanelLayout panel = compute_content_text_panel_layout(layout);
  const String title = "TRANSIENT CONT";
  const String line1 = String("I1(A): ") + ((state.transientSetupStage > 0) ? format_transient_current(state.transientLowCurrentA) : "--");
  const String line2 = String("I2(A): ") + ((state.transientSetupStage > 1) ? format_transient_current(state.transientHighCurrentA) : "--");
  const String line3 = String("dt(ms): ") + ((state.transientSetupStage > 2) ? String(static_cast<unsigned long>(state.transientPeriodMs)) : "--");

  draw_setup_screen_base(state, "TC");
  draw_content_panel_title(layout, panel, title);
  draw_content_stage_line(panel, line1, centered_content_text_x(layout, line1, panel.textSize, panel.textFont), panel.row1Y, state.transientSetupStage == 0);
  draw_content_stage_line(panel, line2, centered_content_text_x(layout, line2, panel.textSize, panel.textFont), panel.row2Y, state.transientSetupStage == 1);
  draw_content_stage_line(panel, line3, centered_content_text_x(layout, line3, panel.textSize, panel.textFont), panel.row3Y, state.transientSetupStage >= 2);
  uiDisplayUpdateTransientContSetupValue(state);
  draw_zone_borders(layout);
}

void uiDisplayUpdateTransientContSetupContent(const UiViewState &state) {
  const ManagedZoneLayout layout = managed_zone_layout();
  const ContentTextPanelLayout panel = compute_content_text_panel_layout(layout);
  const String title = "TRANSIENT CONT";
  const String line1 = String("I1(A): ") + ((state.transientSetupStage > 0) ? format_transient_current(state.transientLowCurrentA) : "--");
  const String line2 = String("I2(A): ") + ((state.transientSetupStage > 1) ? format_transient_current(state.transientHighCurrentA) : "--");
  const String line3 = String("dt(ms): ") + ((state.transientSetupStage > 2) ? String(static_cast<unsigned long>(state.transientPeriodMs)) : "--");
  clear_blue_content_zone(layout);
  draw_content_panel_title(layout, panel, title);
  draw_content_stage_line(panel, line1, centered_content_text_x(layout, line1, panel.textSize, panel.textFont), panel.row1Y, state.transientSetupStage == 0);
  draw_content_stage_line(panel, line2, centered_content_text_x(layout, line2, panel.textSize, panel.textFont), panel.row2Y, state.transientSetupStage == 1);
  draw_content_stage_line(panel, line3, centered_content_text_x(layout, line3, panel.textSize, panel.textFont), panel.row3Y, state.transientSetupStage >= 2);
}

void uiDisplayUpdateTransientContSetupValue(const UiViewState &state) {
  String suffix = "";
  if (state.transientSetupStage < 2) {
    suffix = " A";
  } else {
    suffix = " ms";
  }

  const String setValue = (state.transientInputText[0] != '\0') ? String(state.transientInputText) + suffix : "";
  draw_battery_setup_set_zone("SET> ", setValue);
}

void uiDisplayRenderTransientListSetup(const UiViewState &state) {
  draw_setup_screen_base(state, "TL");
  uiDisplayUpdateTransientListSetupContent(state);
  uiDisplayUpdateTransientListSetupValue(state);
  draw_zone_borders(managed_zone_layout());
}

void uiDisplayUpdateTransientListSetupContent(const UiViewState &state) {
  const ManagedZoneLayout layout = managed_zone_layout();
  const ContentTextPanelLayout panel = compute_content_text_panel_layout(layout);
  const String title = "TRANSIENT LIST";
  clear_blue_content_zone(layout);
  draw_content_panel_title(layout, panel, title);

  if (state.transientListSetupStage == 0) {
    const String line1 = "How many steps?";
    const String line2Prefix = "Allowed range: ";
    const String line2Value = "2 to 10";
    const String line2 = line2Prefix + line2Value;
    const int line1X = centered_content_text_x(layout, line1, panel.textSize, panel.textFont);
    const int line2X = centered_content_text_x(layout, line2, panel.textSize, panel.textFont);
    const int line2ValueX = line2X + uiDisplayTextWidth(line2Prefix, panel.textSize, panel.textFont);
    const int highlightPadX = layout.isLargeDisplay ? 10 : 6;
    const int highlightPadY = layout.isLargeDisplay ? 4 : 2;
    uiDisplayPrintStyledAt(line1X, panel.row1Y, line1, kUiText, kUiModeAreaBg, panel.textSize, panel.textFont);
    uiDisplayPrintStyledAt(line2X, panel.row2Y, line2, kUiText, kUiModeAreaBg, panel.textSize, panel.textFont);
    uiDisplayFillRect(line2ValueX - highlightPadX,
                      panel.row2Y - highlightPadY,
                      uiDisplayTextWidth(line2Value, panel.textSize, panel.textFont) + (highlightPadX * 2),
                      panel.textH + (highlightPadY * 2),
                      kUiHighlight);
    uiDisplayPrintStyledAt(line2ValueX, panel.row2Y, line2Value, kUiDark, kUiHighlight, panel.textSize, panel.textFont);
  } else {
    const int visibleStep = state.transientListDraftStepIndex + 1;
    const int totalSteps = max(2, static_cast<int>(state.transientListDraftStepCount));
    const String line1 = "Step " + String(visibleStep) + " of " + String(totalSteps);
    const String line2 = String("I(A): ") +
                         ((state.transientListDraftField > 0) ? format_transient_current(state.transientListCurrentA) : "--");
    const String line3 = String("dt(ms): ") +
                         ((state.transientListCurrentPeriodMs > 0.0f)
                              ? String(static_cast<unsigned long>(state.transientListCurrentPeriodMs))
                              : "--");
    const int line1X = centered_content_text_x(layout, line1, panel.textSize, panel.textFont);
    const int line2X = centered_content_text_x(layout, line2, panel.textSize, panel.textFont);
    const int line3X = centered_content_text_x(layout, line3, panel.textSize, panel.textFont);
    uiDisplayPrintStyledAt(line1X, panel.row1Y, line1, kUiHighlight, kUiModeAreaBg, panel.textSize, panel.textFont);
    draw_content_stage_line(panel, line2, line2X, panel.row2Y, state.transientListDraftField == 0);
    draw_content_stage_line(panel, line3, line3X, panel.row3Y, state.transientListDraftField != 0);
  }
}

void uiDisplayUpdateTransientListSetupValue(const UiViewState &state) {
  String suffix = "";
  if (state.transientListSetupStage == 0) {
    suffix = "";
  } else if (state.transientListDraftField == 0) {
    suffix = " A";
  } else {
    suffix = " ms";
  }

  const String setValue = (state.transientListInputText[0] != '\0') ? String(state.transientListInputText) + suffix : "";
  const String prefix = (state.transientListSetupStage == 0) ? "STEPS> " : "SET> ";
  draw_battery_setup_set_zone(prefix, setValue);
}

void uiDisplayRenderStartupSplash(void) {
  const StartupScreenLayout layout = startup_screen_layout();
  const uint8_t titleFont = 2;
  const uint8_t titleSize = layout.isLargeDisplay ? 2 : 1;
  const uint8_t textFont = 2;
  const uint8_t textSize = layout.isLargeDisplay ? 2 : 1;
  const String line1 = "DC Electronic Load";
  const String line2 = "Initializing system...";
  const String line3 = "By Guy & Codex";
  const int centerY = layout.contentY + (layout.contentH / 2) - uiDisplayFontHeight(textSize, textFont);
  const int gap = layout.isLargeDisplay ? 20 : 12;
  const int line1Y = centerY - uiDisplayFontHeight(titleSize, titleFont) - gap;
  const int line2Y = centerY;
  const int line3Y = centerY + uiDisplayFontHeight(textSize, textFont) + gap;

  draw_startup_base(layout, "STARTUP", false);
  uiDisplayPrintStyledAt((layout.displayW - uiDisplayTextWidth(line1, titleSize, titleFont)) / 2,
                         line1Y,
                         line1,
                         kUiHighlight,
                         kUiModeAreaBg,
                         titleSize,
                         titleFont);
  uiDisplayPrintStyledAt((layout.displayW - uiDisplayTextWidth(line2, textSize, textFont)) / 2,
                         line2Y,
                         line2,
                         kUiText,
                         kUiModeAreaBg,
                         textSize,
                         textFont);
  uiDisplayPrintStyledAt((layout.displayW - uiDisplayTextWidth(line3, textSize, textFont)) / 2,
                         line3Y,
                         line3,
                         kUiText,
                         kUiModeAreaBg,
                         textSize,
                         textFont);
  draw_startup_borders(layout);
}

void uiDisplayRenderStartupHealthCheck(bool dacDetected, bool adsDetected, bool rtcDetected, bool sensorOk) {
  const StartupScreenLayout layout = startup_screen_layout();
  const uint8_t textFont = 2;
  const uint8_t textSize = layout.isLargeDisplay ? 2 : 1;
  const int lineHeight = uiDisplayFontHeight(textSize, textFont);
  const int verticalGap = layout.isLargeDisplay ? 16 : 10;
  const int totalBlockH = (lineHeight * 4) + (verticalGap * 3);
  const int startY = layout.contentY + max(8, (layout.contentH - totalBlockH) / 2);
  const String line1 = String("DAC: ") + startup_status_text(dacDetected);
  const String line2 = String("ADS: ") + startup_status_text(adsDetected);
  const String line3 = String("RTC: ") + startup_status_text(rtcDetected);
  const String line4 = String("TEMP: ") + (sensorOk ? "Detected" : "Fail");

  draw_startup_base(layout, "HEALTH CHECK", rtcDetected);
  uiDisplayPrintStyledAt((layout.displayW - uiDisplayTextWidth(line1, textSize, textFont)) / 2,
                         startY,
                         line1,
                         dacDetected ? kUiText : kUiHighlight,
                         kUiModeAreaBg,
                         textSize,
                         textFont);
  uiDisplayPrintStyledAt((layout.displayW - uiDisplayTextWidth(line2, textSize, textFont)) / 2,
                         startY + lineHeight + verticalGap,
                         line2,
                         adsDetected ? kUiText : kUiHighlight,
                         kUiModeAreaBg,
                         textSize,
                         textFont);
  uiDisplayPrintStyledAt((layout.displayW - uiDisplayTextWidth(line3, textSize, textFont)) / 2,
                         startY + ((lineHeight + verticalGap) * 2),
                         line3,
                         rtcDetected ? kUiText : kUiHighlight,
                         kUiModeAreaBg,
                         textSize,
                         textFont);
  uiDisplayPrintStyledAt((layout.displayW - uiDisplayTextWidth(line4, textSize, textFont)) / 2,
                         startY + ((lineHeight + verticalGap) * 3),
                         line4,
                         sensorOk ? kUiText : kUiHighlight,
                         kUiModeAreaBg,
                         textSize,
                         textFont);
  draw_startup_borders(layout);
}

namespace {

ManagedZoneLayout managed_zone_layout() {
  const int displayW = uiDisplayWidthPx();
  const int displayH = uiDisplayHeightPx();
  const int topBarH = (displayH * 10) / 100;
  const int metricsH = (displayH * 20) / 100;
  const int setZoneH = (displayH * 10) / 100;
  const int bottomBarH = footer_bar_height_px(displayH);
  const int footerY = displayH - bottomBarH;
  const int setZoneY = footerY - setZoneH;
  const int contentY = topBarH + metricsH;
  const int contentH = setZoneY - contentY;
  return {displayW, displayH, topBarH, metricsH, setZoneH, bottomBarH, footerY, setZoneY, contentY, contentH, displayW >= 400};
}

void draw_zone_borders(const ManagedZoneLayout &layout) {
  uiDisplayDrawRect(0, 0, layout.displayW, layout.displayH, kUiBorder);
  draw_horizontal_separator(layout.topBarH, layout.displayW, kUiBorder);
  draw_horizontal_separator(layout.contentY, layout.displayW, kUiBorder);
  draw_horizontal_separator(layout.setZoneY, layout.displayW, kUiBorder);
  draw_horizontal_separator(layout.footerY, layout.displayW, kUiBorder);
}

MetricsZoneState metrics_zone_stub_state() {
  return {"-.---a", "-.---v", "-.--w"};
}

TopStatusZoneState current_top_status_zone_state(const String &label, bool loadEnabled) {
  return {label,
          loadEnabled,
          static_cast<float>(app_measurements_temp_c()),
          static_cast<float>(app_fan_temp_on_c()),
          app_limits_temp_cutoff()};
}

FooterZoneState current_footer_zone_state() {
  return {String(kFirmwareVersion), rtc_timestamp_text()};
}

void draw_footer_zone(const ManagedZoneLayout &layout, const FooterZoneState &state) {
  const uint8_t footerTextFont = kFooterTextFont;
  const uint8_t footerTextSize = kFooterTextSize;
  const int footerTextY = layout.footerY + ((layout.bottomBarH - uiDisplayFontHeight(footerTextSize, footerTextFont)) / 2);

  uiDisplayFillRect(0, layout.footerY, layout.displayW, layout.bottomBarH, kUiAccent);
  uiDisplayPrintStyledAt(4, footerTextY, state.leftText, kUiText, kUiAccent, footerTextSize, footerTextFont);
  uiDisplayPrintStyledAt(layout.displayW - uiDisplayTextWidth(state.rightText, footerTextSize, footerTextFont) - 4,
                         footerTextY,
                         state.rightText,
                         kUiText,
                         kUiAccent,
                         footerTextSize,
                         footerTextFont);
}

void draw_top_status_zone(const ManagedZoneLayout &layout, const TopStatusZoneState &state) {
  const uint8_t barTextFont = 2;
  const uint8_t barTextSize = layout.isLargeDisplay ? 2 : 1;
  const int topBarTextY = ((layout.topBarH - uiDisplayFontHeight(barTextSize, barTextFont)) / 2) + (layout.isLargeDisplay ? 1 : 0);
  const String tempText = String(constrain(static_cast<int>(state.tempC), 0, 99));
  const uint16_t tempColor = temperature_status_color(state.tempC, state.fanTempOnC, state.tempCutOffC);
  const int tempBlockX = layout.displayW - (layout.isLargeDisplay ? 86 : 38);

  uiDisplayFillRect(0, 0, layout.displayW, layout.topBarH, kUiAccent);
  uiDisplayPrintStyledAt(layout.displayW / 50, topBarTextY, state.label, kUiText, kUiAccent, barTextSize, barTextFont);
  draw_centered_load_status(layout.displayW, layout.topBarH, state.loadEnabled, layout.isLargeDisplay, barTextSize, barTextFont);
  uiDisplayPrintStyledAt(tempBlockX, topBarTextY, tempText, tempColor, kUiAccent, barTextSize, barTextFont);
  draw_degree_c_symbol(tempBlockX + uiDisplayTextWidth(tempText, barTextSize, barTextFont) + (layout.isLargeDisplay ? 8 : 6),
                       topBarTextY + (layout.isLargeDisplay ? 3 : 1),
                       tempColor,
                       kUiAccent,
                       barTextSize,
                       barTextFont);
}

MetricsZoneRenderLayout compute_metrics_zone_render_layout(const ManagedZoneLayout &layout, const MetricsZoneState &state) {
  const uint8_t metricTextFont = 1;
  uint8_t metricTextSize = layout.isLargeDisplay ? 4 : 3;
  int metric1W = 0;
  int metric2W = 0;
  int metric3W = 0;
  int metricGap = 0;
  const int metricsMargin = layout.isLargeDisplay ? 12 : 8;
  const int metricsAvailableW = layout.displayW - (metricsMargin * 2);

  for (; metricTextSize > 0; --metricTextSize) {
    metric1W = uiDisplayTextWidth(state.metric1, metricTextSize, metricTextFont);
    metric2W = uiDisplayTextWidth(state.metric2, metricTextSize, metricTextFont);
    metric3W = uiDisplayTextWidth(state.metric3, metricTextSize, metricTextFont);
    metricGap = uiDisplayTextWidth(" ", metricTextSize, metricTextFont) / 2;
    const int totalW = metric1W + metric2W + metric3W + (metricGap * 2);
    if (totalW <= metricsAvailableW || metricTextSize == 1) break;
  }

  const int metricsTotalW = metric1W + metric2W + metric3W + (metricGap * 2);
  const int metricsStartX = ((layout.displayW - metricsTotalW) / 2) + (layout.isLargeDisplay ? 8 : 0);
  const int metricsTextY = layout.topBarH + ((layout.metricsH - uiDisplayFontHeight(metricTextSize, metricTextFont)) / 2) - (layout.isLargeDisplay ? 2 : 0);
  const int metricPadding = layout.isLargeDisplay ? 10 : 6;
  const int metric1X = metricsStartX - metricGap;
  const int metric2X = metricsStartX + metric1W + metricGap;
  const int metric3X = metricsStartX + metric1W + metricGap + metric2W + metricGap;
  const int metricAreaY = layout.topBarH + 1;
  const int metricAreaH = layout.metricsH - 2;

  return {metricTextFont,
          metricTextSize,
          metric1W,
          metric2W,
          metric3W,
          metricGap,
          metricsTextY,
          metricPadding,
          metric1X,
          metric2X,
          metric3X,
          metricAreaY,
          metricAreaH};
}

void draw_metrics_zone(const ManagedZoneLayout &layout, const MetricsZoneState &state) {
  const MetricsZoneRenderLayout render = compute_metrics_zone_render_layout(layout, state);
  uiDisplayFillRect(1, layout.topBarH + 1, layout.displayW - 2, layout.metricsH - 2, kUiBg);
  uiDisplayPrintStyledAt(render.metric1X, render.metricsTextY, state.metric1, kUiText, kUiBg, render.textSize, render.textFont);
  uiDisplayPrintStyledAt(render.metric2X, render.metricsTextY, state.metric2, kUiText, kUiBg, render.textSize, render.textFont);
  uiDisplayPrintStyledAt(render.metric3X, render.metricsTextY, state.metric3, kUiText, kUiBg, render.textSize, render.textFont);
}

void update_metrics_zone_values(const ManagedZoneLayout &layout,
                                const MetricsZoneState &state,
                                bool layoutChanged,
                                String &lastMetric1,
                                String &lastMetric2,
                                String &lastMetric3) {
  const MetricsZoneRenderLayout render = compute_metrics_zone_render_layout(layout, state);

  if (layoutChanged) {
    uiDisplayFillRect(1, layout.topBarH + 1, layout.displayW - 2, layout.metricsH - 2, kUiBg);
  }

  if (layoutChanged || lastMetric1 != state.metric1) {
    const int clearX = max(1, render.metric1X - render.metricPadding);
    const int clearW = min(layout.displayW - clearX - 1, render.metric1W + (render.metricPadding * 2));
    uiDisplayFillRect(clearX, render.metricAreaY, clearW, render.metricAreaH, kUiBg);
    uiDisplayPrintStyledAt(render.metric1X, render.metricsTextY, state.metric1, kUiText, kUiBg, render.textSize, render.textFont);
    lastMetric1 = state.metric1;
  }
  if (layoutChanged || lastMetric2 != state.metric2) {
    const int clearX = max(1, render.metric2X - render.metricPadding);
    const int clearW = min(layout.displayW - clearX - 1, render.metric2W + (render.metricPadding * 2));
    uiDisplayFillRect(clearX, render.metricAreaY, clearW, render.metricAreaH, kUiBg);
    uiDisplayPrintStyledAt(render.metric2X, render.metricsTextY, state.metric2, kUiText, kUiBg, render.textSize, render.textFont);
    lastMetric2 = state.metric2;
  }
  if (layoutChanged || lastMetric3 != state.metric3) {
    const int clearX = max(1, render.metric3X - render.metricPadding);
    const int clearW = min(layout.displayW - clearX - 1, render.metric3W + (render.metricPadding * 2));
    uiDisplayFillRect(clearX, render.metricAreaY, clearW, render.metricAreaH, kUiBg);
    uiDisplayPrintStyledAt(render.metric3X, render.metricsTextY, state.metric3, kUiText, kUiBg, render.textSize, render.textFont);
    lastMetric3 = state.metric3;
  }
}

InputZoneRenderLayout compute_input_zone_render_layout(const ManagedZoneLayout &layout) {
  const uint8_t textFont = 2;
  const uint8_t textSize = layout.isLargeDisplay ? 2 : 1;
  const int textY = layout.setZoneY + ((layout.setZoneH - uiDisplayFontHeight(textSize, textFont)) / 2);
  const int textX = layout.displayW / 50;
  const int shiftX = layout.displayW - uiDisplayTextWidth("sf", textSize, textFont) - max(6, layout.displayW / 50);
  return {textFont, textSize, textX, textY, shiftX};
}

void clear_input_zone(const ManagedZoneLayout &layout) {
  uiDisplayFillRect(1, layout.setZoneY + 1, layout.displayW - 2, layout.setZoneH - 2, kUiBg);
}

void draw_input_zone_value(const ManagedZoneLayout &layout, const String &prefix, const String &value) {
  const InputZoneRenderLayout render = compute_input_zone_render_layout(layout);
  uiDisplayPrintStyledAt(render.textX, render.textY, prefix, kUiSetColor, kUiBg, render.textSize, render.textFont);
  uiDisplayPrintStyledAt(render.textX + uiDisplayTextWidth(prefix, render.textSize, render.textFont),
                         render.textY,
                         value,
                         kUiSetColor,
                         kUiBg,
                         render.textSize,
                         render.textFont);
}

void draw_input_zone_shift_hint(const ManagedZoneLayout &layout) {
  const InputZoneRenderLayout render = compute_input_zone_render_layout(layout);
  uiDisplayPrintStyledAt(render.shiftX, render.textY, "sf", kUiSetColor, kUiBg, render.textSize, render.textFont);
}

int draw_input_zone_highlighted_value(const ManagedZoneLayout &layout,
                                      int startX,
                                      const String &valueText,
                                      int highlightedIndex,
                                      bool cursorVisible) {
  const InputZoneRenderLayout render = compute_input_zone_render_layout(layout);
  int currentX = startX;
  for (int i = 0; i < valueText.length(); ++i) {
    const String digitText = String(valueText.charAt(i));
    const bool highlighted = (i == highlightedIndex) && cursorVisible;
    uiDisplayPrintStyledAt(currentX,
                           render.textY,
                           digitText,
                           highlighted ? kUiBg : kUiSetColor,
                           highlighted ? kUiSetColor : kUiBg,
                           render.textSize,
                           render.textFont);
    currentX += uiDisplayTextWidth(digitText, render.textSize, render.textFont);
  }
  return currentX;
}

ContentTextPanelLayout compute_content_text_panel_layout(const ManagedZoneLayout &layout) {
  const uint8_t titleFont = 2;
  const uint8_t titleSize = layout.isLargeDisplay ? 2 : 1;
  const uint8_t textFont = 2;
  const uint8_t textSize = layout.isLargeDisplay ? 2 : 1;
  const int titleH = uiDisplayFontHeight(titleSize, titleFont);
  const int textH = uiDisplayFontHeight(textSize, textFont);
  const int verticalGap = max(layout.isLargeDisplay ? 6 : 4, (layout.contentH - titleH - (textH * 3)) / 5);
  const int titleY = layout.contentY + verticalGap;
  const int row1Y = titleY + titleH + verticalGap;
  const int row2Y = row1Y + textH + verticalGap;
  const int row3Y = row2Y + textH + verticalGap;
  return {titleFont, titleSize, textFont, textSize, titleH, textH, verticalGap, titleY, row1Y, row2Y, row3Y};
}

int centered_content_text_x(const ManagedZoneLayout &layout, const String &text, uint8_t textSize, uint8_t textFont) {
  return (layout.displayW - uiDisplayTextWidth(text, textSize, textFont)) / 2;
}

void clear_blue_content_zone(const ManagedZoneLayout &layout) {
  uiDisplayFillRect(1, layout.contentY + 1, layout.displayW - 2, layout.contentH - 1, kUiModeAreaBg);
}

void draw_content_panel_title(const ManagedZoneLayout &layout, const ContentTextPanelLayout &panel, const String &title) {
  uiDisplayPrintStyledAt(centered_content_text_x(layout, title, panel.titleSize, panel.titleFont),
                         panel.titleY,
                         title,
                         kUiHighlight,
                         kUiModeAreaBg,
                         panel.titleSize,
                         panel.titleFont);
}

void draw_content_stage_line(const ContentTextPanelLayout &panel, const String &text, int x, int y, bool selected) {
  const int highlightPadX = (panel.textSize > 1) ? 10 : 6;
  const int highlightPadY = (panel.textSize > 1) ? 4 : 2;
  if (selected) {
    uiDisplayFillRect(x - highlightPadX,
                      y - highlightPadY,
                      uiDisplayTextWidth(text, panel.textSize, panel.textFont) + (highlightPadX * 2),
                      panel.textH + (highlightPadY * 2),
                      kUiHighlight);
  }
  uiDisplayPrintStyledAt(x,
                         y,
                         text,
                         selected ? kUiDark : kUiText,
                         selected ? kUiHighlight : kUiModeAreaBg,
                         panel.textSize,
                         panel.textFont);
}

void render_bc_mode_content(const ManagedZoneLayout &layout,
                            bool refreshAll,
                            const String &title,
                            const String &statusLabel,
                            const String &statusValue,
                            uint16_t statusColor,
                            const String &row2Left,
                            const String &row2Right,
                            const String &row3Left,
                            const String &row3Right) {
  const ContentTextPanelLayout panel = compute_content_text_panel_layout(layout);
  const int statusPrefixW = uiDisplayTextWidth(statusLabel, panel.textSize, panel.textFont);
  const int statusValueW = uiDisplayTextWidth(statusValue, panel.textSize, panel.textFont);
  const int statusGap = uiDisplayTextWidth(" ", panel.textSize, panel.textFont);
  const int statusTotalW = statusPrefixW + statusGap + statusValueW;
  const int statusX = (layout.displayW - statusTotalW) / 2;
  const int pairGap = layout.isLargeDisplay ? 28 : 18;
  const int row2LeftW = uiDisplayTextWidth(row2Left, panel.textSize, panel.textFont);
  const int row2TotalW = row2LeftW + pairGap + uiDisplayTextWidth(row2Right, panel.textSize, panel.textFont);
  const int row2StartX = (layout.displayW - row2TotalW) / 2;
  const int row2RightX = row2StartX + row2LeftW + pairGap;
  const int row3LeftW = uiDisplayTextWidth(row3Left, panel.textSize, panel.textFont);
  const int row3TotalW = row3LeftW + pairGap + uiDisplayTextWidth(row3Right, panel.textSize, panel.textFont);
  const int row3StartX = (layout.displayW - row3TotalW) / 2;
  const int row3RightX = row3StartX + row3LeftW + pairGap;

  if (refreshAll) {
    clear_blue_content_zone(layout);
    draw_content_panel_title(layout, panel, title);
    g_ccLastModeLine1 = "";
    g_ccLastModeLine2 = "";
    g_ccLastModeLine3 = "";
    g_ccLastModeLine4 = "";
    g_ccLastModeLine5 = "";
    g_ccLastBatteryStatus = "";
  }

  if (refreshAll || g_ccLastModeLine1 != statusLabel || g_ccLastBatteryStatus != statusValue || g_ccLastBatteryStatusColor != statusColor) {
    uiDisplayFillRect(1, panel.row1Y - 1, layout.displayW - 2, panel.textH + 2, kUiModeAreaBg);
    uiDisplayPrintStyledAt(statusX, panel.row1Y, statusLabel, kUiText, kUiModeAreaBg, panel.textSize, panel.textFont);
    uiDisplayPrintStyledAt(statusX + statusPrefixW + statusGap,
                           panel.row1Y,
                           statusValue,
                           statusColor,
                           kUiModeAreaBg,
                           panel.textSize,
                           panel.textFont);
    g_ccLastModeLine1 = statusLabel;
    g_ccLastBatteryStatus = statusValue;
    g_ccLastBatteryStatusColor = statusColor;
  }

  if (refreshAll || g_ccLastModeLine2 != row2Right || g_ccLastModeLine3 != row2Left) {
    uiDisplayFillRect(1, panel.row2Y - 1, layout.displayW - 2, panel.textH + 2, kUiModeAreaBg);
    uiDisplayPrintStyledAt(row2StartX, panel.row2Y, row2Left, kUiText, kUiModeAreaBg, panel.textSize, panel.textFont);
    uiDisplayPrintStyledAt(row2RightX, panel.row2Y, row2Right, kUiText, kUiModeAreaBg, panel.textSize, panel.textFont);
    g_ccLastModeLine2 = row2Right;
    g_ccLastModeLine3 = row2Left;
  }

  if (refreshAll || g_ccLastModeLine4 != row3Left || g_ccLastModeLine5 != row3Right) {
    uiDisplayFillRect(1, panel.row3Y - 1, layout.displayW - 2, panel.textH + 2, kUiModeAreaBg);
    uiDisplayPrintStyledAt(row3StartX, panel.row3Y, row3Left, kUiText, kUiModeAreaBg, panel.textSize, panel.textFont);
    uiDisplayPrintStyledAt(row3RightX, panel.row3Y, row3Right, kUiText, kUiModeAreaBg, panel.textSize, panel.textFont);
    g_ccLastModeLine4 = row3Left;
    g_ccLastModeLine5 = row3Right;
  }
}

void render_transient_mode_content(const ManagedZoneLayout &layout,
                                   bool refreshAll,
                                   const String &title,
                                   const String &row1,
                                   const String &row2Left,
                                   const String &row2Right,
                                   const String &row3) {
  const ContentTextPanelLayout panel = compute_content_text_panel_layout(layout);
  const int row1X = centered_content_text_x(layout, row1, panel.textSize, panel.textFont);
  const int pairGap = layout.isLargeDisplay ? 28 : 18;
  const int row2LeftW = uiDisplayTextWidth(row2Left, panel.textSize, panel.textFont);
  const int row2RightW = uiDisplayTextWidth(row2Right, panel.textSize, panel.textFont);
  const int row2TotalW = row2LeftW + pairGap + row2RightW;
  const int row2StartX = (layout.displayW - row2TotalW) / 2;
  const int row2RightX = row2StartX + row2LeftW + pairGap;
  const int row3X = centered_content_text_x(layout, row3, panel.textSize, panel.textFont);

  if (refreshAll) {
    clear_blue_content_zone(layout);
    draw_content_panel_title(layout, panel, title);
    g_ccLastModeLine1 = "";
    g_ccLastModeLine2 = "";
    g_ccLastModeLine3 = "";
    g_ccLastModeLine4 = "";
    g_ccLastModeLine5 = "";
  }

  if (refreshAll || g_ccLastModeLine1 != row1) {
    uiDisplayFillRect(1, panel.row1Y - 1, layout.displayW - 2, panel.textH + 2, kUiModeAreaBg);
    uiDisplayPrintStyledAt(row1X, panel.row1Y, row1, kUiText, kUiModeAreaBg, panel.textSize, panel.textFont);
    g_ccLastModeLine1 = row1;
  }

  if (refreshAll || g_ccLastModeLine2 != row2Right || g_ccLastModeLine3 != row2Left) {
    uiDisplayFillRect(1, panel.row2Y - 1, layout.displayW - 2, panel.textH + 2, kUiModeAreaBg);
    uiDisplayPrintStyledAt(row2StartX, panel.row2Y, row2Left, kUiText, kUiModeAreaBg, panel.textSize, panel.textFont);
    uiDisplayPrintStyledAt(row2RightX, panel.row2Y, row2Right, kUiText, kUiModeAreaBg, panel.textSize, panel.textFont);
    g_ccLastModeLine2 = row2Right;
    g_ccLastModeLine3 = row2Left;
  }

  if (refreshAll || g_ccLastModeLine4 != row3) {
    uiDisplayFillRect(1, panel.row3Y - 1, layout.displayW - 2, panel.textH + 2, kUiModeAreaBg);
    uiDisplayPrintStyledAt(row3X, panel.row3Y, row3, kUiText, kUiModeAreaBg, panel.textSize, panel.textFont);
    g_ccLastModeLine4 = row3;
  }
}

void render_calibration_mode_content(const ManagedZoneLayout &layout,
                                     bool refreshAll,
                                     const String &title,
                                     const String &row1,
                                     const String &row2,
                                     const String &row3) {
  const ContentTextPanelLayout panel = compute_content_text_panel_layout(layout);
  const int row1X = centered_content_text_x(layout, row1, panel.textSize, panel.textFont);
  const int row2X = centered_content_text_x(layout, row2, panel.textSize, panel.textFont);
  const int row3X = centered_content_text_x(layout, row3, panel.textSize, panel.textFont);

  if (refreshAll) {
    clear_blue_content_zone(layout);
    draw_content_panel_title(layout, panel, title);
  }

  if (refreshAll || g_ccLastModeLine1 != row1) {
    uiDisplayFillRect(1, panel.row1Y - 1, layout.displayW - 2, panel.textH + 2, kUiModeAreaBg);
    uiDisplayPrintStyledAt(row1X, panel.row1Y, row1, kUiText, kUiModeAreaBg, panel.textSize, panel.textFont);
    g_ccLastModeLine1 = row1;
  }
  if (refreshAll || g_ccLastModeLine2 != row2) {
    uiDisplayFillRect(1, panel.row2Y - 1, layout.displayW - 2, panel.textH + 2, kUiModeAreaBg);
    uiDisplayPrintStyledAt(row2X, panel.row2Y, row2, kUiText, kUiModeAreaBg, panel.textSize, panel.textFont);
    g_ccLastModeLine2 = row2;
  }
  if (refreshAll || g_ccLastModeLine3 != row3) {
    uiDisplayFillRect(1, panel.row3Y - 1, layout.displayW - 2, panel.textH + 2, kUiModeAreaBg);
    uiDisplayPrintStyledAt(row3X, panel.row3Y, row3, kUiText, kUiModeAreaBg, panel.textSize, panel.textFont);
    g_ccLastModeLine3 = row3;
  }
  g_ccLastModeLine4 = "";
  g_ccLastModeLine5 = "";
}

TracePlotLayout compute_trace_plot_layout(const ManagedZoneLayout &layout) {
  const int leftPad = layout.isLargeDisplay ? 24 : 18;
  const int rightPad = leftPad;
  const int topPad = layout.isLargeDisplay ? 18 : 14;
  const int bottomPad = layout.isLargeDisplay ? 18 : 14;
  const int plotX = 1 + leftPad;
  const int plotY = (layout.contentY + 1) + topPad;
  const int plotW = max(10, (layout.displayW - 2) - leftPad - rightPad);
  const int plotH = max(10, (layout.contentH - 2) - topPad - bottomPad);
  return {leftPad, rightPad, topPad, bottomPad, plotX, plotY, plotW, plotH};
}

void update_trace_overlay_content(const ManagedZoneLayout &layout, bool layoutChanged, bool traceOverlayChanged, uint32_t traceToken) {
  const size_t sampleCount = app_trace_sample_count();
  float maxCurrentA = 0.0f;
  float maxVoltageV = 0.0f;
  float currentA = 0.0f;
  float voltageV = 0.0f;
  for (size_t i = 0; i < sampleCount; ++i) {
    if (!app_trace_read_sample(i, &currentA, &voltageV)) continue;
    maxCurrentA = max(maxCurrentA, currentA);
    maxVoltageV = max(maxVoltageV, voltageV);
  }

  const float nextTimeScale = trace_time_scale_seconds(app_trace_duration_seconds());
  const float nextCurrentScale = trace_current_scale_max(maxCurrentA);
  const float nextVoltageScale = trace_voltage_scale_max(maxVoltageV);
  const bool scalesChanged = layoutChanged || traceOverlayChanged || g_ccLastTraceTimeScaleSeconds != nextTimeScale ||
                             g_ccLastTraceCurrentScaleMax != nextCurrentScale ||
                             g_ccLastTraceVoltageScaleMax != nextVoltageScale || sampleCount < g_ccLastTraceSampleCount;

  if (scalesChanged) {
    draw_cc_trace_graph(1, layout.contentY + 1, layout.displayW - 2, layout.contentH - 2, layout.isLargeDisplay);
    draw_zone_borders(layout);
  } else if (sampleCount > 0 && sampleCount > g_ccLastTraceSampleCount) {
    const TracePlotLayout plot = compute_trace_plot_layout(layout);
    const size_t startIndex = (g_ccLastTraceSampleCount > 0) ? (g_ccLastTraceSampleCount - 1) : 0;
    draw_cc_trace_segment_range(plot.plotX,
                                plot.plotY,
                                plot.plotW,
                                plot.plotH,
                                app_trace_effective_interval_ms(),
                                nextTimeScale,
                                nextCurrentScale,
                                nextVoltageScale,
                                tft.color565(0, 180, 0),
                                TFT_RED,
                                startIndex,
                                sampleCount - 1);
    g_ccLastTraceSampleCount = sampleCount;
    g_ccLastTraceTimeScaleSeconds = nextTimeScale;
    g_ccLastTraceCurrentScaleMax = nextCurrentScale;
    g_ccLastTraceVoltageScaleMax = nextVoltageScale;
  }
  g_ccLastTraceToken = traceToken;
}

void draw_shared_chrome(const ManagedZoneLayout &layout,
                        const String &label,
                        bool loadEnabled,
                        bool clearContentZone,
                        bool drawMetricStub = true) {
  draw_top_status_zone(layout, current_top_status_zone_state(label, loadEnabled));
  if (drawMetricStub) {
    draw_metrics_zone(layout, metrics_zone_stub_state());
  }
  if (clearContentZone) {
    uiDisplayFillRect(1, layout.contentY + 1, layout.displayW - 2, layout.contentH - 1, kUiModeAreaBg);
  }
  draw_footer_zone(layout, current_footer_zone_state());
  draw_zone_borders(layout);
}

void draw_config_chrome(bool clearContentZone) {
  const ManagedZoneLayout layout = managed_zone_layout();
  const uint8_t mode = app_mode_state_mode();
  const char *modeLabel = (mode == CC) ? "CC" : (mode == CP) ? "CP" : (mode == CR) ? "CR" : (mode == BC) ? "BC" : (mode == TC) ? "TC" : (mode == TL) ? "TL" : "CA";
  draw_shared_chrome(layout, modeLabel, false, clearContentZone, true);
}

void draw_config_footer_time_only() {
  const ManagedZoneLayout layout = managed_zone_layout();
  draw_footer_zone(layout, current_footer_zone_state());
  draw_horizontal_separator(layout.footerY, layout.displayW, kUiBorder);
  tft.drawLine(0, layout.displayH - 1, layout.displayW - 1, layout.displayH - 1, kUiBorder);
  tft.drawLine(0, layout.footerY, 0, layout.displayH - 1, kUiBorder);
  tft.drawLine(layout.displayW - 1, layout.footerY, layout.displayW - 1, layout.displayH - 1, kUiBorder);
}

void draw_accent_chrome_temp_only() {
  const ManagedZoneLayout layout = managed_zone_layout();
  draw_top_status_zone(layout, current_top_status_zone_state(String((app_mode_state_mode() == CC) ? "CC" : (app_mode_state_mode() == CP) ? "CP" : (app_mode_state_mode() == CR) ? "CR" : (app_mode_state_mode() == BC) ? "BC" : (app_mode_state_mode() == TC) ? "TC" : (app_mode_state_mode() == TL) ? "TL" : "CA"), false));
  draw_horizontal_separator(layout.topBarH, layout.displayW, kUiBorder);
}

void clear_config_content_zone() {
  const ManagedZoneLayout layout = managed_zone_layout();
  uiDisplayFillRect(1, layout.contentY + 1, layout.displayW - 2, layout.contentH - 1, kUiModeAreaBg);
}

}  // namespace

void uiDisplayUpdateConfigChrome(void) {
  draw_config_chrome(false);
}

void uiDisplayUpdateConfigFooterTime(void) {
  draw_config_footer_time_only();
}

void uiDisplayUpdateAccentChromeStatus(void) {
  draw_accent_chrome_temp_only();
  draw_config_footer_time_only();
}

void draw_config_title(const ManagedZoneLayout &layout, const String &title, uint8_t textSize = 0) {
  const uint8_t font = 2;
  const uint8_t size = (textSize != 0) ? textSize : (layout.isLargeDisplay ? 2 : 1);
  const int y = layout.contentY + max(8, layout.isLargeDisplay ? 12 : 8);
  uiDisplayPrintStyledAt((layout.displayW - uiDisplayTextWidth(title, size, font)) / 2,
                         y,
                         title,
                         kUiHighlight,
                         kUiModeAreaBg,
                         size,
                         font);
}

void draw_config_line(const ManagedZoneLayout &layout, int y, const String &text, bool selected, uint8_t textSize = 0) {
  const uint8_t font = 2;
  const uint8_t size = (textSize != 0) ? textSize : (layout.isLargeDisplay ? 2 : 1);
  const int x = (layout.displayW - uiDisplayTextWidth(text, size, font)) / 2;
  const int h = uiDisplayFontHeight(size, font);
  if (selected) {
    uiDisplayFillRect(x - 4, y - 1, uiDisplayTextWidth(text, size, font) + 8, h + 2, kUiHighlight);
  }
  uiDisplayPrintStyledAt(x,
                         y,
                         text,
                         selected ? kUiDark : kUiText,
                         selected ? kUiHighlight : kUiModeAreaBg,
                         size,
                         font);
}

void draw_config_value_edit_line(const ManagedZoneLayout &layout,
                                 int y,
                                 const String &label,
                                 const String &value,
                                 bool selected,
                                 bool editingValue,
                                 uint8_t textSize = 0) {
  const uint8_t font = 2;
  const uint8_t size = (textSize != 0) ? textSize : (layout.isLargeDisplay ? 2 : 1);
  const int labelW = uiDisplayTextWidth(label, size, font);
  const int valueW = uiDisplayTextWidth(value, size, font);
  const int totalW = labelW + valueW;
  const int x = (layout.displayW - totalW) / 2;
  const int h = uiDisplayFontHeight(size, font);

  if (editingValue) {
    uiDisplayFillRect(x + labelW - 4, y - 1, valueW + 8, h + 2, kUiSetColor);
  } else if (selected) {
    uiDisplayFillRect(x - 4, y - 1, totalW + 8, h + 2, kUiHighlight);
  }

  uiDisplayPrintStyledAt(x,
                         y,
                         label,
                         selected && !editingValue ? kUiDark : kUiText,
                         selected && !editingValue ? kUiHighlight : kUiModeAreaBg,
                         size,
                         font);
  uiDisplayPrintStyledAt(x + labelW,
                         y,
                         value,
                         editingValue || selected ? kUiDark : kUiText,
                         editingValue ? kUiSetColor : (selected ? kUiHighlight : kUiModeAreaBg),
                         size,
                         font);
}

void draw_config_value_edit_item(int x,
                                 int y,
                                 const String &label,
                                 const String &value,
                                 bool selected,
                                 bool editingValue,
                                 uint8_t textSize = 1) {
  const uint8_t font = 2;
  const int labelW = uiDisplayTextWidth(label, textSize, font);
  const int valueW = uiDisplayTextWidth(value, textSize, font);
  const int totalW = labelW + valueW;
  const int h = uiDisplayFontHeight(textSize, font);

  if (editingValue) {
    uiDisplayFillRect(x + labelW - 4, y - 1, valueW + 8, h + 2, kUiSetColor);
  } else if (selected) {
    uiDisplayFillRect(x - 4, y - 1, totalW + 8, h + 2, kUiHighlight);
  }

  uiDisplayPrintStyledAt(x,
                         y,
                         label,
                         selected && !editingValue ? kUiDark : kUiText,
                         selected && !editingValue ? kUiHighlight : kUiModeAreaBg,
                         textSize,
                         font);
  uiDisplayPrintStyledAt(x + labelW,
                         y,
                         value,
                         editingValue || selected ? kUiDark : kUiText,
                         editingValue ? kUiSetColor : (selected ? kUiHighlight : kUiModeAreaBg),
                         textSize,
                         font);
}

void draw_config_status_item(int x,
                             int y,
                             const String &label,
                             const String &value,
                             bool selected,
                             bool active,
                             uint8_t textSize = 1) {
  const uint8_t font = 2;
  const int labelW = uiDisplayTextWidth(label, textSize, font);
  const int valueW = uiDisplayTextWidth(value, textSize, font);
  const int totalW = labelW + valueW;
  const int h = uiDisplayFontHeight(textSize, font);
  const uint16_t selectedBg = active ? kUiLoadOn : kUiHighlight;
  const uint16_t selectedValueFg = active ? kUiText : kUiDark;

  if (selected) {
    uiDisplayFillRect(x - 4, y - 1, totalW + 8, h + 2, kUiHighlight);
    uiDisplayFillRect(x + labelW - 2, y - 1, valueW + 6, h + 2, selectedBg);
  }

  uiDisplayPrintStyledAt(x,
                         y,
                         label,
                         selected ? kUiDark : kUiText,
                         selected ? kUiHighlight : kUiModeAreaBg,
                         textSize,
                         font);
  uiDisplayPrintStyledAt(x + labelW,
                         y,
                         value,
                         selected ? selectedValueFg : (active ? kUiText : kUiText),
                         selected ? selectedBg : (active ? kUiLoadOn : kUiModeAreaBg),
                         textSize,
                         font);
}

void draw_config_column_item(const ManagedZoneLayout &layout, int x, int y, const String &text, bool selected, uint8_t textSize = 1) {
  const uint8_t font = 2;
  const int h = uiDisplayFontHeight(textSize, font);
  if (selected) {
    uiDisplayFillRect(x - 4, y - 1, uiDisplayTextWidth(text, textSize, font) + 8, h + 2, kUiHighlight);
  }
  uiDisplayPrintStyledAt(x,
                         y,
                         text,
                         selected ? kUiDark : kUiText,
                         selected ? kUiHighlight : kUiModeAreaBg,
                         textSize,
                         font);
}

void compute_two_column_positions(const ManagedZoneLayout &layout, int leftWidth, int rightWidth, int gap, int &leftX, int &rightX) {
  const int totalWidth = leftWidth + gap + rightWidth;
  leftX = (layout.displayW - totalWidth) / 2;
  rightX = leftX + leftWidth + gap;
}

String format_limit_current(float value) {
  return String(value, 3) + " A";
}

String format_limit_power(float value) {
  return String(value, 1) + " W";
}

String format_limit_temp(float value) {
  return String(static_cast<int>(value)) + " C";
}

void uiDisplayRenderConfigRootMenu(const UiViewState &state) {
  const ManagedZoneLayout layout = managed_zone_layout();
  const uint8_t titleSize = layout.isLargeDisplay ? 2 : 1;
  const uint8_t itemSize = layout.isLargeDisplay ? 2 : 1;
  const uint8_t font = 2;
  const int titleY = layout.contentY + (layout.isLargeDisplay ? 10 : 8);
  const int lineHeight = uiDisplayFontHeight(itemSize, font);
  const int gap = layout.isLargeDisplay ? 4 : 8;
  const int startY = titleY + uiDisplayFontHeight(titleSize, font) + (layout.isLargeDisplay ? 8 : 12);
  const int columnGap = layout.isLargeDisplay ? 48 : 32;
  int leftX = 0;
  int rightX = 0;
  compute_two_column_positions(layout,
                               uiDisplayTextWidth("2 Calibration", itemSize, font),
                               uiDisplayTextWidth("4 Clock", itemSize, font),
                               columnGap,
                               leftX,
                               rightX);

  clear_config_content_zone();
  draw_config_title(layout, "CONFIGURATION", titleSize);
  draw_config_column_item(layout, leftX, startY, "1 Protection", state.menuRootSelection == 0, itemSize);
  draw_config_column_item(layout, leftX, startY + (lineHeight + gap), "2 Calibration", state.menuRootSelection == 1, itemSize);
  draw_config_column_item(layout, leftX, startY + ((lineHeight + gap) * 2), "3 FW Update", state.menuRootSelection == 2, itemSize);
  draw_config_column_item(layout, rightX, startY, "4 Clock", state.menuRootSelection == 3, itemSize);
  draw_config_column_item(layout, rightX, startY + (lineHeight + gap), "5 Exit", state.menuRootSelection == 4, itemSize);
  draw_battery_setup_set_zone("", "");
}

void uiDisplayRenderProtectionMenu(const UiViewState &state) {
  const ManagedZoneLayout layout = managed_zone_layout();
  const uint8_t titleSize = layout.isLargeDisplay ? 2 : 1;
  const uint8_t itemSize = layout.isLargeDisplay ? 2 : 1;
  const uint8_t font = 2;
  const int titleY = layout.contentY + (layout.isLargeDisplay ? 10 : 8);
  const int gap = layout.isLargeDisplay ? 4 : 8;
  const int lineHeight = uiDisplayFontHeight(itemSize, font);
  const int startY = titleY + uiDisplayFontHeight(titleSize, font) + (layout.isLargeDisplay ? 8 : 12);
  const int columnGap = layout.isLargeDisplay ? 48 : 32;
  int leftX = 0;
  int rightX = 0;
  compute_two_column_positions(layout,
                               uiDisplayTextWidth("2 Fan", itemSize, font),
                               uiDisplayTextWidth("3 Back", itemSize, font),
                               columnGap,
                               leftX,
                               rightX);

  clear_config_content_zone();
  draw_config_title(layout, "PROTECTION", titleSize);
  draw_config_column_item(layout, leftX, startY, "1 Limits", state.protectionMenuSelection == 0, itemSize);
  draw_config_column_item(layout, leftX, startY + (lineHeight + gap), "2 Fan", state.protectionMenuSelection == 1, itemSize);
  draw_config_column_item(layout, rightX, startY, "3 Back", state.protectionMenuSelection == 2, itemSize);
  draw_battery_setup_set_zone("", "");
}

void uiDisplayRenderUpdateMenu(const UiViewState &state) {
  const ManagedZoneLayout layout = managed_zone_layout();
  const uint8_t titleSize = layout.isLargeDisplay ? 2 : 1;
  const uint8_t itemSize = layout.isLargeDisplay ? 2 : 1;
  const uint8_t font = 2;
  const int titleY = layout.contentY + (layout.isLargeDisplay ? 10 : 8);
  const int lineHeight = uiDisplayFontHeight(itemSize, font);
  const int startY = titleY + uiDisplayFontHeight(titleSize, font) + (layout.isLargeDisplay ? 8 : 12);
  const int columnGap = layout.isLargeDisplay ? 48 : 32;
  int leftX = 0;
  int rightX = 0;
  compute_two_column_positions(layout,
                               uiDisplayTextWidth("1 Start OTA", itemSize, font),
                               uiDisplayTextWidth("2 Back", itemSize, font),
                               columnGap,
                               leftX,
                               rightX);

  clear_config_content_zone();
  draw_config_title(layout, "FW UPDATE", titleSize);
  draw_config_column_item(layout, leftX, startY, "1 Start OTA", state.updateMenuSelection == 0, itemSize);
  draw_config_column_item(layout, rightX, startY, "2 Back", state.updateMenuSelection == 1, itemSize);
  draw_battery_setup_set_zone("", "");
}

void uiDisplayRenderFwUpdateScreen(const char *statusLine, const char *detailLine, const char *hintLine) {
  const int displayW = uiDisplayWidthPx();
  const int displayH = uiDisplayHeightPx();
  const bool isLargeDisplay = displayW >= 400;
  const uint8_t titleSize = isLargeDisplay ? 2 : 1;
  const uint8_t textSize = isLargeDisplay ? 2 : 1;
  const uint8_t font = 2;
  const int outerPad = isLargeDisplay ? 10 : 6;
  const int panelX = outerPad;
  const int panelY = outerPad;
  const int panelW = displayW - (outerPad * 2);
  const int panelH = displayH - (outerPad * 2);
  const int lineHeight = uiDisplayFontHeight(textSize, font);
  const int titleHeight = uiDisplayFontHeight(titleSize, font);
  const int gap = isLargeDisplay ? 12 : 8;
  const String status = statusLine ? String(statusLine) : "";
  const String detail = detailLine ? String(detailLine) : "";
  const String hint = hintLine ? String(hintLine) : "";
  const bool uploading = app_ota_is_uploading();
  const bool error = app_ota_has_error();
  const int totalTextHeight = titleHeight + (lineHeight * 3) + (gap * 3);
  const int titleY = panelY + max(isLargeDisplay ? 12 : 8, (panelH - totalTextHeight) / 2);
  const int startY = titleY + titleHeight + gap;
  const bool layoutChanged = !g_fwUpdateLayoutDrawn || g_fwUpdateLastDisplayW != displayW || g_fwUpdateLastDisplayH != displayH;

  if (layoutChanged) {
    uiDisplayClear();
    uiDisplayFillRect(panelX, panelY, panelW, panelH, kUiModeAreaBg);
    uiDisplayDrawRect(panelX, panelY, panelW, panelH, kUiBorder);
    const String title = uploading ? "OTA ACTIVE" : "FW UPDATE";
    uiDisplayPrintStyledAt((displayW - uiDisplayTextWidth(title, titleSize, font)) / 2,
                           titleY,
                           title,
                           uploading ? kUiAlertText : kUiHighlight,
                           kUiModeAreaBg,
                           titleSize,
                           font);
    g_fwUpdateLayoutDrawn = true;
    g_fwUpdateLastDisplayW = displayW;
    g_fwUpdateLastDisplayH = displayH;
    g_fwUpdateLastStatus = "";
    g_fwUpdateLastDetail = "";
    g_fwUpdateLastHint = "";
  }

  const int textBlockX = panelX + 10;
  const int textBlockW = panelW - 20;
  const int statusY = startY;
  const int detailY = startY + (lineHeight + gap);
  const int hintY = startY + ((lineHeight + gap) * 2);

  if (g_fwUpdateLastStatus != status) {
    uiDisplayFillRect(textBlockX, statusY - 3, textBlockW, lineHeight + 8, kUiModeAreaBg);
    if (uploading || error) {
      const int statusW = uiDisplayTextWidth(status, textSize, font);
      const int statusX = (displayW - statusW) / 2;
      const int statusH = uiDisplayFontHeight(textSize, font);
      const uint16_t statusBg = error ? kUiAlertBg : tft.color565(180, 0, 0);
      uiDisplayFillRect(statusX - 6, statusY - 2, statusW + 12, statusH + 4, statusBg);
      uiDisplayPrintStyledAt(statusX, statusY, status, kUiAlertText, statusBg, textSize, font);
    } else {
      uiDisplayPrintStyledAt((displayW - uiDisplayTextWidth(status, textSize, font)) / 2,
                             statusY,
                             status,
                             kUiText,
                             kUiModeAreaBg,
                             textSize,
                             font);
    }
    g_fwUpdateLastStatus = status;
  }

  if (g_fwUpdateLastDetail != detail) {
    uiDisplayFillRect(textBlockX, detailY - 3, textBlockW, lineHeight + 8, kUiModeAreaBg);
    uiDisplayPrintStyledAt((displayW - uiDisplayTextWidth(detail, textSize, font)) / 2,
                           detailY,
                           detail,
                           kUiText,
                           kUiModeAreaBg,
                           textSize,
                           font);
    g_fwUpdateLastDetail = detail;
  }

  if (g_fwUpdateLastHint != hint) {
    uiDisplayFillRect(textBlockX, hintY - 3, textBlockW, lineHeight + 8, kUiModeAreaBg);
    const uint16_t hintBg = (uploading || error) ? kUiHighlight : kUiModeAreaBg;
    const uint16_t hintFg = (uploading || error) ? kUiDark : kUiText;
    uiDisplayPrintStyledAt((displayW - uiDisplayTextWidth(hint, textSize, font)) / 2,
                           hintY,
                           hint,
                           hintFg,
                           hintBg,
                           textSize,
                           font);
    g_fwUpdateLastHint = hint;
  }
}

void uiDisplayRenderFanSettingsMenu(const UiViewState &state) {
  const ManagedZoneLayout layout = managed_zone_layout();
  const uint8_t titleSize = layout.isLargeDisplay ? 2 : 1;
  const uint8_t itemSize = layout.isLargeDisplay ? 2 : 1;
  const uint8_t font = 2;
  const int titleY = layout.contentY + (layout.isLargeDisplay ? 10 : 8);
  const int lineHeight = uiDisplayFontHeight(itemSize, font);
  const int gap = layout.isLargeDisplay ? 4 : 6;
  const int startY = titleY + uiDisplayFontHeight(titleSize, font) + (layout.isLargeDisplay ? 8 : 10);
  const int columnGap = layout.isLargeDisplay ? 48 : 32;
  const int leftWidth = max(uiDisplayTextWidth("1 Temp: 100C", itemSize, font),
                            max(uiDisplayTextWidth("2 Hold: 255s", itemSize, font),
                                uiDisplayTextWidth("3 Fan: OFF", itemSize, font)));
  int leftX = 0;
  int rightX = 0;
  compute_two_column_positions(layout,
                               leftWidth,
                               uiDisplayTextWidth("4 Back", itemSize, font),
                               columnGap,
                               leftX,
                               rightX);
  const String tempValue = (state.fanEditActive && state.fanSettingsMenuSelection == 0 && state.fanInputText[0] != '\0')
                               ? String(state.fanInputText)
                               : String(static_cast<int>(state.fanDraftTempC));
  const String holdValue = (state.fanEditActive && state.fanSettingsMenuSelection == 1 && state.fanInputText[0] != '\0')
                               ? String(state.fanInputText)
                               : String(static_cast<int>(state.fanDraftHoldSeconds));
  const String fanState = state.fanOutputOn ? "ON" : "OFF";

  clear_config_content_zone();
  draw_config_title(layout, "FAN SETTINGS", titleSize);
  draw_config_value_edit_item(leftX,
                              startY,
                              "1 Temp: ",
                              tempValue + "C",
                              state.fanSettingsMenuSelection == 0,
                              state.fanEditActive && state.fanSettingsMenuSelection == 0,
                              itemSize);
  draw_config_value_edit_item(leftX,
                              startY + (lineHeight + gap),
                              "2 Hold: ",
                              holdValue + "s",
                              state.fanSettingsMenuSelection == 1,
                              state.fanEditActive && state.fanSettingsMenuSelection == 1,
                              itemSize);
  draw_config_status_item(leftX,
                          startY + ((lineHeight + gap) * 2),
                          "3 Fan: ",
                          fanState,
                          state.fanSettingsMenuSelection == 2,
                          state.fanOutputOn,
                          itemSize);
  draw_config_column_item(layout, rightX, startY, "4 Back", state.fanSettingsMenuSelection == 3, itemSize);
  draw_battery_setup_set_zone("SET> ", state.fanEditActive ? String(state.fanInputText) : String(""));
}

void uiDisplayRenderLimitsMenu(const UiViewState &state) {
  const ManagedZoneLayout layout = managed_zone_layout();
  const uint8_t titleSize = layout.isLargeDisplay ? 2 : 1;
  const uint8_t itemSize = layout.isLargeDisplay ? 2 : 1;
  const uint8_t font = 2;
  const int titleY = layout.contentY + (layout.isLargeDisplay ? 10 : 8);
  const int lineHeight = uiDisplayFontHeight(itemSize, font);
  const int gap = layout.isLargeDisplay ? 4 : 6;
  const int startY = titleY + uiDisplayFontHeight(titleSize, font) + (layout.isLargeDisplay ? 8 : 10);
  const String currValue = (state.limitsEditActive && state.limitsMenuField == 0 && state.limitsInputText[0] != '\0')
                               ? String(state.limitsInputText)
                               : format_limit_current(state.limitsDraftCurrentA);
  const String powerValue = (state.limitsEditActive && state.limitsMenuField == 1 && state.limitsInputText[0] != '\0')
                                ? String(state.limitsInputText)
                                : format_limit_power(state.limitsDraftPowerW);
  const String tempValue = (state.limitsEditActive && state.limitsMenuField == 2 && state.limitsInputText[0] != '\0')
                               ? String(state.limitsInputText)
                               : format_limit_temp(state.limitsDraftTempC);

  clear_config_content_zone();
  draw_config_title(layout, "LIMITS", titleSize);
  draw_config_value_edit_line(layout,
                              startY,
                              "1 Current: ",
                              currValue,
                              state.limitsMenuField == 0,
                              state.limitsEditActive && state.limitsMenuField == 0,
                              itemSize);
  draw_config_value_edit_line(layout,
                              startY + (lineHeight + gap),
                              "2 Power: ",
                              powerValue,
                              state.limitsMenuField == 1,
                              state.limitsEditActive && state.limitsMenuField == 1,
                              itemSize);
  draw_config_value_edit_line(layout,
                              startY + ((lineHeight + gap) * 2),
                              "3 Temp: ",
                              tempValue,
                              state.limitsMenuField == 2,
                              state.limitsEditActive && state.limitsMenuField == 2,
                              itemSize);
  draw_battery_setup_set_zone("SET> ", state.limitsEditActive ? String(state.limitsInputText) : String(""));
}

void uiDisplayRenderCalibrationMenu(const UiViewState &state) {
  const ManagedZoneLayout layout = managed_zone_layout();
  const uint8_t titleSize = layout.isLargeDisplay ? 2 : 1;
  const uint8_t itemSize = layout.isLargeDisplay ? 2 : 1;
  const uint8_t font = 2;
  const int titleY = layout.contentY + (layout.isLargeDisplay ? 10 : 8);
  const int lineHeight = uiDisplayFontHeight(itemSize, font);
  const int gap = layout.isLargeDisplay ? 4 : 6;
  const int startY = titleY + uiDisplayFontHeight(titleSize, font) + (layout.isLargeDisplay ? 8 : 10);
  const int columnGap = layout.isLargeDisplay ? 48 : 32;
  int leftX = 0;
  int rightX = 0;
  compute_two_column_positions(layout,
                               uiDisplayTextWidth("3 Temp", itemSize, font),
                               uiDisplayTextWidth("6 Back", itemSize, font),
                               columnGap,
                               leftX,
                               rightX);
  const uint8_t selected = (state.calibrationMenuOption > 0) ? static_cast<uint8_t>(state.calibrationMenuOption - 1) : 0;

  clear_config_content_zone();
  draw_config_title(layout, "CALIBRATION", titleSize);
  draw_config_column_item(layout, leftX, startY, "1 Voltage", selected == 0, itemSize);
  draw_config_column_item(layout, leftX, startY + (lineHeight + gap), "2 Current", selected == 1, itemSize);
  draw_config_column_item(layout, leftX, startY + ((lineHeight + gap) * 2), "3 Temp", selected == 2, itemSize);
  draw_config_column_item(layout, rightX, startY, "4 Load", selected == 3, itemSize);
  draw_config_column_item(layout, rightX, startY + (lineHeight + gap), "5 Save", selected == 4, itemSize);
  draw_config_column_item(layout, rightX, startY + ((lineHeight + gap) * 2), "6 Back", selected == 5, itemSize);
  draw_battery_setup_set_zone("", "");
}

void uiDisplayRenderClockMenu(const UiViewState &state) {
  const ManagedZoneLayout layout = managed_zone_layout();
  const uint8_t titleSize = layout.isLargeDisplay ? 2 : 1;
  const uint8_t itemSize = layout.isLargeDisplay ? 2 : 1;
  const uint8_t font = 2;
  const int titleY = layout.contentY + (layout.isLargeDisplay ? 10 : 8);
  const int lineHeight = uiDisplayFontHeight(itemSize, font);
  const int gap = layout.isLargeDisplay ? 4 : 6;
  const int startY = titleY + uiDisplayFontHeight(titleSize, font) + (layout.isLargeDisplay ? 8 : 10);
  const String dayValue = (state.clockEditActive && state.clockMenuSelection == 0 && state.clockInputText[0] != '\0')
                              ? String(state.clockInputText)
                              : two_digits(state.clockDraftDay);
  const String monthValue = (state.clockEditActive && state.clockMenuSelection == 1 && state.clockInputText[0] != '\0')
                                ? String(state.clockInputText)
                                : two_digits(state.clockDraftMonth);
  const String yearValue = (state.clockEditActive && state.clockMenuSelection == 2 && state.clockInputText[0] != '\0')
                               ? String(state.clockInputText)
                               : two_digits(state.clockDraftYear);
  const String hourValue = (state.clockEditActive && state.clockMenuSelection == 3 && state.clockInputText[0] != '\0')
                               ? String(state.clockInputText)
                               : two_digits(state.clockDraftHour);
  const String minuteValue = (state.clockEditActive && state.clockMenuSelection == 4 && state.clockInputText[0] != '\0')
                                 ? String(state.clockInputText)
                                 : two_digits(state.clockDraftMinute);
  const String preview = two_digits(state.clockDraftDay) + "/" + two_digits(state.clockDraftMonth) + "/" +
                         two_digits(state.clockDraftYear) + " " + two_digits(state.clockDraftHour) + ":" +
                         two_digits(state.clockDraftMinute);
  const int previewY = startY + (layout.isLargeDisplay ? 8 : 4);
  const int controlsY = previewY + lineHeight + (layout.isLargeDisplay ? 20 : 14);
  const int highlightPadX = layout.isLargeDisplay ? 2 : 1;
  const int highlightPadY = 0;
  const String slash = "/";
  const String spacePair = "  ";
  const String colon = ":";
  const int dayW = uiDisplayTextWidth(dayValue, itemSize, font);
  const int monthW = uiDisplayTextWidth(monthValue, itemSize, font);
  const int yearW = uiDisplayTextWidth(yearValue, itemSize, font);
  const int hourW = uiDisplayTextWidth(hourValue, itemSize, font);
  const int minuteW = uiDisplayTextWidth(minuteValue, itemSize, font);
  const int slashW = uiDisplayTextWidth(slash, itemSize, font);
  const int spaceW = uiDisplayTextWidth(spacePair, itemSize, font);
  const int colonW = uiDisplayTextWidth(colon, itemSize, font);
  const int previewW = dayW + slashW + monthW + slashW + yearW + spaceW + hourW + colonW + minuteW;
  const int previewX = (layout.displayW - previewW) / 2;
  const String saveLabel = "1 Save";
  const String backLabel = "2 Back";
  const int controlsGap = layout.isLargeDisplay ? 24 : 16;
  int saveX = 0;
  int backX = 0;
  compute_two_column_positions(layout,
                               uiDisplayTextWidth(saveLabel, itemSize, font),
                               uiDisplayTextWidth(backLabel, itemSize, font),
                               controlsGap,
                               saveX,
                               backX);

  const auto draw_clock_segment = [&](int x, const String &text, bool selected, bool editing) {
    const uint16_t segmentBg = editing ? kUiSetColor : kUiHighlight;
    const uint16_t segmentFg = kUiDark;
    if (selected) {
      uiDisplayFillRect(x - highlightPadX,
                        previewY - highlightPadY,
                        uiDisplayTextWidth(text, itemSize, font) + (highlightPadX * 2),
                        lineHeight + (highlightPadY * 2),
                        segmentBg);
    }
    uiDisplayPrintStyledAt(x,
                           previewY,
                           text,
                           selected ? segmentFg : kUiText,
                           selected ? segmentBg : kUiModeAreaBg,
                           itemSize,
                           font);
  };

  clear_config_content_zone();
  draw_config_title(layout, "CLOCK", titleSize);
  int x = previewX;
  draw_clock_segment(x, dayValue, state.clockMenuSelection == 0, state.clockEditActive && state.clockMenuSelection == 0);
  x += dayW;
  uiDisplayPrintStyledAt(x, previewY, slash, kUiText, kUiModeAreaBg, itemSize, font);
  x += slashW;
  draw_clock_segment(x, monthValue, state.clockMenuSelection == 1, state.clockEditActive && state.clockMenuSelection == 1);
  x += monthW;
  uiDisplayPrintStyledAt(x, previewY, slash, kUiText, kUiModeAreaBg, itemSize, font);
  x += slashW;
  draw_clock_segment(x, yearValue, state.clockMenuSelection == 2, state.clockEditActive && state.clockMenuSelection == 2);
  x += yearW;
  uiDisplayPrintStyledAt(x, previewY, spacePair, kUiText, kUiModeAreaBg, itemSize, font);
  x += spaceW;
  draw_clock_segment(x, hourValue, state.clockMenuSelection == 3, state.clockEditActive && state.clockMenuSelection == 3);
  x += hourW;
  uiDisplayPrintStyledAt(x, previewY, colon, kUiText, kUiModeAreaBg, itemSize, font);
  x += colonW;
  draw_clock_segment(x, minuteValue, state.clockMenuSelection == 4, state.clockEditActive && state.clockMenuSelection == 4);

  draw_config_column_item(layout, saveX, controlsY, saveLabel, state.clockMenuSelection == 5, itemSize);
  draw_config_column_item(layout, backX, controlsY, backLabel, state.clockMenuSelection == 6, itemSize);
  draw_battery_setup_set_zone("SET> ", state.clockEditActive ? String(state.clockInputText) : String(""));
}

void uiDisplayRenderCalibrationSetupMenu(const char *inputText) {
  const ManagedZoneLayout layout = managed_zone_layout();
  const uint8_t titleSize = layout.isLargeDisplay ? 2 : 1;
  const uint8_t itemSize = layout.isLargeDisplay ? 2 : 1;
  const uint8_t font = 2;
  const int titleY = layout.contentY + (layout.isLargeDisplay ? 10 : 8);
  const int lineHeight = uiDisplayFontHeight(itemSize, font);
  const int gap = layout.isLargeDisplay ? 4 : 6;
  const int startY = titleY + uiDisplayFontHeight(titleSize, font) + (layout.isLargeDisplay ? 8 : 10);
  const int columnGap = layout.isLargeDisplay ? 48 : 32;
  int leftX = 0;
  int rightX = 0;
  compute_two_column_positions(layout,
                               uiDisplayTextWidth("3 Temp", itemSize, font),
                               uiDisplayTextWidth("6 Back", itemSize, font),
                               columnGap,
                               leftX,
                               rightX);

  clear_config_content_zone();
  draw_config_title(layout, "CALIBRATION", titleSize);
  draw_config_column_item(layout, leftX, startY, "1 Voltage", false, itemSize);
  draw_config_column_item(layout, leftX, startY + (lineHeight + gap), "2 Current", false, itemSize);
  draw_config_column_item(layout, leftX, startY + ((lineHeight + gap) * 2), "3 Temp", false, itemSize);
  draw_config_column_item(layout, rightX, startY, "4 Load", false, itemSize);
  draw_config_column_item(layout, rightX, startY + (lineHeight + gap), "5 Save", false, itemSize);
  draw_config_column_item(layout, rightX, startY + ((lineHeight + gap) * 2), "6 Back", false, itemSize);
  draw_battery_setup_set_zone("SEL> ", (inputText != nullptr) ? String(inputText) : "");
}

void draw_calibration_overlay_chrome(const char *title) {
  const ManagedZoneLayout layout = managed_zone_layout();
  draw_shared_chrome(layout, String(title), false, false, false);
}

void uiDisplayRenderTempCalibrationEntryScreen(float rawTempC, const char *inputText) {
  const ManagedZoneLayout layout = managed_zone_layout();
  const ContentTextPanelLayout panel = compute_content_text_panel_layout(layout);
  const String sensedLine = "Raw: " + String(rawTempC, 1) + "C";
  const String referenceLine =
      "Real: " + (((inputText != nullptr) && inputText[0] != '\0') ? String(inputText) + "C" : String("--.-C"));
  const String backLine = "CLR-Back";

  draw_calibration_overlay_chrome("CA");
  clear_blue_content_zone(layout);
  draw_content_panel_title(layout, panel, "CAL TEMP");
  draw_config_line(layout, panel.row1Y, sensedLine, false, panel.textSize);
  draw_config_line(layout, panel.row2Y, referenceLine, true, panel.textSize);
  draw_config_line(layout, panel.row3Y, backLine, false, panel.textSize);
  draw_battery_setup_set_zone("REAL> ", (inputText != nullptr) ? String(inputText) : String(""));
  draw_zone_borders(layout);
}

void uiDisplayUpdateTempCalibrationRawValue(float rawTempC) {
  const ManagedZoneLayout layout = managed_zone_layout();
  const ContentTextPanelLayout panel = compute_content_text_panel_layout(layout);
  const String sensedLine = "Raw: " + String(rawTempC, 1) + "C";
  const int bandY = panel.row1Y - 2;
  const int bandH = panel.textH + 6;

  uiDisplayFillRect(1, bandY, layout.displayW - 2, bandH, kUiModeAreaBg);
  draw_config_line(layout, panel.row1Y, sensedLine, false, panel.textSize);
}

void draw_modal_overlay_chrome(const char *title, bool loadEnabled) {
  const ManagedZoneLayout layout = managed_zone_layout();
  draw_shared_chrome(layout, String(title), loadEnabled, false, false);
}

void uiDisplayRenderCalibrationResultScreen(bool voltageMode,
                                            float sensorFactor,
                                            float sensorOffset,
                                            float outputFactor,
                                            float outputOffset) {
  const ManagedZoneLayout layout = managed_zone_layout();
  const ContentTextPanelLayout panel = compute_content_text_panel_layout(layout);
  const uint8_t textSize = panel.textSize;
  const uint8_t font = 2;
  const int lineHeight = uiDisplayFontHeight(textSize, font);
  const int rowGap = layout.isLargeDisplay ? 10 : 8;
  const int startY = panel.row1Y;
  const int columnGap = layout.isLargeDisplay ? 40 : 24;
  const String leftTop = "OF: " + String(outputFactor, 4);
  const String rightTop = "OO: " + String(outputOffset, 0);
  const String leftBottom = "SF: " + String(sensorFactor, voltageMode ? 6 : 4);
  const String rightBottom = "SO: " + String(sensorOffset, voltageMode ? 6 : 3);
  const int leftWidth = max(uiDisplayTextWidth(leftTop, textSize, font), uiDisplayTextWidth(leftBottom, textSize, font));
  const int rightWidth = max(uiDisplayTextWidth(rightTop, textSize, font), uiDisplayTextWidth(rightBottom, textSize, font));
  int leftX = 0;
  int rightX = 0;
  compute_two_column_positions(layout, leftWidth, rightWidth, columnGap, leftX, rightX);

  draw_calibration_overlay_chrome("CA");
  clear_blue_content_zone(layout);
  draw_content_panel_title(layout, panel, voltageMode ? "CAL RESULT V" : "CAL RESULT I");
  draw_config_column_item(layout, leftX, startY, leftTop, false, textSize);
  draw_config_column_item(layout, rightX, startY, rightTop, false, textSize);
  draw_config_column_item(layout, leftX, startY + lineHeight + rowGap, leftBottom, false, textSize);
  draw_config_column_item(layout, rightX, startY + lineHeight + rowGap, rightBottom, false, textSize);
  draw_battery_setup_set_zone("E-Accept  ", "CLR-Reject");
  draw_zone_borders(layout);
}

void uiDisplayRenderCalibrationAbortScreen(const char *detailText) {
  const ManagedZoneLayout layout = managed_zone_layout();
  const ContentTextPanelLayout panel = compute_content_text_panel_layout(layout);
  const String detail = detailText ? String(detailText) : String("Calibration mismatch");

  draw_calibration_overlay_chrome("CA");
  clear_blue_content_zone(layout);
  draw_content_panel_title(layout, panel, "CALIBRATION ABORT");
  draw_config_line(layout, panel.row2Y, detail, true, panel.textSize);
  draw_battery_setup_set_zone("E-Accept  ", "");
  draw_zone_borders(layout);
}

void uiDisplayRenderCalibrationNoticeScreen(const char *title, const char *detail) {
  const ManagedZoneLayout layout = managed_zone_layout();
  const ContentTextPanelLayout panel = compute_content_text_panel_layout(layout);
  draw_calibration_overlay_chrome("CA");
  clear_blue_content_zone(layout);
  draw_content_panel_title(layout, panel, title ? String(title) : String("CALIBRATION"));
  draw_config_line(layout, panel.row2Y, detail ? String(detail) : String(""), true, panel.textSize);
  draw_zone_borders(layout);
}

void uiDisplayRenderProtectionModal(const char *message, char causeCode) {
  const ManagedZoneLayout layout = managed_zone_layout();
  const ContentTextPanelLayout panel = compute_content_text_panel_layout(layout);
  const uint8_t textSize = panel.textSize;
  const uint8_t font = 2;
  const int lineHeight = uiDisplayFontHeight(textSize, font);
  const int startY = panel.row1Y;
  const String detail = message ? String(message) : String("");
  const String cause = String("Cause: ") + String(causeCode);
  const MetricsZoneState metricsState = {format_measured_current_raw(max(0.0f, app_measurements_current_a())),
                                         format_measured_voltage_raw(max(0.0f, app_measurements_voltage_v())),
                                         format_measured_power_raw(max(0.0f, app_measurements_power_w()))};
  const int alertW = uiDisplayTextWidth(detail, textSize, font) + 16;
  const int alertX = (layout.displayW - alertW) / 2;

  draw_modal_overlay_chrome("PROTECTION", app_load_is_enabled());
  clear_blue_content_zone(layout);
  draw_metrics_zone(layout, metricsState);
  draw_content_panel_title(layout, panel, "LOAD DISABLED");
  uiDisplayFillRect(alertX, startY - 1, alertW, lineHeight + 2, kUiAlertBg);
  uiDisplayPrintStyledAt((layout.displayW - uiDisplayTextWidth(detail, textSize, font)) / 2,
                         startY,
                         detail,
                         kUiAlertText,
                         kUiAlertBg,
                         textSize,
                         font);
  draw_config_line(layout, startY + lineHeight + 10, cause, false, textSize);
  draw_battery_setup_set_zone("E-Accept", "");
  draw_zone_borders(layout);
}

void uiDisplayUpdate(void) {
  static unsigned long lastUpdateTime = 0;

  (void)app_ui_consume_clear_cursor_blink_request();

  const UiViewState &state = ui_state_cache_get();
  if (ui_state_machine_current_screen() != UiScreen::Home) {
    g_ccLayoutDrawn = false;
    return;
  }
  if (!state.modeInitialized) {
    g_ccLayoutDrawn = false;
    return;
  }

  if (millis() - lastUpdateTime < LCD_RFSH_TIME) return;
  lastUpdateTime = millis();
  render_managed_home(state, true);
}
