#include "../ui_display.h"
#include "ui_display_internal.h"

#include "../app/app_calibration_context.h"
#include "../app/app_input_buffer.h"
#include "../app/app_mode_state_context.h"
#include "../app/app_msc.h"
#include "../app/app_timing_alerts.h"
#include "../app/app_trace_context.h"
#include "../app/app_ui_context.h"
#include "../config/system_constants.h"
#include "../hw/hw_objects.h"
#include "../ui/ui_state_cache.h"
#include "../ui/ui_state_machine.h"
#include "../ui/ui_view_state.h"

namespace ui_display_internal {
ManagedZoneLayout home_managed_zone_layout() {
  return shared_managed_zone_layout();
}

void home_draw_zone_borders(const ManagedZoneLayout &layout) {
  shared_draw_zone_borders(layout);
}

void home_draw_top_status_zone(const ManagedZoneLayout &layout,
                               const String &label,
                               bool loadEnabled,
                               float tempC,
                               float fanTempOnC,
                               float tempCutOffC) {
  shared_draw_top_status_zone(layout, label, loadEnabled, tempC, fanTempOnC, tempCutOffC);
}

void home_draw_footer_zone(const ManagedZoneLayout &layout) {
  shared_draw_footer_zone(layout);
}

String home_rtc_timestamp_text() {
  return shared_rtc_timestamp_text();
}

void home_update_metrics_zone_values(const ManagedZoneLayout &layout,
                                     const MetricsZoneState &state,
                                     bool layoutChanged,
                                     String &lastMetric1,
                                     String &lastMetric2,
                                     String &lastMetric3) {
  shared_update_metrics_zone_values(layout, state, layoutChanged, lastMetric1, lastMetric2, lastMetric3);
}

InputZoneRenderLayout home_input_zone_render_layout(const ManagedZoneLayout &layout) {
  return shared_input_zone_render_layout(layout);
}

void home_clear_input_zone(const ManagedZoneLayout &layout) {
  shared_clear_input_zone(layout);
}

void home_draw_input_zone_shift_hint(const ManagedZoneLayout &layout) {
  shared_draw_input_zone_shift_hint(layout);
}

int home_draw_input_zone_highlighted_value(const ManagedZoneLayout &layout,
                                           int startX,
                                           const String &valueText,
                                           int highlightedIndex,
                                           bool cursorVisible) {
  return shared_draw_input_zone_highlighted_value(layout, startX, valueText, highlightedIndex, cursorVisible);
}

ContentTextPanelLayout home_content_text_panel_layout(const ManagedZoneLayout &layout) {
  return shared_content_text_panel_layout(layout);
}

int home_centered_content_text_x(const ManagedZoneLayout &layout, const String &text, uint8_t textSize, uint8_t textFont) {
  return shared_centered_content_text_x(layout, text, textSize, textFont);
}

void home_clear_blue_content_zone(const ManagedZoneLayout &layout) {
  shared_clear_blue_content_zone(layout);
}

void home_draw_content_panel_title(const ManagedZoneLayout &layout, const ContentTextPanelLayout &panel, const String &title) {
  shared_draw_content_panel_title(layout, panel, title);
}

void home_draw_content_stage_line(const ContentTextPanelLayout &panel, const String &text, int x, int y, bool selected) {
  shared_draw_content_stage_line(panel, text, x, y, selected);
}

const char *home_mode_label_for_value(uint8_t mode) {
  return shared_mode_label_for_value(mode);
}
}  // namespace ui_display_internal

namespace {
using ui_display_internal::ContentTextPanelLayout;
using ui_display_internal::InputZoneRenderLayout;
using ui_display_internal::ManagedZoneLayout;
using ui_display_internal::MetricsZoneState;
using ui_display_internal::kUiAccent;
using ui_display_internal::kUiBg;
using ui_display_internal::kUiBorder;
using ui_display_internal::kUiDark;
using ui_display_internal::kUiHighlight;
using ui_display_internal::kUiModeAreaBg;
using ui_display_internal::kUiSetColor;
using ui_display_internal::kUiText;

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
  constexpr float kTraceTimeScales[] = {5.0f, 10.0f, 20.0f, 40.0f, 60.0f, 120.0f, 300.0f, 600.0f, 1200.0f, 2400.0f, 3600.0f};

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
  if (timeScaleSeconds < 60.0f) return String(static_cast<int>(timeScaleSeconds)) + "s";
  const int minutes = static_cast<int>(timeScaleSeconds / 60.0f);
  if (minutes < 60) return String(minutes) + "m";
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
  if (maxVoltageV > 1.0f && maxVoltageV <= 5.0f) return 5.0f;
  if (maxVoltageV > 5.0f && maxVoltageV <= 10.0f) return 10.0f;
  if (maxVoltageV > 10.0f && maxVoltageV <= 15.0f) return 15.0f;
  if (maxVoltageV > 15.0f && maxVoltageV <= 20.0f) return 20.0f;
  if (maxVoltageV > 20.0f && maxVoltageV <= 25.0f) return 25.0f;
  if (maxVoltageV > 25.0f && maxVoltageV <= 30.0f) return 30.0f;
  if (maxVoltageV > 30.0f) return 35.0f;
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

  const String currentLegend = "Imax:" + String(currentScaleMax, 0);
  const String voltageLegend = "Vmax:" + String(voltageScaleMax, 0);
  const String timeLegend = trace_time_legend(timeScaleSeconds);
  const float currentTickStep = (currentScaleMax <= 1.0f) ? 0.1f : (currentScaleMax <= 2.0f ? 0.5f : 1.0f);
  const float voltageTickStep = (voltageScaleMax <= 1.0f) ? 0.5f : ((voltageScaleMax <= 10.0f) ? 1.0f : (voltageScaleMax <= 20.0f ? 2.0f : 5.0f));

  const int legendY = y + 8;
  const int legendStartX = panelX + (panelW / 2) - uiDisplayTextWidth(currentLegend + "   " + voltageLegend, textSize, textFont) / 2;
  uiDisplayPrintStyledAt(legendStartX, legendY, currentLegend, currentColor, textBg, textSize, textFont);
  uiDisplayPrintStyledAt(legendStartX + uiDisplayTextWidth(currentLegend + "   ", textSize, textFont), legendY, voltageLegend, voltageColor, textBg, textSize, textFont);
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
    lastTickX = tickX;
    tft.drawLine(tickX, plotY + plotH, tickX, plotY + plotH + (isLargeDisplay ? 4 : 3), axisColor);
  }

  if (sampleCount > 0) {
    draw_cc_trace_segment_range(plotX, plotY, plotW, plotH, sampleIntervalMs, timeScaleSeconds, currentScaleMax, voltageScaleMax, currentColor, voltageColor, 0, sampleCount - 1);
  }

  g_ccLastTraceSampleCount = sampleCount;
  g_ccLastTraceTimeScaleSeconds = timeScaleSeconds;
  g_ccLastTraceCurrentScaleMax = currentScaleMax;
  g_ccLastTraceVoltageScaleMax = voltageScaleMax;
}

String format_cp_value(float readingValue) {
  return String(readingValue, 1);
}

String format_cr_value(float readingValue) {
  return String(readingValue, 1);
}

String format_tc_period_value(float periodMs) {
  return String(static_cast<unsigned long>(max(0.0f, periodMs)));
}

String format_transient_current(float currentA) {
  return String(currentA, 3) + "A";
}

String format_transient_period_label(float periodMs) {
  return String(static_cast<unsigned long>(max(0.0f, periodMs))) + " mS";
}

int cc_cursor_text_index(const String &valueText, int cursorPosition) {
  const int decimal = valueText.indexOf('.');
  switch (cursorPosition) {
    case 8: return 0;
    case 9: return 1;
    case 10: return 2;
    case 11: return (decimal >= 0 && decimal + 1 < valueText.length()) ? (decimal + 1) : -1;
    case 12: return (decimal >= 0 && decimal + 2 < valueText.length()) ? (decimal + 2) : -1;
    case 13: return (decimal >= 0 && decimal + 3 < valueText.length()) ? (decimal + 3) : -1;
    default: return -1;
  }
}

int cp_cursor_text_index(const String &valueText, int cursorPosition) {
  const int decimal = valueText.indexOf('.');
  switch (cursorPosition) {
    case 8: return 0;
    case 9: return 1;
    case 10: return (decimal >= 0 && decimal + 1 < valueText.length()) ? (decimal + 1) : -1;
    default: return -1;
  }
}

int cr_cursor_text_index(const String &valueText, int cursorPosition) {
  const int decimal = valueText.indexOf('.');
  switch (cursorPosition) {
    case 8: return 0;
    case 9: return 1;
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
  const ContentTextPanelLayout panel = ui_display_internal::home_content_text_panel_layout(layout);
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
    ui_display_internal::home_clear_blue_content_zone(layout);
    ui_display_internal::home_draw_content_panel_title(layout, panel, title);
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
    uiDisplayPrintStyledAt(statusX + statusPrefixW + statusGap, panel.row1Y, statusValue, statusColor, kUiModeAreaBg, panel.textSize, panel.textFont);
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
  const ContentTextPanelLayout panel = ui_display_internal::home_content_text_panel_layout(layout);
  const int row1X = ui_display_internal::home_centered_content_text_x(layout, row1, panel.textSize, panel.textFont);
  const int pairGap = layout.isLargeDisplay ? 28 : 18;
  const int row2LeftW = uiDisplayTextWidth(row2Left, panel.textSize, panel.textFont);
  const int row2RightW = uiDisplayTextWidth(row2Right, panel.textSize, panel.textFont);
  const int row2TotalW = row2LeftW + pairGap + row2RightW;
  const int row2StartX = (layout.displayW - row2TotalW) / 2;
  const int row2RightX = row2StartX + row2LeftW + pairGap;
  const int row3X = ui_display_internal::home_centered_content_text_x(layout, row3, panel.textSize, panel.textFont);

  if (refreshAll) {
    ui_display_internal::home_clear_blue_content_zone(layout);
    ui_display_internal::home_draw_content_panel_title(layout, panel, title);
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
  const ContentTextPanelLayout panel = ui_display_internal::home_content_text_panel_layout(layout);
  const int row1X = ui_display_internal::home_centered_content_text_x(layout, row1, panel.textSize, panel.textFont);
  const int row2X = ui_display_internal::home_centered_content_text_x(layout, row2, panel.textSize, panel.textFont);
  const int row3X = ui_display_internal::home_centered_content_text_x(layout, row3, panel.textSize, panel.textFont);

  if (refreshAll) {
    ui_display_internal::home_clear_blue_content_zone(layout);
    ui_display_internal::home_draw_content_panel_title(layout, panel, title);
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
    ui_display_internal::home_draw_zone_borders(layout);
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

bool render_managed_home(const UiViewState &state, bool cursorVisible) {
  const ManagedZoneLayout layout = ui_display_internal::home_managed_zone_layout();
  const int displayW = layout.displayW;
  const int displayH = layout.displayH;
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
  const String footerDateTime = ui_display_internal::home_rtc_timestamp_text();
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
    uiDisplayFillRect(0, 0, displayW, layout.topBarH, kUiAccent);
    uiDisplayFillRect(1, layout.contentY + 1, displayW - 2, layout.contentH - 1, kUiModeAreaBg);
    ui_display_internal::home_draw_footer_zone(layout);
    ui_display_internal::home_draw_zone_borders(layout);
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
    uiDisplayFillRect(1, layout.contentY + 1, displayW - 2, layout.contentH - 1, kUiModeAreaBg);
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
  if (layoutChanged || g_ccLastLoadEnabled != state.loadEnabled || g_ccLastTempText != tempText) {
    ui_display_internal::home_draw_top_status_zone(layout, String(modeLabel), state.loadEnabled, state.tempC, state.fanTempOnC, state.tempCutOffC);
    g_ccLastLoadEnabled = state.loadEnabled;
    g_ccLastTempText = tempText;
    ui_display_internal::home_draw_zone_borders(layout);
  }

  ui_display_internal::home_update_metrics_zone_values(layout, metricsState, layoutChanged, g_ccLastMetric1, g_ccLastMetric2, g_ccLastMetric3);

  if (!traceOverlayActive &&
      isBcMode &&
      (layoutChanged || traceOverlayChanged || g_ccLastModeLine1 != modeLine1 || g_ccLastModeLine2 != modeLine2 ||
       g_ccLastModeLine3 != modeLine3 || g_ccLastModeLine4 != modeLine4 || g_ccLastModeLine5 != modeLine5 ||
       g_ccLastBatteryStatus != bcStatus || g_ccLastBatteryStatusColor != bcStatusColor)) {
    render_bc_mode_content(layout, layoutChanged || traceOverlayChanged, modeTitle, modeLine1, bcStatus, bcStatusColor, modeLine3, modeLine2, modeLine4, modeLine5);
  }

  if (!traceOverlayActive &&
      isTransientMode &&
      (layoutChanged || traceOverlayChanged || g_ccLastModeLine1 != modeLine1 || g_ccLastModeLine2 != modeLine2 ||
       g_ccLastModeLine3 != modeLine3 || g_ccLastModeLine4 != modeLine4)) {
    render_transient_mode_content(layout, layoutChanged || traceOverlayChanged, modeTitle, modeLine1, modeLine3, modeLine2, modeLine4);
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
      (supportsCursorHighlight && !cursorRefreshBlocked && (g_ccLastCursorVisible != cursorVisible || g_ccLastCursorPosition != state.cursorPosition))) {
    ui_display_internal::home_clear_input_zone(layout);
    const InputZoneRenderLayout inputLayout = ui_display_internal::home_input_zone_render_layout(layout);
    if (isCaMode) {
      const int realBlockWidth = uiDisplayTextWidth(calibrationRealPrefix + String("0.000") + calibrationRealUnit, inputLayout.textSize, inputLayout.textFont);
      const int realX = displayW - realBlockWidth - max(24, displayW / 25);
      const int prefixWidth = uiDisplayTextWidth(setPrefix, inputLayout.textSize, inputLayout.textFont);
      uiDisplayPrintStyledAt(inputLayout.textX, inputLayout.textY, setPrefix, kUiSetColor, kUiBg, inputLayout.textSize, inputLayout.textFont);
      const int highlightedIndex = cc_cursor_text_index(setValueText, state.cursorPosition);
      int currentX = ui_display_internal::home_draw_input_zone_highlighted_value(layout, inputLayout.textX + prefixWidth, setValueText, highlightedIndex, cursorVisible);
      uiDisplayPrintStyledAt(currentX, inputLayout.textY, setUnit, kUiSetColor, kUiBg, inputLayout.textSize, inputLayout.textFont);
      uiDisplayPrintStyledAt(realX, inputLayout.textY, calibrationRealPrefix, kUiSetColor, kUiBg, inputLayout.textSize, inputLayout.textFont);
      uiDisplayPrintStyledAt(realX + uiDisplayTextWidth(calibrationRealPrefix, inputLayout.textSize, inputLayout.textFont), inputLayout.textY, calibrationRealValue + calibrationRealUnit, kUiSetColor, kUiBg, inputLayout.textSize, inputLayout.textFont);
    } else {
      const int prefixWidth = uiDisplayTextWidth(setPrefix, inputLayout.textSize, inputLayout.textFont);
      uiDisplayPrintStyledAt(inputLayout.textX, inputLayout.textY, setPrefix, kUiSetColor, kUiBg, inputLayout.textSize, inputLayout.textFont);

      if (hasLiveInput) {
        uiDisplayPrintStyledAt(inputLayout.textX + prefixWidth, inputLayout.textY, setText.substring(5), kUiSetColor, kUiBg, inputLayout.textSize, inputLayout.textFont);
      } else if (supportsCursorHighlight) {
        const int highlightedIndex = isCcMode ? cc_cursor_text_index(setValueText, state.cursorPosition)
                                              : isCpMode ? cp_cursor_text_index(setValueText, state.cursorPosition)
                                                         : isCrMode ? cr_cursor_text_index(setValueText, state.cursorPosition)
                                                                    : isTcMode ? tc_cursor_text_index(setValueText, state.cursorPosition)
                                                                               : bc_cursor_text_index(setValueText, state.cursorPosition);
        int currentX = ui_display_internal::home_draw_input_zone_highlighted_value(layout, inputLayout.textX + prefixWidth, setValueText, highlightedIndex, cursorVisible);
        uiDisplayPrintStyledAt(currentX, inputLayout.textY, setUnit, kUiSetColor, kUiBg, inputLayout.textSize, inputLayout.textFont);
      } else {
        uiDisplayPrintStyledAt(inputLayout.textX + prefixWidth, inputLayout.textY, setValueText + setUnit, kUiSetColor, kUiBg, inputLayout.textSize, inputLayout.textFont);
      }
    }
    if (shiftActive) {
      ui_display_internal::home_draw_input_zone_shift_hint(layout);
    }
    g_ccLastSetText = isCaMode ? (setText + "|" + calibrationRealValue) : setText;
    g_ccLastCursorVisible = cursorVisible;
    g_ccLastCursorPosition = state.cursorPosition;
    g_ccLastShiftActive = shiftActive;
  }

  if (layoutChanged || g_ccLastFooterText != footerDateTime) {
    ui_display_internal::home_draw_footer_zone(layout);
    g_ccLastFooterText = footerDateTime;
    ui_display_internal::home_draw_zone_borders(layout);
  }
  ui_display_internal::home_draw_zone_borders(layout);
  return true;
}
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
  g_ccLastBatteryStatus = "";
  g_ccLastTraceToken = 0;
  g_ccLastTraceSampleCount = 0;
  g_ccLastTraceTimeScaleSeconds = 0.0f;
  g_ccLastTraceCurrentScaleMax = 0.0f;
  g_ccLastTraceVoltageScaleMax = 0.0f;
  g_ccLastTraceOverlayActive = false;
  g_ccLastBatteryStatusColor = 0;
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
