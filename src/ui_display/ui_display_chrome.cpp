#include "../ui_display.h"
#include "ui_display_internal.h"

#include "../app/app_fan_context.h"
#include "../app/app_limits_context.h"
#include "../app/app_measurements_context.h"
#include "../app/app_mode_state_context.h"
#include "../app/app_msc.h"
#include "../config/system_constants.h"
#include "../hw/hw_objects.h"
#include "../ui/ui_view_state.h"

namespace {
using ui_display_internal::ContentTextPanelLayout;
using ui_display_internal::InputZoneRenderLayout;
using ui_display_internal::ManagedZoneLayout;
using ui_display_internal::MetricsZoneState;
using ui_display_internal::kFooterTextFont;
using ui_display_internal::kFooterTextSize;
using ui_display_internal::kFirmwareVersion;
using ui_display_internal::kUiAccent;
using ui_display_internal::kUiAlertBg;
using ui_display_internal::kUiBg;
using ui_display_internal::kUiBorder;
using ui_display_internal::kUiDark;
using ui_display_internal::kUiHighlight;
using ui_display_internal::kUiLoadOff;
using ui_display_internal::kUiLoadOn;
using ui_display_internal::kUiModeAreaBg;
using ui_display_internal::kUiSetColor;
using ui_display_internal::kUiText;

bool g_setupMetricsValid = false;
int g_setupMetricsDisplayW = 0;
int g_setupMetricsDisplayH = 0;
uint8_t g_setupMetricsMode = 0xFF;
String g_setupLastMetric1;
String g_setupLastMetric2;
String g_setupLastMetric3;

struct TopStatusZoneState {
  String label;
  bool loadEnabled;
  float tempC;
  float fanTempOnC;
  float tempCutOffC;
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

struct FooterZoneState {
  String leftText;
  String rightText;
};

int footer_bar_height_px(int displayH) {
  return max(16, (displayH * 8) / 100);
}

void draw_horizontal_separator(int y, int width, uint16_t color) {
  tft.drawLine(0, y, width - 1, y, color);
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
  return tft.color565(255, static_cast<uint8_t>(255.0f * (1.0f - progress)), 0);
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

void draw_degree_c_symbol(int x, int y, uint16_t color, uint16_t bg, uint8_t textSize, uint8_t textFont) {
  const int radius = (textSize >= 2) ? 3 : 2;
  uiDisplayDrawCircle(x, y + radius + 1, radius, color);
  uiDisplayPrintStyledAt(x + 5, y - 1, "C", color, bg, textSize, textFont);
}

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
  return {label, loadEnabled, static_cast<float>(app_measurements_temp_c()), static_cast<float>(app_fan_temp_on_c()), app_limits_temp_cutoff()};
}

FooterZoneState current_footer_zone_state() {
  return {String(kFirmwareVersion), rtc_timestamp_text()};
}

void draw_footer_zone(const ManagedZoneLayout &layout, const FooterZoneState &state) {
  const int footerTextY = layout.footerY + ((layout.bottomBarH - uiDisplayFontHeight(kFooterTextSize, kFooterTextFont)) / 2);

  uiDisplayFillRect(0, layout.footerY, layout.displayW, layout.bottomBarH, kUiAccent);
  uiDisplayPrintStyledAt(4, footerTextY, state.leftText, kUiText, kUiAccent, kFooterTextSize, kFooterTextFont);
  uiDisplayPrintStyledAt(layout.displayW - uiDisplayTextWidth(state.rightText, kFooterTextSize, kFooterTextFont) - 4,
                         footerTextY,
                         state.rightText,
                         kUiText,
                         kUiAccent,
                         kFooterTextSize,
                         kFooterTextFont);
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

  return {metricTextFont, metricTextSize, metric1W, metric2W, metric3W, metricGap, metricsTextY, metricPadding, metric1X, metric2X, metric3X, metricAreaY, metricAreaH};
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
  draw_shared_chrome(layout, mode_label_for_value(app_mode_state_mode()), false, clearContentZone, true);
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
  draw_top_status_zone(layout, current_top_status_zone_state(String(mode_label_for_value(app_mode_state_mode())), false));
  draw_horizontal_separator(layout.topBarH, layout.displayW, kUiBorder);
}

void clear_config_content_zone() {
  const ManagedZoneLayout layout = managed_zone_layout();
  uiDisplayFillRect(1, layout.contentY + 1, layout.displayW - 2, layout.contentH - 1, kUiModeAreaBg);
}

void draw_calibration_overlay_chrome(const char *title) {
  const ManagedZoneLayout layout = managed_zone_layout();
  draw_shared_chrome(layout, String(title), false, false, false);
}

void draw_modal_overlay_chrome(const char *title, bool loadEnabled) {
  const ManagedZoneLayout layout = managed_zone_layout();
  draw_shared_chrome(layout, String(title), loadEnabled, false, false);
}
}  // namespace

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

namespace ui_display_internal {
ManagedZoneLayout shared_managed_zone_layout() {
  return ::managed_zone_layout();
}

ContentTextPanelLayout shared_content_text_panel_layout(const ManagedZoneLayout &layout) {
  return ::compute_content_text_panel_layout(layout);
}

int shared_centered_content_text_x(const ManagedZoneLayout &layout, const String &text, uint8_t textSize, uint8_t textFont) {
  return ::centered_content_text_x(layout, text, textSize, textFont);
}

void config_menu_draw_battery_setup_set_zone(const String &prefix, const String &valueText) {
  ::draw_battery_setup_set_zone(prefix, valueText);
}

void config_menu_clear_content_zone() {
  ::clear_config_content_zone();
}

void shared_clear_blue_content_zone(const ManagedZoneLayout &layout) {
  ::clear_blue_content_zone(layout);
}

void shared_draw_content_panel_title(const ManagedZoneLayout &layout, const ContentTextPanelLayout &panel, const String &title) {
  ::draw_content_panel_title(layout, panel, title);
}

void shared_draw_content_stage_line(const ContentTextPanelLayout &panel, const String &text, int x, int y, bool selected) {
  ::draw_content_stage_line(panel, text, x, y, selected);
}

void shared_draw_zone_borders(const ManagedZoneLayout &layout) {
  ::draw_zone_borders(layout);
}

void shared_draw_metrics_zone(const ManagedZoneLayout &layout, const MetricsZoneState &state) {
  ::draw_metrics_zone(layout, state);
}

void shared_draw_input_zone(const String &prefix, const String &valueText) {
  ::draw_battery_setup_set_zone(prefix, valueText);
}

String shared_format_measured_current_raw(float value) {
  return ::format_measured_current_raw(value);
}

String shared_format_measured_voltage_raw(float value) {
  return ::format_measured_voltage_raw(value);
}

String shared_format_measured_power_raw(float value) {
  return ::format_measured_power_raw(value);
}

void shared_setup_draw_screen_base(const UiViewState &state, const char *modeLabel) {
  ::draw_setup_screen_base(state, modeLabel);
}

void shared_draw_top_status_zone(const ManagedZoneLayout &layout,
                                 const String &label,
                                 bool loadEnabled,
                                 float tempC,
                                 float fanTempOnC,
                                 float tempCutOffC) {
  ::draw_top_status_zone(layout, {label, loadEnabled, tempC, fanTempOnC, tempCutOffC});
}

void shared_draw_footer_zone(const ManagedZoneLayout &layout) {
  ::draw_footer_zone(layout, current_footer_zone_state());
}

String shared_rtc_timestamp_text() {
  return ::rtc_timestamp_text();
}

void shared_update_metrics_zone_values(const ManagedZoneLayout &layout,
                                       const MetricsZoneState &state,
                                       bool layoutChanged,
                                       String &lastMetric1,
                                       String &lastMetric2,
                                       String &lastMetric3) {
  ::update_metrics_zone_values(layout, state, layoutChanged, lastMetric1, lastMetric2, lastMetric3);
}

InputZoneRenderLayout shared_input_zone_render_layout(const ManagedZoneLayout &layout) {
  return ::compute_input_zone_render_layout(layout);
}

void shared_clear_input_zone(const ManagedZoneLayout &layout) {
  ::clear_input_zone(layout);
}

void shared_draw_input_zone_shift_hint(const ManagedZoneLayout &layout) {
  ::draw_input_zone_shift_hint(layout);
}

int shared_draw_input_zone_highlighted_value(const ManagedZoneLayout &layout,
                                             int startX,
                                             const String &valueText,
                                             int highlightedIndex,
                                             bool cursorVisible) {
  return ::draw_input_zone_highlighted_value(layout, startX, valueText, highlightedIndex, cursorVisible);
}

const char *shared_mode_label_for_value(uint8_t mode) {
  return ::mode_label_for_value(mode);
}

void overlay_draw_calibration_chrome(const char *title) {
  ::draw_calibration_overlay_chrome(title);
}

void overlay_draw_modal_chrome(const char *title, bool loadEnabled) {
  ::draw_modal_overlay_chrome(title, loadEnabled);
}

}  // namespace ui_display_internal
