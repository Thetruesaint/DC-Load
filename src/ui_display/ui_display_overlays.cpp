#include "../ui_display.h"
#include "ui_display_internal.h"

#include "../app/app_load_context.h"
#include "../app/app_measurements_context.h"

using namespace ui_display_internal;

namespace ui_display_internal {
ManagedZoneLayout overlay_managed_zone_layout() {
  return shared_managed_zone_layout();
}

ContentTextPanelLayout overlay_content_text_panel_layout(const ManagedZoneLayout &layout) {
  return shared_content_text_panel_layout(layout);
}

void overlay_clear_blue_content_zone(const ManagedZoneLayout &layout) {
  shared_clear_blue_content_zone(layout);
}

void overlay_draw_content_panel_title(const ManagedZoneLayout &layout, const ContentTextPanelLayout &panel, const String &title) {
  shared_draw_content_panel_title(layout, panel, title);
}

void overlay_draw_line(const ManagedZoneLayout &layout, int y, const String &text, bool selected, uint8_t textSize) {
  config_menu_draw_line(layout, y, text, selected, textSize);
}

void overlay_draw_column_item(const ManagedZoneLayout &layout, int x, int y, const String &text, bool selected, uint8_t textSize) {
  config_menu_draw_column_item(layout, x, y, text, selected, textSize);
}

void overlay_draw_input_zone(const String &prefix, const String &valueText) {
  shared_draw_input_zone(prefix, valueText);
}

void overlay_draw_zone_borders(const ManagedZoneLayout &layout) {
  shared_draw_zone_borders(layout);
}

void overlay_compute_two_column_positions(const ManagedZoneLayout &layout, int leftWidth, int rightWidth, int gap, int &leftX, int &rightX) {
  config_menu_compute_two_column_positions(layout, leftWidth, rightWidth, gap, leftX, rightX);
}

void overlay_draw_metrics_zone(const ManagedZoneLayout &layout, const MetricsZoneState &state) {
  shared_draw_metrics_zone(layout, state);
}

String overlay_format_measured_current_raw(float value) {
  return shared_format_measured_current_raw(value);
}

String overlay_format_measured_voltage_raw(float value) {
  return shared_format_measured_voltage_raw(value);
}

String overlay_format_measured_power_raw(float value) {
  return shared_format_measured_power_raw(value);
}
}  // namespace ui_display_internal

void uiDisplayRenderTempCalibrationEntryScreen(float rawTempC, const char *inputText) {
  const ManagedZoneLayout layout = overlay_managed_zone_layout();
  const ContentTextPanelLayout panel = overlay_content_text_panel_layout(layout);
  const String sensedLine = "Raw: " + String(rawTempC, 1) + "C";
  const String referenceLine =
      "Real: " + (((inputText != nullptr) && inputText[0] != '\0') ? String(inputText) + "C" : String("--.-C"));
  const String backLine = "CLR-Back";

  overlay_draw_calibration_chrome("CA");
  overlay_clear_blue_content_zone(layout);
  overlay_draw_content_panel_title(layout, panel, "CAL TEMP");
  overlay_draw_line(layout, panel.row1Y, sensedLine, false, panel.textSize);
  overlay_draw_line(layout, panel.row2Y, referenceLine, true, panel.textSize);
  overlay_draw_line(layout, panel.row3Y, backLine, false, panel.textSize);
  overlay_draw_input_zone("REAL> ", (inputText != nullptr) ? String(inputText) : String(""));
  overlay_draw_zone_borders(layout);
}

void uiDisplayUpdateTempCalibrationRawValue(float rawTempC) {
  const ManagedZoneLayout layout = overlay_managed_zone_layout();
  const ContentTextPanelLayout panel = overlay_content_text_panel_layout(layout);
  const String sensedLine = "Raw: " + String(rawTempC, 1) + "C";
  const int bandY = panel.row1Y - 2;
  const int bandH = panel.textH + 6;

  uiDisplayFillRect(1, bandY, layout.displayW - 2, bandH, kUiModeAreaBg);
  overlay_draw_line(layout, panel.row1Y, sensedLine, false, panel.textSize);
}

void uiDisplayRenderCalibrationResultScreen(bool voltageMode,
                                            float sensorFactor,
                                            float sensorOffset,
                                            float outputFactor,
                                            float outputOffset) {
  const ManagedZoneLayout layout = overlay_managed_zone_layout();
  const ContentTextPanelLayout panel = overlay_content_text_panel_layout(layout);
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
  overlay_compute_two_column_positions(layout, leftWidth, rightWidth, columnGap, leftX, rightX);

  overlay_draw_calibration_chrome("CA");
  overlay_clear_blue_content_zone(layout);
  overlay_draw_content_panel_title(layout, panel, voltageMode ? "CAL RESULT V" : "CAL RESULT I");
  overlay_draw_column_item(layout, leftX, startY, leftTop, false, textSize);
  overlay_draw_column_item(layout, rightX, startY, rightTop, false, textSize);
  overlay_draw_column_item(layout, leftX, startY + lineHeight + rowGap, leftBottom, false, textSize);
  overlay_draw_column_item(layout, rightX, startY + lineHeight + rowGap, rightBottom, false, textSize);
  overlay_draw_input_zone("E-Accept  ", "CLR-Reject");
  overlay_draw_zone_borders(layout);
}

void uiDisplayRenderCalibrationAbortScreen(const char *detailText) {
  const ManagedZoneLayout layout = overlay_managed_zone_layout();
  const ContentTextPanelLayout panel = overlay_content_text_panel_layout(layout);
  const String detail = detailText ? String(detailText) : String("Calibration mismatch");

  overlay_draw_calibration_chrome("CA");
  overlay_clear_blue_content_zone(layout);
  overlay_draw_content_panel_title(layout, panel, "CALIBRATION ABORT");
  overlay_draw_line(layout, panel.row2Y, detail, true, panel.textSize);
  overlay_draw_input_zone("E-Accept  ", "");
  overlay_draw_zone_borders(layout);
}

void uiDisplayRenderCalibrationNoticeScreen(const char *title, const char *detail) {
  const ManagedZoneLayout layout = overlay_managed_zone_layout();
  const ContentTextPanelLayout panel = overlay_content_text_panel_layout(layout);

  overlay_draw_calibration_chrome("CA");
  overlay_clear_blue_content_zone(layout);
  overlay_draw_content_panel_title(layout, panel, title ? String(title) : String("CALIBRATION"));
  overlay_draw_line(layout, panel.row2Y, detail ? String(detail) : String(""), true, panel.textSize);
  overlay_draw_zone_borders(layout);
}

void uiDisplayRenderProtectionModal(const char *message, char causeCode) {
  const ManagedZoneLayout layout = overlay_managed_zone_layout();
  const ContentTextPanelLayout panel = overlay_content_text_panel_layout(layout);
  const uint8_t textSize = panel.textSize;
  const uint8_t font = 2;
  const int lineHeight = uiDisplayFontHeight(textSize, font);
  const int startY = panel.row1Y;
  const String detail = message ? String(message) : String("");
  const String cause = String("Cause: ") + String(causeCode);
  const MetricsZoneState metricsState = {overlay_format_measured_current_raw(max(0.0f, app_measurements_current_a())),
                                         overlay_format_measured_voltage_raw(max(0.0f, app_measurements_voltage_v())),
                                         overlay_format_measured_power_raw(max(0.0f, app_measurements_power_w()))};
  const int alertW = uiDisplayTextWidth(detail, textSize, font) + 16;
  const int alertX = (layout.displayW - alertW) / 2;

  overlay_draw_modal_chrome("PROTECTION", app_load_is_enabled());
  overlay_clear_blue_content_zone(layout);
  overlay_draw_metrics_zone(layout, metricsState);
  overlay_draw_content_panel_title(layout, panel, "LOAD DISABLED");
  uiDisplayFillRect(alertX, startY - 1, alertW, lineHeight + 2, kUiAlertBg);
  uiDisplayPrintStyledAt((layout.displayW - uiDisplayTextWidth(detail, textSize, font)) / 2,
                         startY,
                         detail,
                         kUiAlertText,
                         kUiAlertBg,
                         textSize,
                         font);
  overlay_draw_line(layout, startY + lineHeight + 10, cause, false, textSize);
  overlay_draw_input_zone("E-Accept", "");
  overlay_draw_zone_borders(layout);
}
