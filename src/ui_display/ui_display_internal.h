#ifndef UI_DISPLAY_INTERNAL_H
#define UI_DISPLAY_INTERNAL_H

#include <Arduino.h>
#include <TFT_eSPI.h>
#include "../config/system_constants.h"

struct UiViewState;

namespace ui_display_internal {
struct ManagedZoneLayout {
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

struct MetricsZoneState {
  String metric1;
  String metric2;
  String metric3;
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

struct InputZoneRenderLayout {
  uint8_t textFont;
  uint8_t textSize;
  int textX;
  int textY;
  int shiftX;
};

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

constexpr uint8_t kFooterTextFont = 1;
#ifdef WOKWI_SIMULATION
constexpr uint8_t kFooterTextSize = 1;
#else
constexpr uint8_t kFooterTextSize = 2;
#endif

String two_digits(int value);
ManagedZoneLayout shared_managed_zone_layout();
ContentTextPanelLayout shared_content_text_panel_layout(const ManagedZoneLayout &layout);
int shared_centered_content_text_x(const ManagedZoneLayout &layout, const String &text, uint8_t textSize, uint8_t textFont);
void shared_clear_blue_content_zone(const ManagedZoneLayout &layout);
void shared_draw_content_panel_title(const ManagedZoneLayout &layout, const ContentTextPanelLayout &panel, const String &title);
void shared_draw_content_stage_line(const ContentTextPanelLayout &panel, const String &text, int x, int y, bool selected);
void shared_draw_zone_borders(const ManagedZoneLayout &layout);
void shared_draw_metrics_zone(const ManagedZoneLayout &layout, const MetricsZoneState &state);
void shared_draw_input_zone(const String &prefix, const String &valueText);
String shared_format_measured_current_raw(float value);
String shared_format_measured_voltage_raw(float value);
String shared_format_measured_power_raw(float value);
void shared_setup_draw_screen_base(const UiViewState &state, const char *modeLabel);
void shared_draw_top_status_zone(const ManagedZoneLayout &layout,
                                 const String &label,
                                 bool loadEnabled,
                                 float tempC,
                                 float fanTempOnC,
                                 float tempCutOffC);
void shared_draw_footer_zone(const ManagedZoneLayout &layout);
String shared_rtc_timestamp_text();
void shared_update_metrics_zone_values(const ManagedZoneLayout &layout,
                                       const MetricsZoneState &state,
                                       bool layoutChanged,
                                       String &lastMetric1,
                                       String &lastMetric2,
                                       String &lastMetric3);
InputZoneRenderLayout shared_input_zone_render_layout(const ManagedZoneLayout &layout);
void shared_clear_input_zone(const ManagedZoneLayout &layout);
void shared_draw_input_zone_shift_hint(const ManagedZoneLayout &layout);
int shared_draw_input_zone_highlighted_value(const ManagedZoneLayout &layout,
                                             int startX,
                                             const String &valueText,
                                             int highlightedIndex,
                                             bool cursorVisible);
const char *shared_mode_label_for_value(uint8_t mode);
String config_menu_two_digits(int value);
ManagedZoneLayout config_menu_managed_zone_layout();
void config_menu_draw_battery_setup_set_zone(const String &prefix, const String &valueText);
void config_menu_clear_content_zone();
void config_menu_draw_title(const ManagedZoneLayout &layout, const String &title, uint8_t textSize = 0);
void config_menu_draw_line(const ManagedZoneLayout &layout, int y, const String &text, bool selected, uint8_t textSize = 0);
void config_menu_draw_value_edit_line(const ManagedZoneLayout &layout,
                                      int y,
                                      const String &label,
                                      const String &value,
                                      bool selected,
                                      bool editingValue,
                                      uint8_t textSize = 0);
void config_menu_draw_value_edit_item(int x,
                                      int y,
                                      const String &label,
                                      const String &value,
                                      bool selected,
                                      bool editingValue,
                                      uint8_t textSize = 1);
void config_menu_draw_status_item(int x,
                                  int y,
                                  const String &label,
                                  const String &value,
                                  bool selected,
                                  bool active,
                                  uint8_t textSize = 1);
void config_menu_draw_column_item(const ManagedZoneLayout &layout, int x, int y, const String &text, bool selected, uint8_t textSize = 1);
void config_menu_compute_two_column_positions(const ManagedZoneLayout &layout, int leftWidth, int rightWidth, int gap, int &leftX, int &rightX);
String config_menu_format_limit_current(float value);
String config_menu_format_limit_power(float value);
String config_menu_format_limit_temp(float value);

ManagedZoneLayout overlay_managed_zone_layout();
ContentTextPanelLayout overlay_content_text_panel_layout(const ManagedZoneLayout &layout);
void overlay_draw_calibration_chrome(const char *title);
void overlay_draw_modal_chrome(const char *title, bool loadEnabled);
void overlay_clear_blue_content_zone(const ManagedZoneLayout &layout);
void overlay_draw_content_panel_title(const ManagedZoneLayout &layout, const ContentTextPanelLayout &panel, const String &title);
void overlay_draw_line(const ManagedZoneLayout &layout, int y, const String &text, bool selected, uint8_t textSize);
void overlay_draw_column_item(const ManagedZoneLayout &layout, int x, int y, const String &text, bool selected, uint8_t textSize);
void overlay_draw_input_zone(const String &prefix, const String &valueText);
void overlay_draw_zone_borders(const ManagedZoneLayout &layout);
void overlay_compute_two_column_positions(const ManagedZoneLayout &layout, int leftWidth, int rightWidth, int gap, int &leftX, int &rightX);
void overlay_draw_metrics_zone(const ManagedZoneLayout &layout, const MetricsZoneState &state);
String overlay_format_measured_current_raw(float value);
String overlay_format_measured_voltage_raw(float value);
String overlay_format_measured_power_raw(float value);

ManagedZoneLayout setup_managed_zone_layout();
ContentTextPanelLayout setup_content_text_panel_layout(const ManagedZoneLayout &layout);
void setup_draw_screen_base(const UiViewState &state, const char *modeLabel);
void setup_draw_content_panel_title(const ManagedZoneLayout &layout, const ContentTextPanelLayout &panel, const String &title);
int setup_centered_content_text_x(const ManagedZoneLayout &layout, const String &text, uint8_t textSize, uint8_t textFont);
void setup_draw_input_zone(const String &prefix, const String &valueText);
void setup_draw_zone_borders(const ManagedZoneLayout &layout);
void setup_draw_content_stage_line(const ContentTextPanelLayout &panel, const String &text, int x, int y, bool selected);
void setup_clear_blue_content_zone(const ManagedZoneLayout &layout);
String setup_format_transient_current(float currentA);

ManagedZoneLayout home_managed_zone_layout();
void home_draw_zone_borders(const ManagedZoneLayout &layout);
void home_draw_top_status_zone(const ManagedZoneLayout &layout,
                               const String &label,
                               bool loadEnabled,
                               float tempC,
                               float fanTempOnC,
                               float tempCutOffC);
void home_draw_footer_zone(const ManagedZoneLayout &layout);
String home_rtc_timestamp_text();
void home_update_metrics_zone_values(const ManagedZoneLayout &layout,
                                     const MetricsZoneState &state,
                                     bool layoutChanged,
                                     String &lastMetric1,
                                     String &lastMetric2,
                                     String &lastMetric3);
InputZoneRenderLayout home_input_zone_render_layout(const ManagedZoneLayout &layout);
void home_clear_input_zone(const ManagedZoneLayout &layout);
void home_draw_input_zone_shift_hint(const ManagedZoneLayout &layout);
int home_draw_input_zone_highlighted_value(const ManagedZoneLayout &layout,
                                           int startX,
                                           const String &valueText,
                                           int highlightedIndex,
                                           bool cursorVisible);
ContentTextPanelLayout home_content_text_panel_layout(const ManagedZoneLayout &layout);
int home_centered_content_text_x(const ManagedZoneLayout &layout, const String &text, uint8_t textSize, uint8_t textFont);
void home_clear_blue_content_zone(const ManagedZoneLayout &layout);
void home_draw_content_panel_title(const ManagedZoneLayout &layout, const ContentTextPanelLayout &panel, const String &title);
void home_draw_content_stage_line(const ContentTextPanelLayout &panel, const String &text, int x, int y, bool selected);
const char *home_mode_label_for_value(uint8_t mode);
}

#endif
