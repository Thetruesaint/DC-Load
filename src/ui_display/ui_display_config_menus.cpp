#include "../ui_display.h"
#include "ui_display_internal.h"

#include "../ui/ui_view_state.h"

using namespace ui_display_internal;

void uiDisplayRenderConfigRootMenu(const UiViewState &state) {
  const ManagedZoneLayout layout = config_menu_managed_zone_layout();
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
  config_menu_compute_two_column_positions(layout,
                                           uiDisplayTextWidth("2 Calibration", itemSize, font),
                                           uiDisplayTextWidth("4 Clock", itemSize, font),
                                           columnGap,
                                           leftX,
                                           rightX);

  config_menu_clear_content_zone();
  config_menu_draw_title(layout, "CONFIGURATION", titleSize);
  config_menu_draw_column_item(layout, leftX, startY, "1 Protection", state.menuRootSelection == 0, itemSize);
  config_menu_draw_column_item(layout, leftX, startY + (lineHeight + gap), "2 Calibration", state.menuRootSelection == 1, itemSize);
  config_menu_draw_column_item(layout, leftX, startY + ((lineHeight + gap) * 2), "3 FW Update", state.menuRootSelection == 2, itemSize);
  config_menu_draw_column_item(layout, rightX, startY, "4 Clock", state.menuRootSelection == 3, itemSize);
  config_menu_draw_column_item(layout, rightX, startY + (lineHeight + gap), "5 Exit", state.menuRootSelection == 4, itemSize);
  config_menu_draw_battery_setup_set_zone("", "");
}

void uiDisplayRenderProtectionMenu(const UiViewState &state) {
  const ManagedZoneLayout layout = config_menu_managed_zone_layout();
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
  config_menu_compute_two_column_positions(layout,
                                           uiDisplayTextWidth("2 Fan", itemSize, font),
                                           uiDisplayTextWidth("3 Back", itemSize, font),
                                           columnGap,
                                           leftX,
                                           rightX);

  config_menu_clear_content_zone();
  config_menu_draw_title(layout, "PROTECTION", titleSize);
  config_menu_draw_column_item(layout, leftX, startY, "1 Limits", state.protectionMenuSelection == 0, itemSize);
  config_menu_draw_column_item(layout, leftX, startY + (lineHeight + gap), "2 Fan", state.protectionMenuSelection == 1, itemSize);
  config_menu_draw_column_item(layout, rightX, startY, "3 Back", state.protectionMenuSelection == 2, itemSize);
  config_menu_draw_battery_setup_set_zone("", "");
}

void uiDisplayRenderUpdateMenu(const UiViewState &state) {
  const ManagedZoneLayout layout = config_menu_managed_zone_layout();
  const uint8_t titleSize = layout.isLargeDisplay ? 2 : 1;
  const uint8_t itemSize = layout.isLargeDisplay ? 2 : 1;
  const uint8_t font = 2;
  const int titleY = layout.contentY + (layout.isLargeDisplay ? 10 : 8);
  const int lineHeight = uiDisplayFontHeight(itemSize, font);
  const int startY = titleY + uiDisplayFontHeight(titleSize, font) + (layout.isLargeDisplay ? 8 : 12);
  const int columnGap = layout.isLargeDisplay ? 48 : 32;
  int leftX = 0;
  int rightX = 0;
  config_menu_compute_two_column_positions(layout,
                                           uiDisplayTextWidth("1 Start OTA", itemSize, font),
                                           uiDisplayTextWidth("2 Back", itemSize, font),
                                           columnGap,
                                           leftX,
                                           rightX);

  config_menu_clear_content_zone();
  config_menu_draw_title(layout, "FW UPDATE", titleSize);
  config_menu_draw_column_item(layout, leftX, startY, "1 Start OTA", state.updateMenuSelection == 0, itemSize);
  config_menu_draw_column_item(layout, rightX, startY, "2 Back", state.updateMenuSelection == 1, itemSize);
  config_menu_draw_battery_setup_set_zone("", "");
}

void uiDisplayRenderFanSettingsMenu(const UiViewState &state) {
  const ManagedZoneLayout layout = config_menu_managed_zone_layout();
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
  config_menu_compute_two_column_positions(layout,
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

  config_menu_clear_content_zone();
  config_menu_draw_title(layout, "FAN SETTINGS", titleSize);
  config_menu_draw_value_edit_item(leftX,
                                   startY,
                                   "1 Temp: ",
                                   tempValue + "C",
                                   state.fanSettingsMenuSelection == 0,
                                   state.fanEditActive && state.fanSettingsMenuSelection == 0,
                                   itemSize);
  config_menu_draw_value_edit_item(leftX,
                                   startY + (lineHeight + gap),
                                   "2 Hold: ",
                                   holdValue + "s",
                                   state.fanSettingsMenuSelection == 1,
                                   state.fanEditActive && state.fanSettingsMenuSelection == 1,
                                   itemSize);
  config_menu_draw_status_item(leftX,
                               startY + ((lineHeight + gap) * 2),
                               "3 Fan: ",
                               fanState,
                               state.fanSettingsMenuSelection == 2,
                               state.fanOutputOn,
                               itemSize);
  config_menu_draw_column_item(layout, rightX, startY, "4 Back", state.fanSettingsMenuSelection == 3, itemSize);
  config_menu_draw_battery_setup_set_zone("SET> ", state.fanEditActive ? String(state.fanInputText) : String(""));
}

void uiDisplayRenderLimitsMenu(const UiViewState &state) {
  const ManagedZoneLayout layout = config_menu_managed_zone_layout();
  const uint8_t titleSize = layout.isLargeDisplay ? 2 : 1;
  const uint8_t itemSize = layout.isLargeDisplay ? 2 : 1;
  const uint8_t font = 2;
  const int titleY = layout.contentY + (layout.isLargeDisplay ? 10 : 8);
  const int lineHeight = uiDisplayFontHeight(itemSize, font);
  const int gap = layout.isLargeDisplay ? 4 : 6;
  const int startY = titleY + uiDisplayFontHeight(titleSize, font) + (layout.isLargeDisplay ? 8 : 10);
  const String currValue = (state.limitsEditActive && state.limitsMenuField == 0 && state.limitsInputText[0] != '\0')
                               ? String(state.limitsInputText)
                               : config_menu_format_limit_current(state.limitsDraftCurrentA);
  const String powerValue = (state.limitsEditActive && state.limitsMenuField == 1 && state.limitsInputText[0] != '\0')
                                ? String(state.limitsInputText)
                                : config_menu_format_limit_power(state.limitsDraftPowerW);
  const String tempValue = (state.limitsEditActive && state.limitsMenuField == 2 && state.limitsInputText[0] != '\0')
                               ? String(state.limitsInputText)
                               : config_menu_format_limit_temp(state.limitsDraftTempC);

  config_menu_clear_content_zone();
  config_menu_draw_title(layout, "LIMITS", titleSize);
  config_menu_draw_value_edit_line(layout,
                                   startY,
                                   "1 Current: ",
                                   currValue,
                                   state.limitsMenuField == 0,
                                   state.limitsEditActive && state.limitsMenuField == 0,
                                   itemSize);
  config_menu_draw_value_edit_line(layout,
                                   startY + (lineHeight + gap),
                                   "2 Power: ",
                                   powerValue,
                                   state.limitsMenuField == 1,
                                   state.limitsEditActive && state.limitsMenuField == 1,
                                   itemSize);
  config_menu_draw_value_edit_line(layout,
                                   startY + ((lineHeight + gap) * 2),
                                   "3 Temp: ",
                                   tempValue,
                                   state.limitsMenuField == 2,
                                   state.limitsEditActive && state.limitsMenuField == 2,
                                   itemSize);
  config_menu_draw_battery_setup_set_zone("SET> ", state.limitsEditActive ? String(state.limitsInputText) : String(""));
}

void uiDisplayRenderCalibrationMenu(const UiViewState &state) {
  const ManagedZoneLayout layout = config_menu_managed_zone_layout();
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
  config_menu_compute_two_column_positions(layout,
                                           uiDisplayTextWidth("3 Temp", itemSize, font),
                                           uiDisplayTextWidth("6 Back", itemSize, font),
                                           columnGap,
                                           leftX,
                                           rightX);
  const uint8_t selected = (state.calibrationMenuOption > 0) ? static_cast<uint8_t>(state.calibrationMenuOption - 1) : 0;

  config_menu_clear_content_zone();
  config_menu_draw_title(layout, "CALIBRATION", titleSize);
  config_menu_draw_column_item(layout, leftX, startY, "1 Voltage", selected == 0, itemSize);
  config_menu_draw_column_item(layout, leftX, startY + (lineHeight + gap), "2 Current", selected == 1, itemSize);
  config_menu_draw_column_item(layout, leftX, startY + ((lineHeight + gap) * 2), "3 Temp", selected == 2, itemSize);
  config_menu_draw_column_item(layout, rightX, startY, "4 Load", selected == 3, itemSize);
  config_menu_draw_column_item(layout, rightX, startY + (lineHeight + gap), "5 Save", selected == 4, itemSize);
  config_menu_draw_column_item(layout, rightX, startY + ((lineHeight + gap) * 2), "6 Back", selected == 5, itemSize);
  config_menu_draw_battery_setup_set_zone("", "");
}

void uiDisplayRenderClockMenu(const UiViewState &state) {
  const ManagedZoneLayout layout = config_menu_managed_zone_layout();
  const uint8_t titleSize = layout.isLargeDisplay ? 2 : 1;
  const uint8_t itemSize = layout.isLargeDisplay ? 2 : 1;
  const uint8_t font = 2;
  const int titleY = layout.contentY + (layout.isLargeDisplay ? 10 : 8);
  const int lineHeight = uiDisplayFontHeight(itemSize, font);
  const int gap = layout.isLargeDisplay ? 4 : 6;
  const int startY = titleY + uiDisplayFontHeight(titleSize, font) + (layout.isLargeDisplay ? 8 : 10);
  const String dayValue = (state.clockEditActive && state.clockMenuSelection == 0 && state.clockInputText[0] != '\0')
                              ? String(state.clockInputText)
                              : config_menu_two_digits(state.clockDraftDay);
  const String monthValue = (state.clockEditActive && state.clockMenuSelection == 1 && state.clockInputText[0] != '\0')
                                ? String(state.clockInputText)
                                : config_menu_two_digits(state.clockDraftMonth);
  const String yearValue = (state.clockEditActive && state.clockMenuSelection == 2 && state.clockInputText[0] != '\0')
                               ? String(state.clockInputText)
                               : config_menu_two_digits(state.clockDraftYear);
  const String hourValue = (state.clockEditActive && state.clockMenuSelection == 3 && state.clockInputText[0] != '\0')
                               ? String(state.clockInputText)
                               : config_menu_two_digits(state.clockDraftHour);
  const String minuteValue = (state.clockEditActive && state.clockMenuSelection == 4 && state.clockInputText[0] != '\0')
                                 ? String(state.clockInputText)
                                 : config_menu_two_digits(state.clockDraftMinute);
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
  config_menu_compute_two_column_positions(layout,
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

  config_menu_clear_content_zone();
  config_menu_draw_title(layout, "CLOCK", titleSize);
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

  config_menu_draw_column_item(layout, saveX, controlsY, saveLabel, state.clockMenuSelection == 5, itemSize);
  config_menu_draw_column_item(layout, backX, controlsY, backLabel, state.clockMenuSelection == 6, itemSize);
  config_menu_draw_battery_setup_set_zone("SET> ", state.clockEditActive ? String(state.clockInputText) : String(""));
}

void uiDisplayRenderCalibrationSetupMenu(const char *inputText) {
  const ManagedZoneLayout layout = config_menu_managed_zone_layout();
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
  config_menu_compute_two_column_positions(layout,
                                           uiDisplayTextWidth("3 Temp", itemSize, font),
                                           uiDisplayTextWidth("6 Back", itemSize, font),
                                           columnGap,
                                           leftX,
                                           rightX);

  config_menu_clear_content_zone();
  config_menu_draw_title(layout, "CALIBRATION", titleSize);
  config_menu_draw_column_item(layout, leftX, startY, "1 Voltage", false, itemSize);
  config_menu_draw_column_item(layout, leftX, startY + (lineHeight + gap), "2 Current", false, itemSize);
  config_menu_draw_column_item(layout, leftX, startY + ((lineHeight + gap) * 2), "3 Temp", false, itemSize);
  config_menu_draw_column_item(layout, rightX, startY, "4 Load", false, itemSize);
  config_menu_draw_column_item(layout, rightX, startY + (lineHeight + gap), "5 Save", false, itemSize);
  config_menu_draw_column_item(layout, rightX, startY + ((lineHeight + gap) * 2), "6 Back", false, itemSize);
  config_menu_draw_battery_setup_set_zone("SEL> ", (inputText != nullptr) ? String(inputText) : "");
}
