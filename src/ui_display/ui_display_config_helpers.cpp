#include "../ui_display.h"
#include "ui_display_internal.h"

using namespace ui_display_internal;

void draw_config_value_edit_item(int x,
                                 int y,
                                 const String &label,
                                 const String &value,
                                 bool selected,
                                 bool editingValue,
                                 uint8_t textSize = 1);

void draw_config_title(const ManagedZoneLayout &layout, const String &title, uint8_t textSize = 0) {
  if (textSize == 0) textSize = layout.isLargeDisplay ? 2 : 1;
  const uint8_t font = 2;
  const int x = (layout.displayW - uiDisplayTextWidth(title, textSize, font)) / 2;
  const int y = layout.contentY + (layout.isLargeDisplay ? 10 : 8);
  uiDisplayPrintStyledAt(x, y, title, kUiText, kUiModeAreaBg, textSize, font);
}

void draw_config_line(const ManagedZoneLayout &layout, int y, const String &text, bool selected, uint8_t textSize = 0) {
  if (textSize == 0) textSize = layout.isLargeDisplay ? 2 : 1;
  const uint8_t font = 2;
  const int h = uiDisplayFontHeight(textSize, font);
  const int x = (layout.displayW - uiDisplayTextWidth(text, textSize, font)) / 2;
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

void draw_config_value_edit_line(const ManagedZoneLayout &layout,
                                 int y,
                                 const String &label,
                                 const String &value,
                                 bool selected,
                                 bool editingValue,
                                 uint8_t textSize) {
  const uint8_t font = 2;
  const int labelW = uiDisplayTextWidth(label, textSize, font);
  const int valueW = uiDisplayTextWidth(value, textSize, font);
  const int totalW = labelW + valueW;
  const int x = (layout.displayW - totalW) / 2;
  draw_config_value_edit_item(x, y, label, value, selected, editingValue, textSize);
}

void draw_config_value_edit_item(int x,
                                 int y,
                                 const String &label,
                                 const String &value,
                                 bool selected,
                                 bool editingValue,
                                 uint8_t textSize) {
  const uint8_t font = 2;
  const int labelW = uiDisplayTextWidth(label, textSize, font);
  const int valueW = uiDisplayTextWidth(value, textSize, font);
  const int totalW = labelW + valueW;
  const int h = uiDisplayFontHeight(textSize, font);

  if (selected || editingValue) {
    uiDisplayFillRect(x - 4, y - 1, totalW + 8, h + 2, selected && !editingValue ? kUiHighlight : kUiModeAreaBg);
    if (editingValue) {
      uiDisplayFillRect(x + labelW - 2, y - 1, valueW + 6, h + 2, kUiSetColor);
    }
  }

  uiDisplayPrintStyledAt(x,
                         y,
                         label,
                         selected ? kUiDark : kUiText,
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
                             uint8_t textSize) {
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

namespace ui_display_internal {
String config_menu_two_digits(int value) {
  return (value < 10) ? "0" + String(value) : String(value);
}

ManagedZoneLayout config_menu_managed_zone_layout() {
  return shared_managed_zone_layout();
}

void config_menu_draw_title(const ManagedZoneLayout &layout, const String &title, uint8_t textSize) {
  ::draw_config_title(layout, title, textSize);
}

void config_menu_draw_line(const ManagedZoneLayout &layout, int y, const String &text, bool selected, uint8_t textSize) {
  ::draw_config_line(layout, y, text, selected, textSize);
}

void config_menu_draw_value_edit_line(const ManagedZoneLayout &layout,
                                      int y,
                                      const String &label,
                                      const String &value,
                                      bool selected,
                                      bool editingValue,
                                      uint8_t textSize) {
  ::draw_config_value_edit_line(layout, y, label, value, selected, editingValue, textSize);
}

void config_menu_draw_value_edit_item(int x,
                                      int y,
                                      const String &label,
                                      const String &value,
                                      bool selected,
                                      bool editingValue,
                                      uint8_t textSize) {
  ::draw_config_value_edit_item(x, y, label, value, selected, editingValue, textSize);
}

void config_menu_draw_status_item(int x,
                                  int y,
                                  const String &label,
                                  const String &value,
                                  bool selected,
                                  bool active,
                                  uint8_t textSize) {
  ::draw_config_status_item(x, y, label, value, selected, active, textSize);
}

void config_menu_draw_column_item(const ManagedZoneLayout &layout, int x, int y, const String &text, bool selected, uint8_t textSize) {
  ::draw_config_column_item(layout, x, y, text, selected, textSize);
}

void config_menu_compute_two_column_positions(const ManagedZoneLayout &layout, int leftWidth, int rightWidth, int gap, int &leftX, int &rightX) {
  ::compute_two_column_positions(layout, leftWidth, rightWidth, gap, leftX, rightX);
}

String config_menu_format_limit_current(float value) {
  return ::format_limit_current(value);
}

String config_menu_format_limit_power(float value) {
  return ::format_limit_power(value);
}

String config_menu_format_limit_temp(float value) {
  return ::format_limit_temp(value);
}
}  // namespace ui_display_internal
