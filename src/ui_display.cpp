#include "ui_display.h"

#include "app/app_input_buffer.h"
#include "app/app_ui_context.h"
#include "config/system_constants.h"
#include "hw/hw_objects.h"
#include "ui/ui_state_cache.h"
#include "ui/ui_state_machine.h"

#include <cstring>

namespace {
#if defined(TFT_WIDTH) && (TFT_WIDTH <= 240)
constexpr UiGridMetrics kGrid = {20, 4, 12, 16, 0, 0, 2};
#else
constexpr UiGridMetrics kGrid = {20, 4, 18, 24, 0, 0, 3};
#endif

constexpr uint16_t TFT_TEXT_COLOR = TFT_CYAN;
constexpr uint16_t TFT_BG_COLOR = TFT_BLACK;
constexpr int TFT_STATUS_COL = 8;
constexpr int TFT_STATUS_ROW = 0;
constexpr int TFT_STATUS_COLS = 4;
constexpr int TFT_STATUS_ROWS = 1;
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

void draw_horizontal_separator(int y, int width, uint16_t color) {
  tft.drawLine(0, y, width - 1, y, color);
}

void draw_home_zone_borders(int displayW, int displayH, int topBarH, int contentY, int setZoneY, int footerY) {
  uiDisplayDrawRect(0, 0, displayW, displayH, kUiBorder);
  draw_horizontal_separator(topBarH, displayW, kUiBorder);
  draw_horizontal_separator(contentY, displayW, kUiBorder);
  draw_horizontal_separator(setZoneY, displayW, kUiBorder);
  draw_horizontal_separator(footerY, displayW, kUiBorder);
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

int cell_origin_x(int col) {
  return kGrid.originXPx + (col * kGrid.cellWidthPx);
}

int cell_origin_y(int row) {
  return kGrid.originYPx + (row * kGrid.cellHeightPx);
}

void restore_tft_text_style() {
  tft.setTextColor(TFT_TEXT_COLOR, TFT_BG_COLOR);
  tft.setTextFont(1);
  tft.setTextSize(kGrid.textSize);
}

void draw_tft_load_status(bool enabled) {
  const uint16_t bg = enabled ? TFT_RED : TFT_GREEN;
  const int statusX = cell_origin_x(TFT_STATUS_COL);
  const int statusY = cell_origin_y(TFT_STATUS_ROW);
  const int statusW = TFT_STATUS_COLS * kGrid.cellWidthPx;
  const int statusH = (TFT_STATUS_ROWS * kGrid.cellHeightPx) - 2;

  tft.fillRect(statusX, statusY, statusW, statusH, bg);
  tft.setTextColor(TFT_WHITE, bg);
  tft.setCursor(statusX, statusY);
  tft.print(enabled ? "ON  " : "OFF ");
  restore_tft_text_style();
}

void clear_cursor_cell(int col, int row) {
  tft.fillRect(cell_origin_x(col), cell_origin_y(row), kGrid.cellWidthPx, kGrid.cellHeightPx - 2, TFT_BG_COLOR);
}

String format_fixed(float value, int decimals) {
  return String(value, decimals);
}

String format_measured_current(float currentA) {
  return format_fixed(currentA, (currentA <= 9.999f) ? 3 : 2) + "a";
}

String format_measured_voltage(float voltageV) {
  const int decimals = (voltageV <= 9.999f) ? 3 : (voltageV <= 99.99f) ? 2 : 1;
  return format_fixed(voltageV, decimals) + "v";
}

String format_measured_power(float powerW) {
  const int decimals = (powerW < 100.0f) ? 2 : 1;
  return format_fixed(powerW, decimals) + "w";
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

String two_digits(int value) {
  if (value < 10) return String("0") + String(value);
  return String(value);
}

String rtc_timestamp_text() {
  const DateTime now = rtc.now();
  return two_digits(now.day()) + "/" + two_digits(now.month()) + "/" +
         two_digits(now.year() % 100) + " " + two_digits(now.hour()) + ":" +
         two_digits(now.minute()) + ":" + two_digits(now.second());
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

void draw_degree_c_symbol(int x, int y, uint16_t color, uint16_t bg, uint8_t textSize, uint8_t textFont) {
  const int radius = (textSize >= 2) ? 3 : 2;
  uiDisplayDrawCircle(x, y + radius + 1, radius, color);
  uiDisplayPrintStyledAt(x + 5, y - 1, "C", color, bg, textSize, textFont);
}

bool render_managed_home(const UiViewState &state, bool cursorVisible) {
  const int displayW = uiDisplayWidthPx();
  const int displayH = uiDisplayHeightPx();
  const bool isLargeDisplay = displayW >= 400;
  const bool isCcMode = state.mode == CC;
  const bool isCpMode = state.mode == CP;
  const bool isCrMode = state.mode == CR;
  const int topBarH = (displayH * 10) / 100;
  const int metricsH = (displayH * 20) / 100;
  const int setZoneH = (displayH * 10) / 100;
  const int bottomBarH = (displayH * 10) / 100;
  const int footerY = displayH - bottomBarH;
  const int setZoneY = footerY - setZoneH;
  const int contentY = topBarH + metricsH;
  const int contentH = setZoneY - contentY;
  const int metricsBoxY = topBarH;
  const int metricsBoxH = metricsH;
  const uint8_t barTextFont = 2;
  const uint8_t barTextSize = isLargeDisplay ? 2 : 1;
  const uint8_t metricTextFont = 1;
  const uint8_t sectionTextFont = 2;
  const uint8_t sectionTextSize = isLargeDisplay ? 2 : 1;
  const uint8_t footerTextFont = 2;
  const uint8_t footerTextSize = isLargeDisplay ? 2 : 1;

  const int topBarTextY = ((topBarH - uiDisplayFontHeight(barTextSize, barTextFont)) / 2) + (isLargeDisplay ? 1 : 0);
  const char *modeLabel = isCcMode ? "CC" : isCpMode ? "CP" : "CR";
  const String metric1 = format_measured_current(max(0.0f, state.measuredCurrent_A));
  const String metric2 = format_measured_voltage(max(0.0f, state.measuredVoltage_V));
  const String metric3 = format_measured_power(max(0.0f, state.measuredPower_W));
  uint8_t metricTextSize = isLargeDisplay ? 4 : 3;
  int metric1W = 0;
  int metric2W = 0;
  int metric3W = 0;
  int metricGap = 0;
  const int metricsMargin = isLargeDisplay ? 12 : 8;
  const int metricsAvailableW = displayW - (metricsMargin * 2);

  for (; metricTextSize > 0; --metricTextSize) {
    metric1W = uiDisplayTextWidth(metric1, metricTextSize, metricTextFont);
    metric2W = uiDisplayTextWidth(metric2, metricTextSize, metricTextFont);
    metric3W = uiDisplayTextWidth(metric3, metricTextSize, metricTextFont);
    metricGap = isLargeDisplay ? (uiDisplayTextWidth(" ", metricTextSize, metricTextFont) / 2)
                               : (uiDisplayTextWidth(" ", metricTextSize, metricTextFont) / 2);
    const int totalW = metric1W + metric2W + metric3W + (metricGap * 2);
    if (totalW <= metricsAvailableW || metricTextSize == 1) {
      break;
    }
  }

  const int metricsTotalW = metric1W + metric2W + metric3W + (metricGap * 2);
  const int metricsLeftBias = isLargeDisplay ? 8 : 0;
  const int metricsStartX = ((displayW - metricsTotalW) / 2) + metricsLeftBias;
  const int metricsTextY = metricsBoxY + ((metricsBoxH - uiDisplayFontHeight(metricTextSize, metricTextFont)) / 2) - (isLargeDisplay ? 2 : 0);
  const int setTextY = setZoneY + ((setZoneH - uiDisplayFontHeight(sectionTextSize, sectionTextFont)) / 2);
  const String footerVersion = "v2.12";
  const String footerDateTime = rtc_timestamp_text();
  const int footerTextY = footerY + ((bottomBarH - uiDisplayFontHeight(footerTextSize, footerTextFont)) / 2);
  const char *liveInput = app_input_text();
  const bool hasLiveInput = (liveInput != nullptr) && (liveInput[0] != '\0');
  const String setValueText = hasLiveInput
                                  ? String(liveInput)
                                  : (isCcMode ? String(state.readingValue, 3)
                                              : isCpMode ? format_cp_value(state.readingValue)
                                                         : format_cr_value(state.readingValue));
  const String setText = String("SET> ") + setValueText + (isCrMode ? " ohms" : (isCcMode ? "A" : "W"));

  const bool modeChanged = g_ccLastMode != state.mode;
  const bool layoutChanged = !g_ccLayoutDrawn || g_ccLastDisplayW != displayW || g_ccLastDisplayH != displayH || modeChanged;
  if (layoutChanged) {
    uiDisplayClear();
    uiDisplayFillRect(0, 0, displayW, topBarH, kUiAccent);
    uiDisplayFillRect(1, contentY + 1, displayW - 2, contentH - 1, kUiModeAreaBg);
    uiDisplayFillRect(0, footerY, displayW, bottomBarH, kUiAccent);
    draw_home_zone_borders(displayW, displayH, topBarH, contentY, setZoneY, footerY);
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
  }

  const int indicatorRadius = isLargeDisplay ? 10 : ((topBarH >= 30) ? 8 : 6);
  const int indicatorX = (displayW / 6) + 9;
  const int indicatorY = topBarH / 2;
  const String tempText = String(constrain(static_cast<int>(state.tempC), 0, 99));
  if (layoutChanged || g_ccLastLoadEnabled != state.loadEnabled || g_ccLastTempText != tempText) {
    uiDisplayFillRect(0, 0, displayW, topBarH, kUiAccent);
    uiDisplayPrintStyledAt(displayW / 50, topBarTextY, modeLabel, kUiHighlight, kUiAccent, barTextSize, barTextFont);
    const uint16_t loadColor = state.loadEnabled ? kUiLoadOn : kUiLoadOff;
    uiDisplayFillCircle(indicatorX, indicatorY, indicatorRadius, loadColor);
    uiDisplayDrawCircle(indicatorX, indicatorY, indicatorRadius, kUiDark);
    uiDisplayPrintStyledAt(indicatorX + indicatorRadius + 8,
                           topBarTextY,
                           state.loadEnabled ? "ON" : "OFF",
                           kUiHighlight,
                           kUiAccent,
                           barTextSize,
                           barTextFont);
  }

  const int tempBlockX = displayW - (isLargeDisplay ? 86 : 38);
  if (layoutChanged || g_ccLastLoadEnabled != state.loadEnabled || g_ccLastTempText != tempText) {
    uiDisplayPrintStyledAt(tempBlockX, topBarTextY, tempText, kUiHighlight, kUiAccent, barTextSize, barTextFont);
    draw_degree_c_symbol(tempBlockX + uiDisplayTextWidth(tempText, barTextSize, barTextFont) + (isLargeDisplay ? 8 : 6),
                         topBarTextY + (isLargeDisplay ? 3 : 1),
                         kUiHighlight,
                         kUiAccent,
                         barTextSize,
                         barTextFont);
    g_ccLastLoadEnabled = state.loadEnabled;
    g_ccLastTempText = tempText;
    draw_home_zone_borders(displayW, displayH, topBarH, contentY, setZoneY, footerY);
  }

  const int metricPadding = isLargeDisplay ? 10 : 6;
  const int metric1X = metricsStartX - metricGap;
  const int metric2X = metricsStartX + metric1W + metricGap;
  const int metric3X = metricsStartX + metric1W + metricGap + metric2W + metricGap;
  const int metricAreaY = metricsBoxY + 1;
  const int metricAreaH = metricsH - 2;
  if (layoutChanged) {
    uiDisplayFillRect(1, metricsBoxY + 1, displayW - 2, metricsH - 2, kUiBg);
  }
  if (layoutChanged || g_ccLastMetric1 != metric1) {
    const int clearX = max(1, metric1X - metricPadding);
    const int clearW = min(displayW - clearX - 1, metric1W + (metricPadding * 2));
    uiDisplayFillRect(clearX, metricAreaY, clearW, metricAreaH, kUiBg);
    uiDisplayPrintStyledAt(metric1X,
                           metricsTextY,
                           metric1,
                           kUiText,
                           kUiBg,
                           metricTextSize,
                           metricTextFont);
    g_ccLastMetric1 = metric1;
  }
  if (layoutChanged || g_ccLastMetric2 != metric2) {
    const int clearX = max(1, metric2X - metricPadding);
    const int clearW = min(displayW - clearX - 1, metric2W + (metricPadding * 2));
    uiDisplayFillRect(clearX, metricAreaY, clearW, metricAreaH, kUiBg);
    uiDisplayPrintStyledAt(metric2X,
                           metricsTextY,
                           metric2,
                           kUiText,
                           kUiBg,
                           metricTextSize,
                           metricTextFont);
    g_ccLastMetric2 = metric2;
  }
  if (layoutChanged || g_ccLastMetric3 != metric3) {
    const int clearX = max(1, metric3X - metricPadding);
    const int clearW = min(displayW - clearX - 1, metric3W + (metricPadding * 2));
    uiDisplayFillRect(clearX, metricAreaY, clearW, metricAreaH, kUiBg);
    uiDisplayPrintStyledAt(metric3X,
                           metricsTextY,
                           metric3,
                           kUiText,
                           kUiBg,
                           metricTextSize,
                           metricTextFont);
    g_ccLastMetric3 = metric3;
  }

  if (layoutChanged || g_ccLastSetText != setText ||
      (!hasLiveInput &&
       (g_ccLastCursorVisible != cursorVisible || g_ccLastCursorPosition != state.cursorPosition))) {
    uiDisplayFillRect(1, setZoneY + 1, displayW - 2, setZoneH - 2, kUiBg);
    const int setTextX = displayW / 50;
    const String prefixText = "SET> ";
    const int prefixWidth = uiDisplayTextWidth(prefixText, sectionTextSize, sectionTextFont);
    uiDisplayPrintStyledAt(setTextX, setTextY, prefixText, kUiSetColor, kUiBg, sectionTextSize, sectionTextFont);

    if (hasLiveInput) {
      uiDisplayPrintStyledAt(setTextX + prefixWidth,
                             setTextY,
                             setText.substring(5),
                             kUiSetColor,
                             kUiBg,
                             sectionTextSize,
                             sectionTextFont);
    } else {
      const int highlightedIndex = isCcMode
                                       ? cc_cursor_text_index(setValueText, state.cursorPosition)
                                       : isCpMode ? cp_cursor_text_index(setValueText, state.cursorPosition)
                                                  : cr_cursor_text_index(setValueText, state.cursorPosition);
      int currentX = setTextX + prefixWidth;
      for (int i = 0; i < setValueText.length(); ++i) {
        const String digitText = String(setValueText.charAt(i));
        const bool highlighted = (i == highlightedIndex) && cursorVisible;
        uiDisplayPrintStyledAt(currentX,
                               setTextY,
                               digitText,
                               highlighted ? kUiBg : kUiSetColor,
                               highlighted ? kUiSetColor : kUiBg,
                               sectionTextSize,
                               sectionTextFont);
        currentX += uiDisplayTextWidth(digitText, sectionTextSize, sectionTextFont);
      }
      uiDisplayPrintStyledAt(currentX,
                             setTextY,
                             isCrMode ? " ohms" : (isCcMode ? "A" : "W"),
                             kUiSetColor,
                             kUiBg,
                             sectionTextSize,
                             sectionTextFont);
    }
    g_ccLastSetText = setText;
    g_ccLastCursorVisible = cursorVisible;
    g_ccLastCursorPosition = state.cursorPosition;
  }

  if (layoutChanged || g_ccLastFooterText != footerDateTime) {
    uiDisplayFillRect(0, footerY, displayW, bottomBarH, kUiAccent);
    uiDisplayPrintStyledAt(4, footerTextY, footerVersion, kUiText, kUiAccent, footerTextSize, footerTextFont);
    uiDisplayPrintStyledAt(displayW - uiDisplayTextWidth(footerDateTime, footerTextSize, footerTextFont) - 4,
                           footerTextY,
                           footerDateTime,
                           kUiText,
                           kUiAccent,
                           footerTextSize,
                           footerTextFont);
    g_ccLastFooterText = footerDateTime;
    draw_home_zone_borders(displayW, displayH, topBarH, contentY, setZoneY, footerY);
  }
  return true;
}

void render_keypad_input(uint8_t mode, bool calibrationMode) {
  static uint8_t lastMode = 0xFF;
  static char lastInput[10] = {'\0'};
  static bool rowWasVisible = false;

  const int inputCol = 1;
  const int inputRow = 3;
  const bool visible = (mode != TC && mode != TL && mode != BC);
  const byte maxDigits = calibrationMode ? 6 : 5;
  const char *currentInput = app_input_text();

  if (!visible) {
    rowWasVisible = false;
    lastInput[0] = '\0';
    lastMode = mode;
    return;
  }

  if (rowWasVisible && lastMode == mode && strcmp(lastInput, currentInput) == 0) {
    return;
  }

  uiClearCells(inputCol, inputRow, maxDigits);
  uiGridSetCursor(inputCol, inputRow);
  printLCDRaw(currentInput);

  strncpy(lastInput, currentInput, sizeof(lastInput) - 1);
  lastInput[sizeof(lastInput) - 1] = '\0';
  rowWasVisible = true;
  lastMode = mode;
}
}

const UiGridMetrics &uiGridMetrics() { return kGrid; }

int uiGridPixelX(int col) { return cell_origin_x(col); }

int uiGridPixelY(int row) { return cell_origin_y(row); }

int uiDisplayWidthPx() { return tft.width(); }

int uiDisplayHeightPx() { return tft.height(); }

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
  tft.setCursor(kGrid.originXPx, kGrid.originYPx);
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

void uiGridSetCursor(int col, int row) {
  tft.setCursor(cell_origin_x(col), cell_origin_y(row));
}

void uiClearCells(int col, int row, byte count) {
  tft.fillRect(cell_origin_x(col), cell_origin_y(row), count * kGrid.cellWidthPx, kGrid.cellHeightPx - 2, TFT_BG_COLOR);
}

void printLCDRaw(const String &message) { tft.print(message); }

void printLCDRaw(const char *message) { tft.print(message); }

void printLCDRaw(const __FlashStringHelper *message) { tft.print(message); }

void printLCDRaw(char value) { tft.print(value); }

void printLCDRaw(int value) { tft.print(value); }

void printLCDRaw(unsigned long value) { tft.print(value); }

void printLCDRaw(float value, int decimals) { tft.print(value, decimals); }

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

  if (state.mode == CC) {
    render_managed_home(state, true);
    return;
  }

  if (state.mode == CP) {
    render_managed_home(state, true);
    return;
  }

  if (state.mode == CR) {
    render_managed_home(state, true);
    return;
  }

  g_ccLayoutDrawn = false;

  float measuredVoltage = state.measuredVoltage_V;
  float measuredCurrent = state.measuredCurrent_A;
  if (measuredVoltage < 0.011f && state.mode != CA) measuredVoltage = 0.0f;
  if (measuredCurrent < 0.006f && state.mode != CA) measuredCurrent = 0.0f;
  const float power = state.measuredPower_W;

  draw_tft_load_status(state.loadEnabled);

  uiGridPrintNumber(0, 1, measuredCurrent, 'A', (measuredCurrent <= 9.999f) ? 3 : 2);
  uiGridPrintNumber(7, 1, measuredVoltage, 'v', (measuredVoltage <= 9.999f) ? 3 : (measuredVoltage <= 99.99f) ? 2 : 1);
  const uint8_t mode = state.mode;
  if (mode != BC && mode != CA) {
    uiGridSetCursor(14, 1);
    if (power < 10) {
      uiClearCells(14, 1);
      printLCDRaw(power, 2);
    } else if (power < 100) {
      printLCDRaw(power, 2);
    } else {
      printLCDRaw(power, 1);
    }
    uiGridSetCursor(19, 1);
    printLCDRaw(F("w"));
  }

  if (mode != TC && mode != TL) {
    uiGridSetCursor(6, 2);
    const float readingValue = state.readingValue;
    if (mode == CC || mode == BC || mode == CA) {
      if (readingValue < 100) uiClearCells(6, 2);
      if (readingValue < 10) uiClearCells(7, 2);
      printLCDRaw(readingValue, 3);
    } else {
      if (readingValue < 100) printLCDRaw("0");
      if (readingValue < 10) printLCDRaw("0");
      printLCDRaw(readingValue, 1);
    }
    uiGridSetCursor(state.cursorPosition, 2);

    uiGridSetCursor(state.cursorPosition, 2);
    static int blink_cntr = 0;
    blink_cntr = (blink_cntr + 1) % 5;
    if (blink_cntr == 4) {
      clear_cursor_cell(state.cursorPosition, 2);
    }
  }

  render_keypad_input(mode, state.mode == CA);
}

void uiGridPrintString(int col, int row, const String &message) {
  uiGridSetCursor(col, row);
  printLCDRaw(message);
}

void uiGridPrint(int col, int row, const __FlashStringHelper *message) {
  uiGridSetCursor(col, row);
  printLCDRaw(message);
}

void uiGridPrintNumber(int col, int row, float number, char unit, int decimals) {
  uiGridSetCursor(col, row);
  printLCDRaw(number, decimals);

  if (unit != '\0' && unit != ' ') {
    printLCDRaw(unit);
  }
}

void initLCD(void) { uiDisplayInit(); }

void clearLCD(void) { uiDisplayClear(); }

void setCursorLCD(int col, int row) { uiGridSetCursor(col, row); }

void Update_LCD(void) { uiDisplayUpdate(); }

void printLCD_S(int col, int row, const String &message) { uiGridPrintString(col, row, message); }

void printLCD(int col, int row, const __FlashStringHelper *message) { uiGridPrint(col, row, message); }

void printLCDNumber(int col, int row, float number, char unit, int decimals) {
  uiGridPrintNumber(col, row, number, unit, decimals);
}

void Print_Spaces(int col, int row, byte count) { uiClearCells(col, row, count); }
