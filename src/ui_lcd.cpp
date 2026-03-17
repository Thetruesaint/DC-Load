#include "ui_lcd.h"
#include "hw/hw_objects.h"
#include "config/system_constants.h"
#include "ui/ui_state_cache.h"
#include "ui/ui_state_machine.h"
#include "app/app_input_buffer.h"
#include "app/app_ui_context.h"

#include <cstring>

namespace {
#if defined(TFT_WIDTH) && (TFT_WIDTH <= 240)
constexpr UiGridMetrics kGrid = {20, 4, 12, 16, 0, 0, 2};
#else
constexpr UiGridMetrics kGrid = {20, 4, 18, 24, 0, 0, 3};
#endif

constexpr uint16_t TFT_TEXT_COLOR = TFT_CYAN;
constexpr uint16_t TFT_BG_COLOR = TFT_BLACK;

int cell_origin_x(int col) {
  return kGrid.originXPx + (col * kGrid.cellWidthPx);
}

int cell_origin_y(int row) {
  return kGrid.originYPx + (row * kGrid.cellHeightPx);
}

constexpr int TFT_STATUS_COL = 8;
constexpr int TFT_STATUS_ROW = 0;
constexpr int TFT_STATUS_COLS = 4;
constexpr int TFT_STATUS_ROWS = 1;

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

void clear_tft_cursor_cell(int col, int row) {
  tft.fillRect(cell_origin_x(col), cell_origin_y(row), kGrid.cellWidthPx, kGrid.cellHeightPx - 2, TFT_BG_COLOR);
}
}

void initLCD(void) {
  tft.init();
  tft.setRotation(1);
  tft.fillScreen(TFT_BG_COLOR);
  restore_tft_text_style();
  tft.setTextWrap(false);
}

void clearLCD(void) {
  tft.fillScreen(TFT_BG_COLOR);
  tft.setCursor(kGrid.originXPx, kGrid.originYPx);
  restore_tft_text_style();
}

void setCursorLCD(int col, int row) {
  tft.setCursor(cell_origin_x(col), cell_origin_y(row));
}

const UiGridMetrics &uiGridMetrics() { return kGrid; }

int uiGridPixelX(int col) { return cell_origin_x(col); }

int uiGridPixelY(int row) { return cell_origin_y(row); }

void uiClearCells(int col, int row, byte count) {
  tft.fillRect(cell_origin_x(col), cell_origin_y(row), count * kGrid.cellWidthPx, kGrid.cellHeightPx - 2, TFT_BG_COLOR);
}

void printLCDRaw(const String &message) {
  tft.print(message);
}

void printLCDRaw(const char *message) {
  tft.print(message);
}

void printLCDRaw(const __FlashStringHelper *message) {
  tft.print(message);
}

void printLCDRaw(char value) {
  tft.print(value);
}

void printLCDRaw(int value) {
  tft.print(value);
}

void printLCDRaw(unsigned long value) {
  tft.print(value);
}

void printLCDRaw(float value, int decimals) {
  tft.print(value, decimals);
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
  setCursorLCD(inputCol, inputRow);
  printLCDRaw(currentInput);

  strncpy(lastInput, currentInput, sizeof(lastInput) - 1);
  lastInput[sizeof(lastInput) - 1] = '\0';
  rowWasVisible = true;
  lastMode = mode;
}

void Update_LCD(void) {
  static unsigned long lastUpdateTime = 0;
  static int blink_cntr = 0;

  (void)app_ui_consume_clear_cursor_blink_request();

  const UiViewState &state = ui_state_cache_get();
  if (ui_state_machine_current_screen() != UiScreen::Home) return;
  if (!state.modeInitialized) return;

  if (millis() - lastUpdateTime < LCD_RFSH_TIME) return;
  lastUpdateTime = millis();

  float measuredVoltage = state.measuredVoltage_V;
  float measuredCurrent = state.measuredCurrent_A;
  if (measuredVoltage < 0.011f && state.mode != CA) measuredVoltage = 0.0f;
  if (measuredCurrent < 0.006f && state.mode != CA) measuredCurrent = 0.0f;
  const float power = state.measuredPower_W;

  draw_tft_load_status(state.loadEnabled);

  printLCDNumber(0, 1, measuredCurrent, 'A', (measuredCurrent <= 9.999f) ? 3 : 2);
  printLCDNumber(7, 1, measuredVoltage, 'v', (measuredVoltage <= 9.999f) ? 3 : (measuredVoltage <= 99.99f) ? 2 : 1);
  const uint8_t mode = state.mode;
  if (mode != BC && mode != CA) {
    setCursorLCD(14, 1);
    if (power < 10) {
      uiClearCells(14, 1);
      printLCDRaw(power, 2);
    } else if (power < 100) {
      printLCDRaw(power, 2);
    } else {
      printLCDRaw(power, 1);
    }
    setCursorLCD(19, 1);
    printLCDRaw(F("w"));
  }

  if (mode != TC && mode != TL) {
    setCursorLCD(6, 2);
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
    setCursorLCD(state.cursorPosition, 2);

    blink_cntr = (blink_cntr + 1) % 5;
    setCursorLCD(state.cursorPosition, 2);
    if (blink_cntr == 4) {
      clear_tft_cursor_cell(state.cursorPosition, 2);
    }
  }

  render_keypad_input(mode, state.mode == CA);
}

void printLCD_S(int col, int row, const String &message) {
  setCursorLCD(col, row);
  printLCDRaw(message);
}

void printLCD(int col, int row, const __FlashStringHelper *message) {
  setCursorLCD(col, row);
  printLCDRaw(message);
}

void printLCDNumber(int col, int row, float number, char unit, int decimals) {
  setCursorLCD(col, row);
  printLCDRaw(number, decimals);

  if (unit != '\0' && unit != ' ') {
    printLCDRaw(unit);
  }
}

void Print_Spaces(int col, int row, byte count) {
  uiClearCells(col, row, count);
}
