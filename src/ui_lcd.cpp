#include "ui_lcd.h"
#include "hw/hw_objects.h"
#include "config/system_constants.h"
#include "ui/ui_symbols.h"
#include "ui/ui_state_cache.h"
#include "ui/ui_state_machine.h"
#include "app/app_input_buffer.h"
#include "app/app_ui_context.h"

#include <cstring>

namespace {
#if defined(TFT_WIDTH) && (TFT_WIDTH <= 240)
constexpr uint16_t TFT_CELL_W = 12;  // 20 columns across a 240px-wide sim TFT
constexpr uint16_t TFT_CELL_H = 16;  // 4 rows with a compact text grid
constexpr uint8_t TFT_TEXT_SIZE = 2;
#else
constexpr uint16_t TFT_CELL_W = 18;  // Grid column width for 20x4 layout on TFT
constexpr uint16_t TFT_CELL_H = 24;  // Grid row height for 20x4 layout on TFT
constexpr uint8_t TFT_TEXT_SIZE = 3;
#endif
constexpr uint16_t TFT_TEXT_COLOR = TFT_CYAN;
constexpr uint16_t TFT_BG_COLOR = TFT_BLACK;
constexpr int TFT_STATUS_X = 8 * TFT_CELL_W;
constexpr int TFT_STATUS_Y = 0;
constexpr int TFT_STATUS_W = 4 * TFT_CELL_W;
constexpr int TFT_STATUS_H = TFT_CELL_H - 2;

void restore_tft_text_style() {
  tft.setTextColor(TFT_TEXT_COLOR, TFT_BG_COLOR);
  tft.setTextFont(1);
  tft.setTextSize(TFT_TEXT_SIZE);
}

void draw_tft_load_status(bool enabled) {
  const uint16_t bg = enabled ? TFT_RED : TFT_GREEN;

  tft.fillRect(TFT_STATUS_X, TFT_STATUS_Y, TFT_STATUS_W, TFT_STATUS_H, bg);
  tft.setTextColor(TFT_WHITE, bg);
  tft.setCursor(TFT_STATUS_X, TFT_STATUS_Y);
  tft.print(enabled ? "ON  " : "OFF ");
  restore_tft_text_style();
}

void clear_tft_cursor_cell(int col, int row) {
  const int x = col * TFT_CELL_W;
  const int y = row * TFT_CELL_H;
  tft.fillRect(x, y, TFT_CELL_W, TFT_CELL_H - 2, TFT_BG_COLOR);
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
  tft.setCursor(0, 0);
  restore_tft_text_style();
}

void setCursorLCD(int col, int row) {
  tft.setCursor(col * TFT_CELL_W, row * TFT_CELL_H);
}

void blinkOnLCD(void) {}

void blinkOffLCD(void) {}

void noCursorLCD(void) {}

void writeLCD(byte value) {
  if (value == 0) {
    tft.print('A');
  } else {
    tft.print((char)value);
  }
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

  Print_Spaces(inputCol, inputRow, maxDigits);
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

  if (app_ui_consume_clear_cursor_blink_request()) {
    noCursorLCD();
    blinkOffLCD();
  }

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
      Print_Spaces(14, 1);
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
      if (readingValue < 100) Print_Spaces(6, 2);
      if (readingValue < 10) Print_Spaces(7, 2);
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

  if (unit != '\0' && unit != ' ' && unit != 'A') {
    printLCDRaw(unit);
  } else if (unit == 'A') {
    writeLCD(byte(0));
  }
}

void Print_Spaces(int col, int row, byte count) {
  setCursorLCD(col, row);
  for (byte i = 0; i < count; i++) {
    printLCDRaw(F(" "));
  }
}
