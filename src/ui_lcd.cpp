#include "ui_lcd.h"
#include "hw/hw_objects.h"
#include "config/system_constants.h"
#include "ui/ui_symbols.h"
#include "ui/ui_state_cache.h"
#include "ui/ui_state_machine.h"
#include "app/app_input_buffer.h"
#include "app/app_ui_context.h"

#include <cstring>

#ifndef WOKWI_SIMULATION
namespace {
constexpr uint16_t TFT_CELL_W = 18;  // Grid column width for 20x4 layout on TFT
constexpr uint16_t TFT_CELL_H = 24;  // Grid row height for 20x4 layout on TFT
}
#endif

void initLCD(void) {
#ifdef WOKWI_SIMULATION
  lcd.begin(20, 4);
  lcd.backlight();
  lcd.createChar(0, amp_char);
#else
  tft.init();
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextFont(1);
  tft.setTextSize(3);
  tft.setTextWrap(false);
#endif
}

void clearLCD(void) {
#ifdef WOKWI_SIMULATION
  lcd.clear();
#else
  tft.fillScreen(TFT_BLACK);
  tft.setCursor(0, 0);
#endif
}

void setCursorLCD(int col, int row) {
#ifdef WOKWI_SIMULATION
  lcd.setCursor(col, row);
#else
  tft.setCursor(col * TFT_CELL_W, row * TFT_CELL_H);
#endif
}

void blinkOnLCD(void) {
#ifdef WOKWI_SIMULATION
  lcd.blink_on();
#endif
}

void blinkOffLCD(void) {
#ifdef WOKWI_SIMULATION
  lcd.blink_off();
#endif
}

void noCursorLCD(void) {
#ifdef WOKWI_SIMULATION
  lcd.noCursor();
#endif
}

void writeLCD(byte value) {
#ifdef WOKWI_SIMULATION
  lcd.write(value);
#else
  if (value == 0) {
    tft.print('A');
  } else {
    tft.print((char)value);
  }
#endif
}

void printLCDRaw(const String &message) {
#ifdef WOKWI_SIMULATION
  lcd.print(message);
#else
  tft.print(message);
#endif
}

void printLCDRaw(const char *message) {
#ifdef WOKWI_SIMULATION
  lcd.print(message);
#else
  tft.print(message);
#endif
}

void printLCDRaw(const __FlashStringHelper *message) {
#ifdef WOKWI_SIMULATION
  lcd.print(message);
#else
  tft.print(message);
#endif
}

void printLCDRaw(char value) {
#ifdef WOKWI_SIMULATION
  lcd.print(value);
#else
  tft.print(value);
#endif
}

void printLCDRaw(int value) {
#ifdef WOKWI_SIMULATION
  lcd.print(value);
#else
  tft.print(value);
#endif
}

void printLCDRaw(unsigned long value) {
#ifdef WOKWI_SIMULATION
  lcd.print(value);
#else
  tft.print(value);
#endif
}

void printLCDRaw(float value, int decimals) {
#ifdef WOKWI_SIMULATION
  lcd.print(value, decimals);
#else
  tft.print(value, decimals);
#endif
}

void render_keypad_input(uint8_t mode, bool calibrationMode) {
  static uint8_t lastMode = 0xFF;
  static char lastInput[10] = {'\0'};
  static bool rowWasVisible = false;

  const int inputCol = 1;
  const int inputRow = 3;
  const bool visible = (mode != TC && mode != TL && mode != BC);
  const byte maxDigits = calibrationMode ? 6 : 5;
  const char* currentInput = app_input_text();

  if (!visible) {
    if (rowWasVisible) {
      Print_Spaces(inputCol, inputRow, 6);
      rowWasVisible = false;
      lastInput[0] = '\0';
    }
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

//------------ Calculate and Display Actual Voltage, Current, and Power ------------
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

  // Esperar 100ms antes de actualizar el resto del codigo en el LCD
  if (millis() - lastUpdateTime < LCD_RFSH_TIME) return;
  lastUpdateTime = millis();

  // Evitar valores negativos por errores de medicion
  float measuredVoltage = state.measuredVoltage_V;
  float measuredCurrent = state.measuredCurrent_A;
  if (measuredVoltage < 0.011f && state.mode != CA) measuredVoltage = 0.0f;
  if (measuredCurrent < 0.006f && state.mode != CA) measuredCurrent = 0.0f;
  const float power = state.measuredPower_W;

  printLCD(8, 0, state.loadEnabled ? F("ON ") : F("OFF"));

  // Imprimir los valores actualizados, ojo con W que si se corre puede afectar a col 0, row 3
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
      Print_Spaces(state.cursorPosition, 2);
    }
  }

  render_keypad_input(mode, state.mode == CA);
}

//--------------------------- Funciones para el LCD -----------------------------------
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



