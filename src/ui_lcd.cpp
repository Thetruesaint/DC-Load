#include "ui_lcd.h"

#ifndef WOKWI_SIMULATION
namespace {
constexpr uint16_t TFT_CELL_W = 12;  // Grid column width for 20x4 layout
constexpr uint16_t TFT_CELL_H = 18;  // Grid row height for 20x4 layout
}
#endif

void initLCD(void) {
#ifdef WOKWI_SIMULATION
  lcd.begin(20, 4);
  lcd.backlight();
  lcd.createChar(0, amp_char);
#else
  tft.init();
  tft.setRotation(3);
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextFont(1);
  tft.setTextSize(2);
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

//------------ Calculate and Display Actual Voltage, Current, and Power ------------
void Update_LCD(void) {
  static unsigned long lastUpdateTime = 0;
  static int blink_cntr = 0;

  if (!modeInitialized) return;  // No actualiza el LCD hasta que el modo dibuje la plantilla y ponga modeInitialized = true

  // Esperar 100ms antes de actualizar el resto del codigo en el LCD
  if (millis() - lastUpdateTime < LCD_RFSH_TIME) return;
  lastUpdateTime = millis();  // Actualizar el tiempo de referencia

  // Evitar valores negativos por errores de medicion
  if (voltage < 0.011 && Mode != CA) voltage = 0.0;
  if (current < 0.006 && Mode != CA) current = 0.0;
  float power = voltage * current;

  printLCD(8, 0, toggle ? F("ON ") : F("OFF"));

  // Imprimir los valores actualizados, ojo con W que si se corre puede afectar a col 0, row 3
  printLCDNumber(0, 1, current, 'A', (current <= 9.999) ? 3 : 2);
  printLCDNumber(7, 1, voltage, 'v', (voltage <= 9.999) ? 3 : (voltage <= 99.99) ? 2 : 1);
  if (Mode != BC && Mode != CA) {
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

  if (Mode != TC && Mode != TL) {
    setCursorLCD(6, 2);
    if (Mode == CC || Mode == BC || Mode == CA) {
      if (reading < 100) Print_Spaces(6, 2);
      if (reading < 10) Print_Spaces(7, 2);
      printLCDRaw(reading, 3);
    } else {
      if (reading < 100) printLCDRaw("0");
      if (reading < 10) printLCDRaw("0");
      printLCDRaw(reading, 1);
    }
    setCursorLCD(CuPo, 2);

    blink_cntr = (blink_cntr + 1) % 5;
    setCursorLCD(CuPo, 2);
    if (blink_cntr == 4) {
      Print_Spaces(CuPo, 2);
    }
  }
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
