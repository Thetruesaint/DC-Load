#include "ui_lcd.h"

void initLCD(void) {
  lcd.begin(20, 4);
  lcd.backlight();
  lcd.createChar(0, amp_char);
}

void clearLCD(void) {
  lcd.clear();
}

void setCursorLCD(int col, int row) {
  lcd.setCursor(col, row);
}

void blinkOnLCD(void) {
  lcd.blink_on();
}

void blinkOffLCD(void) {
  lcd.blink_off();
}

void noCursorLCD(void) {
  lcd.noCursor();
}

void writeLCD(byte value) {
  lcd.write(value);
}

void printLCDRaw(const String &message) {
  lcd.print(message);
}

void printLCDRaw(const char *message) {
  lcd.print(message);
}

void printLCDRaw(const __FlashStringHelper *message) {
  lcd.print(message);
}

void printLCDRaw(char value) {
  lcd.print(value);
}

void printLCDRaw(int value) {
  lcd.print(value);
}

void printLCDRaw(unsigned long value) {
  lcd.print(value);
}

void printLCDRaw(float value, int decimals) {
  lcd.print(value, decimals);
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
