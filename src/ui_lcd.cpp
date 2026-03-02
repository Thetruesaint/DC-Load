#include "ui_lcd.h"

//------------ Calculate and Display Actual Voltage, Current, and Power ------------
void Update_LCD(void) {
  static unsigned long lastUpdateTime = 0;
  static int blink_cntr = 0;

  if(!modeInitialized) return;  // No actualiza el LCD hasta que el modo dibuje la plantilla y ponga modeInitialized = true

  // Esperar 100ms antes de actualizar el resto del codigo en el LCD
  if (millis() - lastUpdateTime < LCD_RFSH_TIME) return;
  lastUpdateTime = millis();  // Actualizar el tiempo de referencia

  // Evitar valores negativos por errores de medición

  if (voltage < 0.011 && Mode != CA) voltage = 0.0; // Ruido electrico pero quiero verlo en CA
  if (current < 0.006 && Mode != CA) current = 0.0; // Ruido electrico pero quiero verlo en CA
  float power = voltage * current;

  printLCD(8, 0, toggle ? F("ON ") : F("OFF"));  // Indica el estado de la carga

  // Imprimir los valores actualizados, ojo con W que si se corre puede afectar a col 0, row 3
  printLCDNumber(0, 1, current, 'A', (current <= 9.999) ? 3 : 2);
  printLCDNumber(7, 1, voltage, 'v', (voltage <= 9.999) ? 3 : (voltage <= 99.99) ? 2 : 1);
  if (Mode != BC && Mode != CA) {   // lo reemplazo por BatteryCutoffVolts y en modo CA muestro el Punto a muestrar.
    lcd.setCursor(14,1);
    if (power < 10) {Print_Spaces(14, 1); lcd.print(power, 2);}
    else if (power < 100) {lcd.print(power, 2);}
    else {lcd.print(power, 1);}
    lcd.setCursor(19,1);
    lcd.print(F("w"));
  }

  if (Mode != TC && Mode != TL) {  // Evitar mostrar el encoder en modos transitorios

    lcd.setCursor(6, 2);
    if (Mode == CC || Mode == BC || Mode == CA){
      if (reading < 100) Print_Spaces(6, 2);
      if (reading < 10) Print_Spaces(7, 2);
      lcd.print(reading, 3);
    } else {
      if (reading < 100) lcd.print("0");
      if (reading < 10) lcd.print("0");
      lcd.print(reading, 1);
    }
    lcd.setCursor(CuPo, 2); // Cursor en la unidad a modificar

    blink_cntr = (blink_cntr + 1) % 5;
    lcd.setCursor(CuPo, 2);
    if (blink_cntr == 4) {  // Solo en el último ciclo de cada 500ms imprime espacio
      Print_Spaces(CuPo, 2);
    }
  }
}

//--------------------------- Funciones para el LCD -----------------------------------
// Imprimir un mensaje de texto variable
void printLCD_S(int col, int row, const String &message) {
  lcd.setCursor(col, row);
  lcd.print(message);
}

// Imprimir un mensaje con texto almacenado en FLASH
void printLCD(int col, int row, const __FlashStringHelper *message) {
  lcd.setCursor(col, row);
  lcd.print(message);
}

// Imprimir un mensaje con texto almacenado en FLASH
void printLCDNumber(int col, int row, float number, char unit, int decimals) {
  lcd.setCursor(col, row);
  lcd.print(number, decimals);  // Imprime el número con los decimales especificados

  if (unit != '\0' && unit != ' ' && unit != 'A') {  // Solo imprime la unidad si no es nula o espacio en blanco o si no es A
    lcd.print(unit);
  } else if (unit == 'A') {lcd.write(byte(0));}   // Escribe el carácter personalizado
}

// Imprimir n cantidad de espacios " "
void Print_Spaces(int col, int row, byte count) {
  lcd.setCursor(col ,row);
  for (byte i = 0; i < count; i++) {
    lcd.print(F(" "));
  }
}
