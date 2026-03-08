#include "app_value_input.h"

#include "../funciones.h"

char app_wait_key_pressed() {
  char key;
  do {
    key = customKeypad.getKey();
  } while (key == NO_KEY);
  return key;
}

void app_reset_input_pointers() {
  c_index = 0;
  numbers[c_index] = '\0';
  decimalPoint = ' ';
}

bool app_value_input(int col, int row, int maxDigits, bool decimal) {
  app_reset_input_pointers();
  setCursorLCD(col, row);
  blinkOnLCD();

  while (true) {
    customKey = app_wait_key_pressed();

    if (!Handle_MSC_Keys(customKey)) {
      return false;
    }

    if (customKey >= '0' && customKey <= '9') {
      if (c_index < maxDigits) {
        numbers[c_index++] = customKey;
        numbers[c_index] = '\0';
      }
    } else if (customKey == '.' && decimalPoint != '*' && decimal) {
      if (c_index < maxDigits) {
        numbers[c_index++] = '.';
        numbers[c_index] = '\0';
        decimalPoint = '*';
      }
    } else if (customKey == '<' && c_index > 0) {
      c_index--;
      if (numbers[c_index] == '.') decimalPoint = ' ';
      numbers[c_index] = '\0';
    } else if (customKey == 'E') {
      if (c_index > 0) {
        x = atof(numbers);
        app_reset_input_pointers();
        noCursorLCD();
        blinkOffLCD();
        return true;
      }
    }

    printLCD_S(col, row, String("     ").substring(0, maxDigits));
    printLCD_S(col, row, String(numbers));
  }
}
