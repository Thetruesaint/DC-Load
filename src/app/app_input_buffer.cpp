#include "app_input_buffer.h"

#include <Keypad.h>

namespace {
char inputText[10] = {'\0'};
byte inputIndex = 0;
bool hasDecimalPoint = false;

const uint8_t ROWS = 5;
const uint8_t COLS = 4;
char hexaKeys[ROWS][COLS] = {
  {'1', '2', '3', 'M'},
  {'4', '5', '6', 'C'},
  {'7', '8', '9', 'S'},
  {'<', '0', '.', 'E'},
  {'U', 'D', 'L', 'R'}
};
byte rowPins[ROWS] = {33, 5, 27, 12, 0};
byte colPins[COLS] = {34, 35, 19, 26};
Keypad customKeypad = Keypad(makeKeymap(hexaKeys), rowPins, colPins, ROWS, COLS);
}

char app_input_read_key() {
  return customKeypad.getKey();
}

char app_input_wait_key() {
  char key;
  do {
    key = app_input_read_key();
  } while (key == NO_KEY);
  return key;
}

void app_input_reset() {
  inputIndex = 0;
  inputText[inputIndex] = '\0';
  hasDecimalPoint = false;
}

uint8_t app_input_length() {
  return inputIndex;
}

const char* app_input_text() {
  return inputText;
}

bool app_input_append_digit(char key, int maxDigits) {
  if (key < '0' || key > '9') return false;
  if (inputIndex >= maxDigits) return false;

  inputText[inputIndex++] = key;
  inputText[inputIndex] = '\0';
  return true;
}

bool app_input_append_decimal(int maxDigits) {
  if (hasDecimalPoint) return false;
  if (inputIndex >= maxDigits) return false;

  inputText[inputIndex++] = '.';
  inputText[inputIndex] = '\0';
  hasDecimalPoint = true;
  return true;
}

bool app_input_backspace() {
  if (inputIndex == 0) return false;

  inputIndex--;
  if (inputText[inputIndex] == '.') hasDecimalPoint = false;
  inputText[inputIndex] = '\0';
  return true;
}

float app_input_parse_float() {
  return atof(inputText);
}

