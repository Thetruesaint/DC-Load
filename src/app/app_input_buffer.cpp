#include "app_input_buffer.h"

#include "../hal/hal_keypad.h"

namespace {
char inputText[10] = {'\0'};
byte inputIndex = 0;
bool hasDecimalPoint = false;
}

char app_input_read_key() {
  return hal_keypad_read_key();
}

char app_input_wait_key() {
  char key;
  do {
    key = app_input_read_key();
  } while (app_input_is_no_key(key));
  return key;
}

bool app_input_is_no_key(char key) {
  return hal_keypad_is_no_key(key);
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
