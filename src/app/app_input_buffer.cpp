#include "app_input_buffer.h"

#include "../variables.h"

char app_input_read_key() {
  customKey = customKeypad.getKey();
  return customKey;
}

char app_input_wait_key() {
  char key;
  do {
    key = app_input_read_key();
  } while (key == NO_KEY);
  return key;
}

void app_input_reset() {
  c_index = 0;
  numbers[c_index] = '\0';
  decimalPoint = ' ';
}

uint8_t app_input_length() {
  return c_index;
}

const char* app_input_text() {
  return numbers;
}

bool app_input_append_digit(char key, int maxDigits) {
  if (key < '0' || key > '9') return false;
  if (c_index >= maxDigits) return false;

  numbers[c_index++] = key;
  numbers[c_index] = '\0';
  return true;
}

bool app_input_append_decimal(int maxDigits) {
  if (decimalPoint == '*') return false;
  if (c_index >= maxDigits) return false;

  numbers[c_index++] = '.';
  numbers[c_index] = '\0';
  decimalPoint = '*';
  return true;
}

bool app_input_backspace() {
  if (c_index == 0) return false;

  c_index--;
  if (numbers[c_index] == '.') decimalPoint = ' ';
  numbers[c_index] = '\0';
  return true;
}

float app_input_parse_float() {
  return atof(numbers);
}
