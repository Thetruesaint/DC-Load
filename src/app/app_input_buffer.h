#ifndef APP_INPUT_BUFFER_H
#define APP_INPUT_BUFFER_H

#include <Arduino.h>

char app_input_read_key();
char app_input_wait_key();

void app_input_reset();
uint8_t app_input_length();
const char* app_input_text();
bool app_input_append_digit(char key, int maxDigits);
bool app_input_append_decimal(int maxDigits);
bool app_input_backspace();
float app_input_parse_float();

#endif
