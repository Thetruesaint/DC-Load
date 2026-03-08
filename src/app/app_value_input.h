#ifndef APP_VALUE_INPUT_H
#define APP_VALUE_INPUT_H

char app_wait_key_pressed();
bool app_value_input(int col, int row, int maxDigits, bool decimal);
void app_reset_input_pointers();

#endif
