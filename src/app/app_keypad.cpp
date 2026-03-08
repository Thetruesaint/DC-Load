#include "app_keypad.h"

#include "../variables.h"
#include "../ui_lcd.h"
#include "../legacy/legacy_hooks.h"
#include "app_msc.h"
#include "../core/core_modes.h"
#include "app_loop.h"

void app_read_keypad(int col, int row) {
  int maxDigits = (Mode == CA) ? 6 : 5;
  customKey = customKeypad.getKey();

  if (customKey == NO_KEY) return;

  app_push_action(ActionType::KeyPressed, 0, customKey);

  if (!app_handle_msc_keys(customKey)) {
    return;
  }

  if (Mode == TC || Mode == TL) return;

  if (customKey == 'U') {
    if (core_mode_is_managed(static_cast<uint8_t>(Mode))) return;
    encoderPosition = encoderPosition + factor;
    encoderPosition = constrain(encoderPosition, 0, maxEncoder);
    return;
  }

  if (customKey == 'D') {
    if (core_mode_is_managed(static_cast<uint8_t>(Mode))) return;
    encoderPosition = encoderPosition - factor;
    return;
  }

  if (customKey == 'L') {
    if (core_mode_is_managed(static_cast<uint8_t>(Mode))) return;
    CuPo--;
    return;
  }

  if (customKey == 'R') {
    if (core_mode_is_managed(static_cast<uint8_t>(Mode))) return;
    CuPo++;
    return;
  }

  if (Mode == BC) return;

  if (customKey >= '0' && customKey <= '9' && c_index < maxDigits) {
    printLCD_S(col + c_index, row, String(customKey));
    numbers[c_index++] = customKey;
    numbers[c_index] = '\0';
  }

  if (customKey == '.' && decimalPoint != '*' && c_index < maxDigits) {
    printLCD(col + c_index, 3, F("."));
    numbers[c_index++] = '.';
    numbers[c_index] = '\0';
    decimalPoint = '*';
  }

  if (customKey == 'E' && c_index != 0) {
    x = atof(numbers);
    if (Mode != CA) {
      reading = x;
      encoderPosition = reading * 1000;
    } else {
      legacy_calibrate(x);
    }
    Print_Spaces(col, row, maxDigits);
    legacy_reset_input_pointers();
  }

  if (customKey == '<' && c_index > 0) {
    c_index--;
    if (numbers[c_index] == '.') decimalPoint = ' ';
    numbers[c_index] = '\0';
    Print_Spaces(col + c_index, row);
  }
}
