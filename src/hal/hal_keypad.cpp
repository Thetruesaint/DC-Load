#include "hal_keypad.h"

#include <Arduino.h>
#include <Keypad.h>

namespace {
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

char hal_keypad_read_key() {
  return customKeypad.getKey();
}

bool hal_keypad_is_no_key(char key) {
  return key == NO_KEY;
}
