#include "app_value_input.h"

#include "../ui_lcd.h"
#include "app_msc.h"
#include "app_input_buffer.h"
#include "app_loop.h"
#include "app_value_result_context.h"

char app_wait_key_pressed() {
  return app_input_wait_key();
}

void app_reset_input_pointers() {
  app_input_reset();
}

bool app_value_input(int col, int row, int maxDigits, bool decimal) {
  app_reset_input_pointers();
  setCursorLCD(col, row);
  blinkOnLCD();

  while (true) {
    char key = app_wait_key_pressed();
    app_push_action(make_key_pressed_action(key));

    if (!app_handle_msc_keys(key)) {
      return false;
    }

    bool handled = app_input_append_digit(key, maxDigits);
    if (!handled && key == '.' && decimal) {
      handled = app_input_append_decimal(maxDigits);
    }
    if (!handled && key == '<') {
      handled = app_input_backspace();
    }

    if (!handled && key == 'E' && app_input_length() > 0) {
      app_value_result_set(app_input_parse_float());
      app_reset_input_pointers();
      noCursorLCD();
      blinkOffLCD();
      return true;
    }

    printLCD_S(col, row, String("     ").substring(0, maxDigits));
    printLCD_S(col, row, String(app_input_text()));
  }
}
