#ifndef APP_VALUE_INPUT_VIEW_H
#define APP_VALUE_INPUT_VIEW_H

void app_value_input_view_begin(int col, int row);
void app_value_input_view_render(int col, int row, int maxDigits, const char *text);
void app_value_input_view_end();

#endif
