#include "../app/app_value_input_view.h"

#include "../ui_display.h"

void app_value_input_view_begin(int col, int row) {
  uiGridSetCursor(col, row);
}

void app_value_input_view_render(int col, int row, int maxDigits, const char *text) {
  uiGridPrintString(col, row, String("     ").substring(0, maxDigits));
  uiGridPrintString(col, row, String(text));
}

void app_value_input_view_end() {
}
