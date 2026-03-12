#include "app_ui_context.h"

namespace {
bool clearCursorBlinkRequested = false;
}

void app_ui_request_clear_cursor_blink() {
  clearCursorBlinkRequested = true;
}

bool app_ui_consume_clear_cursor_blink_request() {
  const bool requested = clearCursorBlinkRequested;
  clearCursorBlinkRequested = false;
  return requested;
}
