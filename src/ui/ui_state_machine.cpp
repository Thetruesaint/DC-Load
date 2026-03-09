#include "ui_state_machine.h"

#include <Arduino.h>

#include "../ui_lcd.h"
#include "ui_mode_templates.h"

namespace {
UiScreen g_currentScreen = UiScreen::Home;
uint8_t g_lastMenuRootSection = 0;

struct LimitsRenderCache {
  float currentA;
  float powerW;
  float tempC;
  uint8_t field;
  bool valid;
};

LimitsRenderCache g_limitsCache = {0.0f, 0.0f, 0.0f, 0, false};
bool g_limitsIntroActive = false;
unsigned long g_limitsIntroUntilMs = 0;

void draw_menu_root_if_needed(const UiViewState &viewState) {
  if (g_lastMenuRootSection == viewState.pendingConfigSection) return;
  ui_draw_config_root_menu(viewState.pendingConfigSection);
  g_lastMenuRootSection = viewState.pendingConfigSection;
}

void draw_limits_menu(const UiViewState &viewState) {
  ui_draw_limits_config_template();

  ui_show_current_limit_value(12, 1, viewState.limitsDraftCurrentA);
  ui_show_value_number(12, 2, viewState.limitsDraftPowerW, 'W', 1);
  ui_show_value_number(12, 3, viewState.limitsDraftTempC, ' ', 0);

  printLCD(11, 1, (viewState.limitsMenuField == 0) ? F("<") : F(" "));
  printLCD(11, 2, (viewState.limitsMenuField == 1) ? F("<") : F(" "));
  printLCD(11, 3, (viewState.limitsMenuField == 2) ? F("<") : F(" "));
}

void draw_limits_if_needed(const UiViewState &viewState) {
  if (g_limitsCache.valid &&
      g_limitsCache.field == viewState.limitsMenuField &&
      g_limitsCache.currentA == viewState.limitsDraftCurrentA &&
      g_limitsCache.powerW == viewState.limitsDraftPowerW &&
      g_limitsCache.tempC == viewState.limitsDraftTempC) {
    return;
  }

  draw_limits_menu(viewState);

  g_limitsCache.currentA = viewState.limitsDraftCurrentA;
  g_limitsCache.powerW = viewState.limitsDraftPowerW;
  g_limitsCache.tempC = viewState.limitsDraftTempC;
  g_limitsCache.field = viewState.limitsMenuField;
  g_limitsCache.valid = true;
}

void screen_enter_home(const UiViewState &viewState) { (void)viewState; }
void screen_update_home(const UiViewState &viewState) { (void)viewState; }
void screen_render_home(const UiViewState &viewState) { (void)viewState; }

void screen_enter_menu_root(const UiViewState &viewState) {
  g_lastMenuRootSection = 0xFF;
  draw_menu_root_if_needed(viewState);
}
void screen_update_menu_root(const UiViewState &viewState) {
  draw_menu_root_if_needed(viewState);
}
void screen_render_menu_root(const UiViewState &viewState) { (void)viewState; }

void screen_enter_menu_limits(const UiViewState &viewState) {
  g_limitsCache.valid = false;
  g_limitsIntroActive = true;
  g_limitsIntroUntilMs = millis() + 1200UL;
  ui_draw_limits_summary(
      viewState.limitsDraftCurrentA,
      viewState.limitsDraftPowerW,
      viewState.limitsDraftTempC);
}
void screen_update_menu_limits(const UiViewState &viewState) {
  if (g_limitsIntroActive) {
    if (millis() < g_limitsIntroUntilMs) {
      return;
    }
    g_limitsIntroActive = false;
    g_limitsCache.valid = false;
  }

  draw_limits_if_needed(viewState);
}
void screen_render_menu_limits(const UiViewState &viewState) { (void)viewState; }

void screen_enter_menu_calibration(const UiViewState &viewState) {
  (void)viewState;
  ui_draw_calibration_setup_menu();
}
void screen_update_menu_calibration(const UiViewState &viewState) { (void)viewState; }
void screen_render_menu_calibration(const UiViewState &viewState) { (void)viewState; }

void run_screen_enter(UiScreen screen, const UiViewState &viewState) {
  switch (screen) {
    case UiScreen::Home: screen_enter_home(viewState); break;
    case UiScreen::MenuRoot: screen_enter_menu_root(viewState); break;
    case UiScreen::MenuLimits: screen_enter_menu_limits(viewState); break;
    case UiScreen::MenuCalibration: screen_enter_menu_calibration(viewState); break;
    default: break;
  }
}

void run_screen_update(UiScreen screen, const UiViewState &viewState) {
  switch (screen) {
    case UiScreen::Home: screen_update_home(viewState); break;
    case UiScreen::MenuRoot: screen_update_menu_root(viewState); break;
    case UiScreen::MenuLimits: screen_update_menu_limits(viewState); break;
    case UiScreen::MenuCalibration: screen_update_menu_calibration(viewState); break;
    default: break;
  }
}

void run_screen_render(UiScreen screen, const UiViewState &viewState) {
  switch (screen) {
    case UiScreen::Home: screen_render_home(viewState); break;
    case UiScreen::MenuRoot: screen_render_menu_root(viewState); break;
    case UiScreen::MenuLimits: screen_render_menu_limits(viewState); break;
    case UiScreen::MenuCalibration: screen_render_menu_calibration(viewState); break;
    default: break;
  }
}
}

void ui_state_machine_reset() {
  g_currentScreen = UiScreen::Home;
  g_lastMenuRootSection = 0;
  g_limitsCache.valid = false;
  g_limitsIntroActive = false;
  g_limitsIntroUntilMs = 0;
}

void ui_state_machine_tick(UiScreen targetScreen, const UiViewState &viewState) {
  if (targetScreen != g_currentScreen) {
    g_currentScreen = targetScreen;
    run_screen_enter(g_currentScreen, viewState);
  }

  run_screen_update(g_currentScreen, viewState);
  run_screen_render(g_currentScreen, viewState);
}

UiScreen ui_state_machine_current_screen() {
  return g_currentScreen;
}
