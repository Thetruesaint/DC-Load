#include "ui_state_machine.h"

#include "ui_mode_templates.h"

namespace {
UiScreen g_currentScreen = UiScreen::Home;
uint8_t g_lastMenuRootSection = 0;

void draw_menu_root_if_needed(const UiViewState &viewState) {
  if (g_lastMenuRootSection == viewState.pendingConfigSection) return;
  ui_draw_config_root_menu(viewState.pendingConfigSection);
  g_lastMenuRootSection = viewState.pendingConfigSection;
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
  ui_draw_limits_summary(viewState.currentCutOffA, viewState.powerCutOffW, viewState.tempCutOffC);
}
void screen_update_menu_limits(const UiViewState &viewState) { (void)viewState; }
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
