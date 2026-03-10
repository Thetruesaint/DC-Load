#include "ui_state_machine.h"

#include <Arduino.h>
#include <cstring>

#include "../ui_lcd.h"
#include "ui_mode_templates.h"

namespace {
UiScreen g_currentScreen = UiScreen::Home;
uint8_t g_lastMenuRootSelection = 0xFF;

struct LimitsRenderCache {
  float currentA;
  float powerW;
  float tempC;
  uint8_t field;
  bool editActive;
  char inputText[8];
  bool valid;
};

struct HomeRenderCache {
  uint8_t mode;
  bool valid;
};

struct ProtectionRenderCache {
  uint8_t option;
  bool valid;
};

struct FanSettingsRenderCache {
  uint8_t option;
  bool valid;
};

struct CalibrationRenderCache {
  uint8_t option;
  bool valid;
};

HomeRenderCache g_homeCache = {0, false};
ProtectionRenderCache g_protectionCache = {0, false};
FanSettingsRenderCache g_fanSettingsCache = {0, false};
LimitsRenderCache g_limitsCache = {0.0f, 0.0f, 0.0f, 0, false, {'\0'}, false};
CalibrationRenderCache g_calibrationCache = {1, false};

void draw_menu_root_if_needed(const UiViewState &viewState) {
  if (g_lastMenuRootSelection == viewState.menuRootSelection) return;
  ui_draw_config_root_menu(viewState.menuRootSelection);
  g_lastMenuRootSelection = viewState.menuRootSelection;
}

bool home_mode_uses_ui_template(uint8_t mode) {
  return mode == 0 || mode == 1 || mode == 2;
}

void draw_home_template_for_mode(uint8_t mode) {
  if (mode == 0) {
    ui_draw_cc_template();
  } else if (mode == 1) {
    ui_draw_cp_template();
  } else if (mode == 2) {
    ui_draw_cr_template();
  }
}

void draw_home_if_needed(const UiViewState &viewState) {
  if (!home_mode_uses_ui_template(viewState.mode)) {
    g_homeCache.valid = false;
    return;
  }

  if (g_homeCache.valid && g_homeCache.mode == viewState.mode) {
    return;
  }

  draw_home_template_for_mode(viewState.mode);
  g_homeCache.mode = viewState.mode;
  g_homeCache.valid = true;
}

void draw_protection_if_needed(const UiViewState &viewState) {
  if (g_protectionCache.valid && g_protectionCache.option == viewState.protectionMenuSelection) {
    return;
  }

  ui_draw_protection_menu(viewState.protectionMenuSelection);
  g_protectionCache.option = viewState.protectionMenuSelection;
  g_protectionCache.valid = true;
}

void draw_fan_settings_if_needed(const UiViewState &viewState) {
  if (g_fanSettingsCache.valid && g_fanSettingsCache.option == viewState.fanSettingsMenuSelection) {
    return;
  }

  ui_draw_fan_settings_menu(viewState.fanSettingsMenuSelection);
  g_fanSettingsCache.option = viewState.fanSettingsMenuSelection;
  g_fanSettingsCache.valid = true;
}

void draw_limits_menu(const UiViewState &viewState) {
  clearLCD();
  printLCD(4, 0, F("Set Limits"));
  printLCD(0, 1, (viewState.limitsMenuField == 0) ? F(">") : F(" "));
  printLCD(1, 1, F("1-Curr"));
  if (viewState.limitsMenuField == 0) {
    Print_Spaces(10, 1, 9);
    if (viewState.limitsEditActive) {
      printLCD_S(10, 1, String(viewState.limitsInputText));
    } else {
      ui_show_current_limit_value(10, 1, viewState.limitsDraftCurrentA);
    }
  } else {
    ui_show_current_limit_value(10, 1, viewState.limitsDraftCurrentA);
  }

  printLCD(0, 2, (viewState.limitsMenuField == 1) ? F(">") : F(" "));
  printLCD(1, 2, F("2-Power"));
  if (viewState.limitsMenuField == 1) {
    Print_Spaces(11, 2, 8);
    if (viewState.limitsEditActive) {
      printLCD_S(11, 2, String(viewState.limitsInputText));
    } else {
      ui_show_value_number(11, 2, viewState.limitsDraftPowerW, 'W', 1);
    }
  } else {
    ui_show_value_number(11, 2, viewState.limitsDraftPowerW, 'W', 1);
  }

  printLCD(0, 3, (viewState.limitsMenuField == 2) ? F(">") : F(" "));
  printLCD(1, 3, F("3-Temp"));
  Print_Spaces(12, 3, 6);
  if (viewState.limitsMenuField == 2 && viewState.limitsEditActive) {
    printLCD_S(12, 3, String(viewState.limitsInputText));
  } else {
    ui_show_value_number(12, 3, viewState.limitsDraftTempC, ' ', 0);
    printLCDRaw(char(0xDF));
    printLCDRaw(F("C"));
  }
}

void draw_limits_if_needed(const UiViewState &viewState) {
  if (g_limitsCache.valid &&
      g_limitsCache.field == viewState.limitsMenuField &&
      g_limitsCache.currentA == viewState.limitsDraftCurrentA &&
      g_limitsCache.powerW == viewState.limitsDraftPowerW &&
      g_limitsCache.tempC == viewState.limitsDraftTempC &&
      g_limitsCache.editActive == viewState.limitsEditActive &&
      std::strcmp(g_limitsCache.inputText, viewState.limitsInputText) == 0) {
    return;
  }

  draw_limits_menu(viewState);
  g_limitsCache.currentA = viewState.limitsDraftCurrentA;
  g_limitsCache.powerW = viewState.limitsDraftPowerW;
  g_limitsCache.tempC = viewState.limitsDraftTempC;
  g_limitsCache.field = viewState.limitsMenuField;
  g_limitsCache.editActive = viewState.limitsEditActive;
  std::strncpy(g_limitsCache.inputText, viewState.limitsInputText, sizeof(g_limitsCache.inputText) - 1);
  g_limitsCache.inputText[sizeof(g_limitsCache.inputText) - 1] = '\0';
  g_limitsCache.valid = true;
}

void draw_calibration_menu(const UiViewState &viewState) {
  clearLCD();
  printLCD(4, 0, F("Calibration"));
  printLCD(0, 1, (viewState.calibrationMenuOption == 1) ? F(">") : F(" "));
  printLCD(1, 1, F("1-Voltage"));
  printLCD(10, 1, (viewState.calibrationMenuOption == 2) ? F(">") : F(" "));
  printLCD(11, 1, F("2-Current"));
  printLCD(0, 2, (viewState.calibrationMenuOption == 3) ? F(">") : F(" "));
  printLCD(1, 2, F("3-Load"));
  printLCD(10, 2, (viewState.calibrationMenuOption == 4) ? F(">") : F(" "));
  printLCD(11, 2, F("4-Save"));
  printLCD(0, 3, (viewState.calibrationMenuOption == 5) ? F(">") : F(" "));
  printLCD(1, 3, F("5-Back"));
}

void draw_calibration_if_needed(const UiViewState &viewState) {
  if (g_calibrationCache.valid && g_calibrationCache.option == viewState.calibrationMenuOption) {
    return;
  }

  draw_calibration_menu(viewState);
  g_calibrationCache.option = viewState.calibrationMenuOption;
  g_calibrationCache.valid = true;
}

void screen_enter_home(const UiViewState &viewState) {
  g_lastMenuRootSelection = 0xFF;
  g_protectionCache.valid = false;
  g_fanSettingsCache.valid = false;
  g_limitsCache.valid = false;
  g_calibrationCache.valid = false;
  g_homeCache.valid = false;
  draw_home_if_needed(viewState);
}

void screen_update_home(const UiViewState &viewState) {
  draw_home_if_needed(viewState);
  Update_LCD();
}

void screen_render_home(const UiViewState &viewState) { (void)viewState; }

void screen_enter_menu_root(const UiViewState &viewState) {
  g_lastMenuRootSelection = 0xFF;
  draw_menu_root_if_needed(viewState);
}

void screen_update_menu_root(const UiViewState &viewState) {
  draw_menu_root_if_needed(viewState);
}

void screen_render_menu_root(const UiViewState &viewState) { (void)viewState; }

void screen_enter_menu_protection(const UiViewState &viewState) {
  g_protectionCache.valid = false;
  draw_protection_if_needed(viewState);
}

void screen_update_menu_protection(const UiViewState &viewState) {
  draw_protection_if_needed(viewState);
}

void screen_render_menu_protection(const UiViewState &viewState) { (void)viewState; }

void screen_enter_menu_fan_settings(const UiViewState &viewState) {
  g_fanSettingsCache.valid = false;
  draw_fan_settings_if_needed(viewState);
}

void screen_update_menu_fan_settings(const UiViewState &viewState) {
  draw_fan_settings_if_needed(viewState);
}

void screen_render_menu_fan_settings(const UiViewState &viewState) { (void)viewState; }

void screen_enter_menu_limits(const UiViewState &viewState) {
  g_limitsCache.valid = false;
  draw_limits_if_needed(viewState);
}

void screen_update_menu_limits(const UiViewState &viewState) {
  draw_limits_if_needed(viewState);
}

void screen_render_menu_limits(const UiViewState &viewState) { (void)viewState; }

void screen_enter_menu_calibration(const UiViewState &viewState) {
  g_calibrationCache.valid = false;
  draw_calibration_if_needed(viewState);
}

void screen_update_menu_calibration(const UiViewState &viewState) {
  draw_calibration_if_needed(viewState);
}

void screen_render_menu_calibration(const UiViewState &viewState) { (void)viewState; }

void run_screen_enter(UiScreen screen, const UiViewState &viewState) {
  switch (screen) {
    case UiScreen::Home: screen_enter_home(viewState); break;
    case UiScreen::MenuRoot: screen_enter_menu_root(viewState); break;
    case UiScreen::MenuProtection: screen_enter_menu_protection(viewState); break;
    case UiScreen::MenuFanSettings: screen_enter_menu_fan_settings(viewState); break;
    case UiScreen::MenuLimits: screen_enter_menu_limits(viewState); break;
    case UiScreen::MenuCalibration: screen_enter_menu_calibration(viewState); break;
    default: break;
  }
}

void run_screen_update(UiScreen screen, const UiViewState &viewState) {
  switch (screen) {
    case UiScreen::Home: screen_update_home(viewState); break;
    case UiScreen::MenuRoot: screen_update_menu_root(viewState); break;
    case UiScreen::MenuProtection: screen_update_menu_protection(viewState); break;
    case UiScreen::MenuFanSettings: screen_update_menu_fan_settings(viewState); break;
    case UiScreen::MenuLimits: screen_update_menu_limits(viewState); break;
    case UiScreen::MenuCalibration: screen_update_menu_calibration(viewState); break;
    default: break;
  }
}

void run_screen_render(UiScreen screen, const UiViewState &viewState) {
  switch (screen) {
    case UiScreen::Home: screen_render_home(viewState); break;
    case UiScreen::MenuRoot: screen_render_menu_root(viewState); break;
    case UiScreen::MenuProtection: screen_render_menu_protection(viewState); break;
    case UiScreen::MenuFanSettings: screen_render_menu_fan_settings(viewState); break;
    case UiScreen::MenuLimits: screen_render_menu_limits(viewState); break;
    case UiScreen::MenuCalibration: screen_render_menu_calibration(viewState); break;
    default: break;
  }
}
}

void ui_state_machine_reset() {
  g_currentScreen = UiScreen::Home;
  g_lastMenuRootSelection = 0xFF;
  g_protectionCache.valid = false;
  g_fanSettingsCache.valid = false;
  g_limitsCache.valid = false;
  g_calibrationCache.valid = false;
  g_homeCache.valid = false;
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
