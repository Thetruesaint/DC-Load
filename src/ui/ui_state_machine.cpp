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
  bool editActive;
  char inputText[8];
  bool valid;
};


struct HomeRenderCache {
  uint8_t mode;
  bool valid;
};

HomeRenderCache g_homeCache = {0, false};
struct CalibrationRenderCache {
  uint8_t option;
  bool valid;
};

LimitsRenderCache g_limitsCache = {0.0f, 0.0f, 0.0f, 0, false, {'\0'}, false};
CalibrationRenderCache g_calibrationCache = {1, false};

void draw_menu_root_if_needed(const UiViewState &viewState) {
  if (g_lastMenuRootSection == viewState.pendingConfigSection) return;
  ui_draw_config_root_menu(viewState.pendingConfigSection);
  g_lastMenuRootSection = viewState.pendingConfigSection;
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
void draw_limits_value_or_input(const UiViewState &viewState, int col, int row, float value, char unit, int decimals) {
  Print_Spaces(col, row, 9);
  if (viewState.limitsEditActive) {
    printLCD_S(col, row, String(viewState.limitsInputText));
    return;
  }
  ui_show_value_number(col, row, value, unit, decimals);
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
      strcmp(g_limitsCache.inputText, viewState.limitsInputText) == 0) {
    return;
  }

  draw_limits_menu(viewState);
  g_limitsCache.currentA = viewState.limitsDraftCurrentA;
  g_limitsCache.powerW = viewState.limitsDraftPowerW;
  g_limitsCache.tempC = viewState.limitsDraftTempC;
  g_limitsCache.field = viewState.limitsMenuField;
  g_limitsCache.editActive = viewState.limitsEditActive;
  strncpy(g_limitsCache.inputText, viewState.limitsInputText, sizeof(g_limitsCache.inputText) - 1);
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
  printLCD(0, 3, F("< Back"));
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
  g_lastMenuRootSection = 0;
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
  g_lastMenuRootSection = 0xFF;
  draw_menu_root_if_needed(viewState);
}
void screen_update_menu_root(const UiViewState &viewState) {
  draw_menu_root_if_needed(viewState);
}
void screen_render_menu_root(const UiViewState &viewState) { (void)viewState; }

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




