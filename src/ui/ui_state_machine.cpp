#include "ui_state_machine.h"

#include <Arduino.h>
#include <cstring>

#include "../app/app_ota.h"
#include "../ui_lcd.h"
#include "../app/app_calibration_context.h"
#include "../config/system_constants.h"
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
  float batteryCutoffVolts;
  char batteryType[8];
  bool valid;
};

struct ProtectionRenderCache {
  uint8_t option;
  bool valid;
};

struct TestsRenderCache {
  uint8_t option;
  bool fanOn;
  bool valid;
};

struct FwUpdateRenderCache {
  char status[21];
  char detail[21];
  char hint[21];
  bool valid;
};

struct FanSettingsRenderCache {
  uint8_t option;
  float tempC;
  float holdSeconds;
  bool editActive;
  bool fanOn;
  char inputText[8];
  bool valid;
};

struct CalibrationRenderCache {
  uint8_t option;
  bool valid;
};

struct BatterySetupRenderCache {
  uint8_t stage;
  char batteryType[8];
  char inputText[8];
  bool valid;
};

struct TransientContSetupRenderCache {
  uint8_t stage;
  float lowCurrentA;
  float highCurrentA;
  float periodMs;
  char inputText[8];
  bool valid;
};

struct TransientListSetupRenderCache {
  uint8_t stage;
  uint8_t stepCount;
  uint8_t stepIndex;
  uint8_t field;
  float currentA;
  float periodMs;
  char inputText[8];
  bool valid;
};

HomeRenderCache g_homeCache = {0, 0.0f, {'\0'}, false};
ProtectionRenderCache g_protectionCache = {0, false};
TestsRenderCache g_testsCache = {0, false, false};
FwUpdateRenderCache g_fwUpdateCache = {{'\0'}, {'\0'}, {'\0'}, false};
FanSettingsRenderCache g_fanSettingsCache = {0, 0.0f, 0.0f, false, false, {'\0'}, false};
LimitsRenderCache g_limitsCache = {0.0f, 0.0f, 0.0f, 0, false, {'\0'}, false};
CalibrationRenderCache g_calibrationCache = {1, false};
BatterySetupRenderCache g_batterySetupCache = {0, {'\0'}, {'\0'}, false};
TransientContSetupRenderCache g_transientContSetupCache = {0, 0.0f, 0.0f, 0.0f, {'\0'}, false};
TransientListSetupRenderCache g_transientListSetupCache = {0, 0, 0, 0, 0.0f, 0.0f, {'\0'}, false};

void draw_menu_root_if_needed(const UiViewState &viewState) {
  if (g_lastMenuRootSelection == viewState.menuRootSelection) return;
  ui_draw_config_root_menu(viewState.menuRootSelection);
  g_lastMenuRootSelection = viewState.menuRootSelection;
}

bool home_mode_uses_ui_template(uint8_t mode) {
  return mode == 0 || mode == 1 || mode == 2 || mode == 3;
}

void draw_home_template_for_mode(const UiViewState &viewState) {
  if (viewState.mode == 0) {
    ui_draw_cc_template();
  } else if (viewState.mode == 1) {
    ui_draw_cp_template();
  } else if (viewState.mode == 2) {
    ui_draw_cr_template();
  } else if (viewState.mode == 3) {
    ui_draw_bc_template(viewState.batteryCutoffVolts,
                        viewState.batteryLife,
                        String(viewState.batteryType));
  }
}

void draw_home_if_needed(const UiViewState &viewState) {
  if (!home_mode_uses_ui_template(viewState.mode)) {
    g_homeCache.valid = false;
    return;
  }

  if (g_homeCache.valid &&
      g_homeCache.mode == viewState.mode &&
      g_homeCache.batteryCutoffVolts == viewState.batteryCutoffVolts &&
      std::strcmp(g_homeCache.batteryType, viewState.batteryType) == 0) {
    return;
  }

  draw_home_template_for_mode(viewState);
  g_homeCache.mode = viewState.mode;
  g_homeCache.batteryCutoffVolts = viewState.batteryCutoffVolts;
  std::strncpy(g_homeCache.batteryType, viewState.batteryType, sizeof(g_homeCache.batteryType) - 1);
  g_homeCache.batteryType[sizeof(g_homeCache.batteryType) - 1] = '\0';
  g_homeCache.valid = true;
}

void draw_battery_setup_task_if_needed() {
  if (g_batterySetupCache.valid && g_batterySetupCache.stage == 0) {
    return;
  }

  ui_draw_battery_task_menu();
  g_batterySetupCache.stage = 0;
  g_batterySetupCache.valid = true;
}

void draw_battery_setup_custom_if_needed(const UiViewState &viewState) {
  if (g_batterySetupCache.valid &&
      g_batterySetupCache.stage == 1 &&
      std::strcmp(g_batterySetupCache.batteryType, viewState.batteryType) == 0 &&
      std::strcmp(g_batterySetupCache.inputText, viewState.batteryInputText) == 0) {
    return;
  }

  ui_draw_battery_custom_cutoff_prompt(String(viewState.batteryType));
  ui_prepare_value_input_prompt(7, 3, 5);
  printLCD_S(7, 3, String(viewState.batteryInputText));
  std::strncpy(g_batterySetupCache.batteryType, viewState.batteryType, sizeof(g_batterySetupCache.batteryType) - 1);
  g_batterySetupCache.batteryType[sizeof(g_batterySetupCache.batteryType) - 1] = '\0';
  std::strncpy(g_batterySetupCache.inputText, viewState.batteryInputText, sizeof(g_batterySetupCache.inputText) - 1);
  g_batterySetupCache.inputText[sizeof(g_batterySetupCache.inputText) - 1] = '\0';
  g_batterySetupCache.stage = 1;
  g_batterySetupCache.valid = true;
}

void draw_battery_setup_cells_if_needed(const UiViewState &viewState) {
  if (g_batterySetupCache.valid &&
      g_batterySetupCache.stage == 2 &&
      std::strcmp(g_batterySetupCache.batteryType, viewState.batteryType) == 0 &&
      std::strcmp(g_batterySetupCache.inputText, viewState.batteryInputText) == 0) {
    return;
  }

  ui_draw_battery_cell_count_prompt(String(viewState.batteryType));
  ui_prepare_value_input_prompt(9, 2, 1);
  printLCD_S(9, 2, String(viewState.batteryInputText));
  std::strncpy(g_batterySetupCache.batteryType, viewState.batteryType, sizeof(g_batterySetupCache.batteryType) - 1);
  g_batterySetupCache.batteryType[sizeof(g_batterySetupCache.batteryType) - 1] = '\0';
  std::strncpy(g_batterySetupCache.inputText, viewState.batteryInputText, sizeof(g_batterySetupCache.inputText) - 1);
  g_batterySetupCache.inputText[sizeof(g_batterySetupCache.inputText) - 1] = '\0';
  g_batterySetupCache.stage = 2;
  g_batterySetupCache.valid = true;
}

void draw_protection_if_needed(const UiViewState &viewState) {
  if (g_protectionCache.valid && g_protectionCache.option == viewState.protectionMenuSelection) {
    return;
  }

  ui_draw_protection_menu(viewState.protectionMenuSelection);
  g_protectionCache.option = viewState.protectionMenuSelection;
  g_protectionCache.valid = true;
}

void draw_tests_if_needed(const UiViewState &viewState) {
  if (g_testsCache.valid &&
      g_testsCache.option == viewState.testsMenuSelection &&
      g_testsCache.fanOn == viewState.fanManualStateOn) {
    return;
  }

  ui_draw_tests_menu(viewState.testsMenuSelection, viewState.fanManualStateOn);
  g_testsCache.option = viewState.testsMenuSelection;
  g_testsCache.fanOn = viewState.fanManualStateOn;
  g_testsCache.valid = true;
}

void draw_fan_settings_if_needed(const UiViewState &viewState) {
  if (g_fanSettingsCache.valid &&
      g_fanSettingsCache.option == viewState.fanSettingsMenuSelection &&
      g_fanSettingsCache.tempC == viewState.fanDraftTempC &&
      g_fanSettingsCache.holdSeconds == viewState.fanDraftHoldSeconds &&
      g_fanSettingsCache.editActive == viewState.fanEditActive &&
      g_fanSettingsCache.fanOn == viewState.fanManualStateOn &&
      std::strcmp(g_fanSettingsCache.inputText, viewState.fanInputText) == 0) {
    return;
  }

  ui_draw_fan_settings_menu(viewState.fanSettingsMenuSelection,
                            viewState.fanDraftTempC,
                            viewState.fanDraftHoldSeconds,
                            viewState.fanEditActive,
                            viewState.fanInputText,
                            viewState.fanManualStateOn);
  g_fanSettingsCache.option = viewState.fanSettingsMenuSelection;
  g_fanSettingsCache.tempC = viewState.fanDraftTempC;
  g_fanSettingsCache.holdSeconds = viewState.fanDraftHoldSeconds;
  g_fanSettingsCache.editActive = viewState.fanEditActive;
  g_fanSettingsCache.fanOn = viewState.fanManualStateOn;
  std::strncpy(g_fanSettingsCache.inputText, viewState.fanInputText, sizeof(g_fanSettingsCache.inputText) - 1);
  g_fanSettingsCache.inputText[sizeof(g_fanSettingsCache.inputText) - 1] = '\0';
  g_fanSettingsCache.valid = true;
}

void draw_fw_update_if_needed() {
  const char *status = app_ota_status_text();
  const char *detail = app_ota_detail_text();
  const char *hint = app_ota_hint_text();

  if (g_fwUpdateCache.valid &&
      std::strcmp(g_fwUpdateCache.status, status) == 0 &&
      std::strcmp(g_fwUpdateCache.detail, detail) == 0 &&
      std::strcmp(g_fwUpdateCache.hint, hint) == 0) {
    return;
  }

  ui_draw_fw_update_screen(status, detail, hint);
  std::strncpy(g_fwUpdateCache.status, status, sizeof(g_fwUpdateCache.status) - 1);
  g_fwUpdateCache.status[sizeof(g_fwUpdateCache.status) - 1] = '\0';
  std::strncpy(g_fwUpdateCache.detail, detail, sizeof(g_fwUpdateCache.detail) - 1);
  g_fwUpdateCache.detail[sizeof(g_fwUpdateCache.detail) - 1] = '\0';
  std::strncpy(g_fwUpdateCache.hint, hint, sizeof(g_fwUpdateCache.hint) - 1);
  g_fwUpdateCache.hint[sizeof(g_fwUpdateCache.hint) - 1] = '\0';
  g_fwUpdateCache.valid = true;
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
  g_testsCache.valid = false;
  g_fwUpdateCache.valid = false;
  g_fanSettingsCache.valid = false;
  g_limitsCache.valid = false;
  g_calibrationCache.valid = false;
  g_batterySetupCache.valid = false;
  g_transientContSetupCache.valid = false;
  g_transientListSetupCache.valid = false;
  g_homeCache.valid = false;
  if (viewState.mode == CA && app_calibration_confirmation_active()) {
    return;
  }
  draw_home_if_needed(viewState);
}

void screen_update_home(const UiViewState &viewState) {
  if (viewState.mode == CA && app_calibration_confirmation_active()) {
    return;
  }
  draw_home_if_needed(viewState);
  Update_LCD();
}

void screen_render_home(const UiViewState &viewState) { (void)viewState; }

void screen_enter_battery_setup_task(const UiViewState &viewState) {
  (void)viewState;
  g_batterySetupCache.valid = false;
  draw_battery_setup_task_if_needed();
}

void screen_update_battery_setup_task(const UiViewState &viewState) {
  (void)viewState;
  draw_battery_setup_task_if_needed();
}

void screen_render_battery_setup_task(const UiViewState &viewState) { (void)viewState; }

void screen_enter_battery_setup_custom(const UiViewState &viewState) {
  g_batterySetupCache.valid = false;
  draw_battery_setup_custom_if_needed(viewState);
}

void screen_update_battery_setup_custom(const UiViewState &viewState) {
  draw_battery_setup_custom_if_needed(viewState);
}

void screen_render_battery_setup_custom(const UiViewState &viewState) { (void)viewState; }

void screen_enter_battery_setup_cells(const UiViewState &viewState) {
  g_batterySetupCache.valid = false;
  draw_battery_setup_cells_if_needed(viewState);
}

void screen_update_battery_setup_cells(const UiViewState &viewState) {
  draw_battery_setup_cells_if_needed(viewState);
}

void screen_render_battery_setup_cells(const UiViewState &viewState) { (void)viewState; }

void draw_transient_cont_setup_if_needed(const UiViewState &viewState) {
  if (g_transientContSetupCache.valid &&
      g_transientContSetupCache.stage == viewState.transientSetupStage &&
      g_transientContSetupCache.lowCurrentA == viewState.transientLowCurrentA &&
      g_transientContSetupCache.highCurrentA == viewState.transientHighCurrentA &&
      g_transientContSetupCache.periodMs == viewState.transientPeriodMs &&
      std::strcmp(g_transientContSetupCache.inputText, viewState.transientInputText) == 0) {
    return;
  }

  ui_draw_transient_cont_setup_template();

  if (viewState.transientSetupStage > 0) {
    ui_show_value_number(11, 1, viewState.transientLowCurrentA, 'A', 3);
  } else {
    Print_Spaces(10, 1, 10);
  }

  if (viewState.transientSetupStage > 1) {
    ui_show_value_number(11, 2, viewState.transientHighCurrentA, 'A', 3);
  } else {
    Print_Spaces(10, 2, 10);
  }

  if (viewState.transientSetupStage == 0) {
    ui_prepare_value_input_prompt(11, 1, 5);
    printLCD_S(11, 1, String(viewState.transientInputText));
    Print_Spaces(10, 2, 10);
    Print_Spaces(10, 3, 6);
  } else if (viewState.transientSetupStage == 1) {
    ui_prepare_value_input_prompt(11, 2, 5);
    printLCD_S(11, 2, String(viewState.transientInputText));
    Print_Spaces(10, 3, 6);
  } else {
    ui_prepare_value_input_prompt(11, 3, 5);
    printLCD_S(11, 3, String(viewState.transientInputText));
  }

  g_transientContSetupCache.stage = viewState.transientSetupStage;
  g_transientContSetupCache.lowCurrentA = viewState.transientLowCurrentA;
  g_transientContSetupCache.highCurrentA = viewState.transientHighCurrentA;
  g_transientContSetupCache.periodMs = viewState.transientPeriodMs;
  std::strncpy(g_transientContSetupCache.inputText, viewState.transientInputText, sizeof(g_transientContSetupCache.inputText) - 1);
  g_transientContSetupCache.inputText[sizeof(g_transientContSetupCache.inputText) - 1] = '\0';
  g_transientContSetupCache.valid = true;
}

void screen_enter_transient_cont_setup(const UiViewState &viewState) {
  g_transientContSetupCache.valid = false;
  draw_transient_cont_setup_if_needed(viewState);
}

void screen_update_transient_cont_setup(const UiViewState &viewState) {
  draw_transient_cont_setup_if_needed(viewState);
}

void screen_render_transient_cont_setup(const UiViewState &viewState) { (void)viewState; }

void draw_transient_list_setup_if_needed(const UiViewState &viewState) {
  if (g_transientListSetupCache.valid &&
      g_transientListSetupCache.stage == viewState.transientListSetupStage &&
      g_transientListSetupCache.stepCount == viewState.transientListDraftStepCount &&
      g_transientListSetupCache.stepIndex == viewState.transientListDraftStepIndex &&
      g_transientListSetupCache.field == viewState.transientListDraftField &&
      g_transientListSetupCache.currentA == viewState.transientListCurrentA &&
      g_transientListSetupCache.periodMs == viewState.transientListCurrentPeriodMs &&
      std::strcmp(g_transientListSetupCache.inputText, viewState.transientListInputText) == 0) {
    return;
  }

  if (viewState.transientListSetupStage == 0) {
    ui_draw_transient_list_setup_template();
    ui_prepare_value_input_prompt(9, 2, 2);
    printLCD_S(9, 2, String(viewState.transientListInputText));
  } else {
    ui_draw_transient_list_step_template(viewState.transientListDraftStepIndex);
    if (viewState.transientListDraftField == 0) {
      ui_prepare_value_input_prompt(13, 2, 5);
      printLCD_S(13, 2, String(viewState.transientListInputText));
      if (viewState.transientListCurrentPeriodMs > 0.0f) {
        Print_Spaces(12, 3, 6);
        printLCD_S(13, 3, String(static_cast<unsigned long>(viewState.transientListCurrentPeriodMs)));
      } else {
        Print_Spaces(12, 3, 6);
      }
    } else {
      ui_show_value_number(13, 2, viewState.transientListCurrentA, 'A', 3);
      ui_prepare_value_input_prompt(13, 3, 5);
      printLCD_S(13, 3, String(viewState.transientListInputText));
    }
  }

  g_transientListSetupCache.stage = viewState.transientListSetupStage;
  g_transientListSetupCache.stepCount = viewState.transientListDraftStepCount;
  g_transientListSetupCache.stepIndex = viewState.transientListDraftStepIndex;
  g_transientListSetupCache.field = viewState.transientListDraftField;
  g_transientListSetupCache.currentA = viewState.transientListCurrentA;
  g_transientListSetupCache.periodMs = viewState.transientListCurrentPeriodMs;
  std::strncpy(g_transientListSetupCache.inputText, viewState.transientListInputText, sizeof(g_transientListSetupCache.inputText) - 1);
  g_transientListSetupCache.inputText[sizeof(g_transientListSetupCache.inputText) - 1] = '\0';
  g_transientListSetupCache.valid = true;
}

void screen_enter_transient_list_setup(const UiViewState &viewState) {
  g_transientListSetupCache.valid = false;
  draw_transient_list_setup_if_needed(viewState);
}

void screen_update_transient_list_setup(const UiViewState &viewState) {
  draw_transient_list_setup_if_needed(viewState);
}

void screen_render_transient_list_setup(const UiViewState &viewState) { (void)viewState; }

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

void screen_enter_menu_tests(const UiViewState &viewState) {
  g_testsCache.valid = false;
  draw_tests_if_needed(viewState);
}

void screen_update_menu_tests(const UiViewState &viewState) {
  draw_tests_if_needed(viewState);
}

void screen_render_menu_tests(const UiViewState &viewState) { (void)viewState; }

void screen_enter_menu_fw_update(const UiViewState &viewState) {
  (void)viewState;
  g_fwUpdateCache.valid = false;
  app_ota_begin();
  draw_fw_update_if_needed();
}

void screen_update_menu_fw_update(const UiViewState &viewState) {
  (void)viewState;
  draw_fw_update_if_needed();
}

void screen_render_menu_fw_update(const UiViewState &viewState) { (void)viewState; }

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
    case UiScreen::BatterySetupTask: screen_enter_battery_setup_task(viewState); break;
    case UiScreen::BatterySetupCustomCutoff: screen_enter_battery_setup_custom(viewState); break;
    case UiScreen::BatterySetupCellCount: screen_enter_battery_setup_cells(viewState); break;
    case UiScreen::TransientContSetupLow:
    case UiScreen::TransientContSetupHigh:
    case UiScreen::TransientContSetupPeriod: screen_enter_transient_cont_setup(viewState); break;
    case UiScreen::TransientListSetupCount:
    case UiScreen::TransientListSetupStep: screen_enter_transient_list_setup(viewState); break;
    case UiScreen::MenuRoot: screen_enter_menu_root(viewState); break;
    case UiScreen::MenuProtection: screen_enter_menu_protection(viewState); break;
    case UiScreen::MenuTests: screen_enter_menu_tests(viewState); break;
    case UiScreen::MenuFwUpdate: screen_enter_menu_fw_update(viewState); break;
    case UiScreen::MenuFanSettings: screen_enter_menu_fan_settings(viewState); break;
    case UiScreen::MenuLimits: screen_enter_menu_limits(viewState); break;
    case UiScreen::MenuCalibration: screen_enter_menu_calibration(viewState); break;
    default: break;
  }
}

void run_screen_update(UiScreen screen, const UiViewState &viewState) {
  switch (screen) {
    case UiScreen::Home: screen_update_home(viewState); break;
    case UiScreen::BatterySetupTask: screen_update_battery_setup_task(viewState); break;
    case UiScreen::BatterySetupCustomCutoff: screen_update_battery_setup_custom(viewState); break;
    case UiScreen::BatterySetupCellCount: screen_update_battery_setup_cells(viewState); break;
    case UiScreen::TransientContSetupLow:
    case UiScreen::TransientContSetupHigh:
    case UiScreen::TransientContSetupPeriod: screen_update_transient_cont_setup(viewState); break;
    case UiScreen::TransientListSetupCount:
    case UiScreen::TransientListSetupStep: screen_update_transient_list_setup(viewState); break;
    case UiScreen::MenuRoot: screen_update_menu_root(viewState); break;
    case UiScreen::MenuProtection: screen_update_menu_protection(viewState); break;
    case UiScreen::MenuTests: screen_update_menu_tests(viewState); break;
    case UiScreen::MenuFwUpdate: screen_update_menu_fw_update(viewState); break;
    case UiScreen::MenuFanSettings: screen_update_menu_fan_settings(viewState); break;
    case UiScreen::MenuLimits: screen_update_menu_limits(viewState); break;
    case UiScreen::MenuCalibration: screen_update_menu_calibration(viewState); break;
    default: break;
  }
}

void run_screen_render(UiScreen screen, const UiViewState &viewState) {
  switch (screen) {
    case UiScreen::Home: screen_render_home(viewState); break;
    case UiScreen::BatterySetupTask: screen_render_battery_setup_task(viewState); break;
    case UiScreen::BatterySetupCustomCutoff: screen_render_battery_setup_custom(viewState); break;
    case UiScreen::BatterySetupCellCount: screen_render_battery_setup_cells(viewState); break;
    case UiScreen::TransientContSetupLow:
    case UiScreen::TransientContSetupHigh:
    case UiScreen::TransientContSetupPeriod: screen_render_transient_cont_setup(viewState); break;
    case UiScreen::TransientListSetupCount:
    case UiScreen::TransientListSetupStep: screen_render_transient_list_setup(viewState); break;
    case UiScreen::MenuRoot: screen_render_menu_root(viewState); break;
    case UiScreen::MenuProtection: screen_render_menu_protection(viewState); break;
    case UiScreen::MenuTests: screen_render_menu_tests(viewState); break;
    case UiScreen::MenuFwUpdate: screen_render_menu_fw_update(viewState); break;
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
  g_testsCache.valid = false;
  g_fwUpdateCache.valid = false;
  g_fanSettingsCache.valid = false;
  g_limitsCache.valid = false;
  g_calibrationCache.valid = false;
  g_batterySetupCache.valid = false;
  g_transientContSetupCache.valid = false;
  g_transientListSetupCache.valid = false;
  g_homeCache.valid = false;
}

void ui_state_machine_tick(UiScreen targetScreen, const UiViewState &viewState) {
  if (targetScreen != g_currentScreen) {
    if (g_currentScreen == UiScreen::MenuFwUpdate && targetScreen != UiScreen::MenuFwUpdate) {
      app_ota_stop();
      g_fwUpdateCache.valid = false;
    }
    g_currentScreen = targetScreen;
    run_screen_enter(g_currentScreen, viewState);
  }

  run_screen_update(g_currentScreen, viewState);
  run_screen_render(g_currentScreen, viewState);
}

UiScreen ui_state_machine_current_screen() {
  return g_currentScreen;
}






