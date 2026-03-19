#include "ui_state_machine.h"

#include <Arduino.h>
#include <cstring>

#include "../app/app_msc.h"
#include "../app/app_ota.h"
#include "../ui_display.h"
#include "../app/app_calibration_context.h"
#include "../config/system_constants.h"

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

struct ProtectionRenderCache {
  uint8_t option;
  bool valid;
};

struct UpdateRenderCache {
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

struct ClockRenderCache {
  uint8_t option;
  uint8_t rtcDay;
  uint8_t rtcMonth;
  uint8_t rtcYear;
  uint8_t rtcHour;
  uint8_t rtcMinute;
  uint8_t day;
  uint8_t month;
  uint8_t year;
  uint8_t hour;
  uint8_t minute;
  bool editActive;
  char inputText[3];
  bool valid;
};

struct ConfigRtcRenderCache {
  uint8_t day;
  uint8_t month;
  uint8_t year;
  uint8_t hour;
  uint8_t minute;
  bool shiftActive;
  bool valid;
};

struct BatterySetupRenderCache {
  uint8_t stage;
  char batteryType[8];
  char inputText[8];
  bool shiftActive;
  bool valid;
};

struct TransientContSetupRenderCache {
  uint8_t stage;
  float lowCurrentA;
  float highCurrentA;
  float periodMs;
  char inputText[8];
  bool shiftActive;
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
  bool shiftActive;
  bool valid;
};

ProtectionRenderCache g_protectionCache = {0, false};
UpdateRenderCache g_updateCache = {0, false, false};
FwUpdateRenderCache g_fwUpdateCache = {{'\0'}, {'\0'}, {'\0'}, false};
FanSettingsRenderCache g_fanSettingsCache = {0, 0.0f, 0.0f, false, false, {'\0'}, false};
LimitsRenderCache g_limitsCache = {0.0f, 0.0f, 0.0f, 0, false, {'\0'}, false};
CalibrationRenderCache g_calibrationCache = {1, false};
ClockRenderCache g_clockCache = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, false, {'\0'}, false};
ConfigRtcRenderCache g_configRtcCache = {0, 0, 0, 0, 0, false, false};
BatterySetupRenderCache g_batterySetupCache = {0, {'\0'}, {'\0'}, false, false};
TransientContSetupRenderCache g_transientContSetupCache = {0, 0.0f, 0.0f, 0.0f, {'\0'}, false, false};
TransientListSetupRenderCache g_transientListSetupCache = {0, 0, 0, 0, 0.0f, 0.0f, {'\0'}, false, false};

void draw_menu_root_if_needed(const UiViewState &viewState) {
  if (g_lastMenuRootSelection == viewState.menuRootSelection) return;
  uiDisplayRenderConfigRootMenu(viewState);
  g_lastMenuRootSelection = viewState.menuRootSelection;
}

void draw_battery_setup_task_if_needed(const UiViewState &viewState) {
  const bool shiftActive = app_msc_shift_active();
  if (g_batterySetupCache.valid && g_batterySetupCache.stage == 0 &&
      g_batterySetupCache.shiftActive == shiftActive) {
    return;
  }

  uiDisplayRenderBatterySetupTask(viewState);
  g_batterySetupCache.stage = 0;
  g_batterySetupCache.shiftActive = shiftActive;
  g_batterySetupCache.valid = true;
}

void draw_battery_setup_custom_if_needed(const UiViewState &viewState) {
  const bool shiftActive = app_msc_shift_active();
  const bool sameStage = g_batterySetupCache.valid && g_batterySetupCache.stage == 1;
  const bool sameType = std::strcmp(g_batterySetupCache.batteryType, viewState.batteryType) == 0;
  const bool sameInput = std::strcmp(g_batterySetupCache.inputText, viewState.batteryInputText) == 0;
  const bool sameShift = g_batterySetupCache.shiftActive == shiftActive;

  if (sameStage && sameType && sameInput && sameShift) {
    return;
  }

  if (!sameStage || !sameType) {
    uiDisplayRenderBatterySetupCustom(viewState);
  } else {
    uiDisplayUpdateBatterySetupCustomValue(viewState);
  }
  std::strncpy(g_batterySetupCache.batteryType, viewState.batteryType, sizeof(g_batterySetupCache.batteryType) - 1);
  g_batterySetupCache.batteryType[sizeof(g_batterySetupCache.batteryType) - 1] = '\0';
  std::strncpy(g_batterySetupCache.inputText, viewState.batteryInputText, sizeof(g_batterySetupCache.inputText) - 1);
  g_batterySetupCache.inputText[sizeof(g_batterySetupCache.inputText) - 1] = '\0';
  g_batterySetupCache.stage = 1;
  g_batterySetupCache.shiftActive = shiftActive;
  g_batterySetupCache.valid = true;
}

void draw_battery_setup_cells_if_needed(const UiViewState &viewState) {
  const bool shiftActive = app_msc_shift_active();
  const bool sameStage = g_batterySetupCache.valid && g_batterySetupCache.stage == 2;
  const bool sameType = std::strcmp(g_batterySetupCache.batteryType, viewState.batteryType) == 0;
  const bool sameInput = std::strcmp(g_batterySetupCache.inputText, viewState.batteryInputText) == 0;
  const bool sameShift = g_batterySetupCache.shiftActive == shiftActive;

  if (sameStage && sameType && sameInput && sameShift) {
    return;
  }

  if (!sameStage || !sameType) {
    uiDisplayRenderBatterySetupCells(viewState);
  } else {
    uiDisplayUpdateBatterySetupCellsValue(viewState);
  }
  std::strncpy(g_batterySetupCache.batteryType, viewState.batteryType, sizeof(g_batterySetupCache.batteryType) - 1);
  g_batterySetupCache.batteryType[sizeof(g_batterySetupCache.batteryType) - 1] = '\0';
  std::strncpy(g_batterySetupCache.inputText, viewState.batteryInputText, sizeof(g_batterySetupCache.inputText) - 1);
  g_batterySetupCache.inputText[sizeof(g_batterySetupCache.inputText) - 1] = '\0';
  g_batterySetupCache.stage = 2;
  g_batterySetupCache.shiftActive = shiftActive;
  g_batterySetupCache.valid = true;
}

void draw_protection_if_needed(const UiViewState &viewState) {
  if (g_protectionCache.valid && g_protectionCache.option == viewState.protectionMenuSelection) {
    return;
  }

  uiDisplayRenderProtectionMenu(viewState);
  g_protectionCache.option = viewState.protectionMenuSelection;
  g_protectionCache.valid = true;
}

void draw_update_if_needed(const UiViewState &viewState) {
  if (g_updateCache.valid &&
      g_updateCache.option == viewState.updateMenuSelection &&
      g_updateCache.fanOn == viewState.fanManualStateOn) {
    return;
  }

  uiDisplayRenderUpdateMenu(viewState);
  g_updateCache.option = viewState.updateMenuSelection;
  g_updateCache.fanOn = viewState.fanManualStateOn;
  g_updateCache.valid = true;
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

  uiDisplayRenderFanSettingsMenu(viewState);
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

  uiDisplayRenderFwUpdateScreen(status, detail, hint);
  std::strncpy(g_fwUpdateCache.status, status, sizeof(g_fwUpdateCache.status) - 1);
  g_fwUpdateCache.status[sizeof(g_fwUpdateCache.status) - 1] = '\0';
  std::strncpy(g_fwUpdateCache.detail, detail, sizeof(g_fwUpdateCache.detail) - 1);
  g_fwUpdateCache.detail[sizeof(g_fwUpdateCache.detail) - 1] = '\0';
  std::strncpy(g_fwUpdateCache.hint, hint, sizeof(g_fwUpdateCache.hint) - 1);
  g_fwUpdateCache.hint[sizeof(g_fwUpdateCache.hint) - 1] = '\0';
  g_fwUpdateCache.valid = true;
}

void draw_limits_menu(const UiViewState &viewState) { uiDisplayRenderLimitsMenu(viewState); }

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

void draw_calibration_menu(const UiViewState &viewState) { uiDisplayRenderCalibrationMenu(viewState); }

void draw_calibration_if_needed(const UiViewState &viewState) {
  if (g_calibrationCache.valid && g_calibrationCache.option == viewState.calibrationMenuOption) {
    return;
  }

  draw_calibration_menu(viewState);
  g_calibrationCache.option = viewState.calibrationMenuOption;
  g_calibrationCache.valid = true;
}

void draw_clock_if_needed(const UiViewState &viewState) {
  if (g_clockCache.valid &&
      g_clockCache.option == viewState.clockMenuSelection &&
      g_clockCache.rtcDay == viewState.rtcDay &&
      g_clockCache.rtcMonth == viewState.rtcMonth &&
      g_clockCache.rtcYear == viewState.rtcYear &&
      g_clockCache.rtcHour == viewState.rtcHour &&
      g_clockCache.rtcMinute == viewState.rtcMinute &&
      g_clockCache.day == viewState.clockDraftDay &&
      g_clockCache.month == viewState.clockDraftMonth &&
      g_clockCache.year == viewState.clockDraftYear &&
      g_clockCache.hour == viewState.clockDraftHour &&
      g_clockCache.minute == viewState.clockDraftMinute &&
      g_clockCache.editActive == viewState.clockEditActive &&
      std::strcmp(g_clockCache.inputText, viewState.clockInputText) == 0) {
    return;
  }

  uiDisplayRenderClockMenu(viewState);
  g_clockCache.option = viewState.clockMenuSelection;
  g_clockCache.rtcDay = viewState.rtcDay;
  g_clockCache.rtcMonth = viewState.rtcMonth;
  g_clockCache.rtcYear = viewState.rtcYear;
  g_clockCache.rtcHour = viewState.rtcHour;
  g_clockCache.rtcMinute = viewState.rtcMinute;
  g_clockCache.day = viewState.clockDraftDay;
  g_clockCache.month = viewState.clockDraftMonth;
  g_clockCache.year = viewState.clockDraftYear;
  g_clockCache.hour = viewState.clockDraftHour;
  g_clockCache.minute = viewState.clockDraftMinute;
  g_clockCache.editActive = viewState.clockEditActive;
  std::strncpy(g_clockCache.inputText, viewState.clockInputText, sizeof(g_clockCache.inputText) - 1);
  g_clockCache.inputText[sizeof(g_clockCache.inputText) - 1] = '\0';
  g_clockCache.valid = true;
}

bool invalidate_active_config_screen_if_rtc_changed(UiScreen screen, const UiViewState &viewState) {
  const bool shiftActive = app_msc_shift_active();
  const bool sameRtc = g_configRtcCache.valid &&
                       g_configRtcCache.day == viewState.rtcDay &&
                       g_configRtcCache.month == viewState.rtcMonth &&
                       g_configRtcCache.year == viewState.rtcYear &&
                       g_configRtcCache.hour == viewState.rtcHour &&
                       g_configRtcCache.minute == viewState.rtcMinute &&
                       g_configRtcCache.shiftActive == shiftActive;

  if (!sameRtc) {
    switch (screen) {
      case UiScreen::MenuRoot:
        g_lastMenuRootSelection = 0xFF;
        break;
      case UiScreen::MenuProtection:
        g_protectionCache.valid = false;
        break;
      case UiScreen::MenuUpdate:
        g_updateCache.valid = false;
        break;
      case UiScreen::MenuFwUpdate:
        g_fwUpdateCache.valid = false;
        break;
      case UiScreen::MenuFanSettings:
        g_fanSettingsCache.valid = false;
        break;
      case UiScreen::MenuLimits:
        g_limitsCache.valid = false;
        break;
      case UiScreen::MenuCalibration:
        g_calibrationCache.valid = false;
        break;
      case UiScreen::MenuClock:
        g_clockCache.valid = false;
        break;
      default:
        break;
    }

    g_configRtcCache.day = viewState.rtcDay;
    g_configRtcCache.month = viewState.rtcMonth;
    g_configRtcCache.year = viewState.rtcYear;
    g_configRtcCache.hour = viewState.rtcHour;
    g_configRtcCache.minute = viewState.rtcMinute;
    g_configRtcCache.shiftActive = shiftActive;
    g_configRtcCache.valid = true;
    return true;
  }
  return false;
}

void screen_enter_home(const UiViewState &viewState) {
  g_lastMenuRootSelection = 0xFF;
  g_protectionCache.valid = false;
  g_updateCache.valid = false;
  g_fwUpdateCache.valid = false;
  g_fanSettingsCache.valid = false;
  g_limitsCache.valid = false;
  g_calibrationCache.valid = false;
  g_clockCache.valid = false;
  g_batterySetupCache.valid = false;
  g_transientContSetupCache.valid = false;
  g_transientListSetupCache.valid = false;
  if (viewState.mode == CA && app_calibration_confirmation_active()) {
    return;
  }
}

void screen_update_home(const UiViewState &viewState) {
  if (viewState.mode == CA && app_calibration_confirmation_active()) {
    return;
  }
  uiDisplayUpdate();
}

void screen_render_home(const UiViewState &viewState) { (void)viewState; }

void screen_enter_battery_setup_task(const UiViewState &viewState) {
  g_batterySetupCache.valid = false;
  draw_battery_setup_task_if_needed(viewState);
}

void screen_update_battery_setup_task(const UiViewState &viewState) {
  draw_battery_setup_task_if_needed(viewState);
  uiDisplayUpdateSetupMetrics(viewState);
}

void screen_render_battery_setup_task(const UiViewState &viewState) { (void)viewState; }

void screen_enter_battery_setup_custom(const UiViewState &viewState) {
  g_batterySetupCache.valid = false;
  draw_battery_setup_custom_if_needed(viewState);
}

void screen_update_battery_setup_custom(const UiViewState &viewState) {
  draw_battery_setup_custom_if_needed(viewState);
  uiDisplayUpdateSetupMetrics(viewState);
}

void screen_render_battery_setup_custom(const UiViewState &viewState) { (void)viewState; }

void screen_enter_battery_setup_cells(const UiViewState &viewState) {
  g_batterySetupCache.valid = false;
  draw_battery_setup_cells_if_needed(viewState);
}

void screen_update_battery_setup_cells(const UiViewState &viewState) {
  draw_battery_setup_cells_if_needed(viewState);
  uiDisplayUpdateSetupMetrics(viewState);
}

void screen_render_battery_setup_cells(const UiViewState &viewState) { (void)viewState; }

void draw_transient_cont_setup_if_needed(const UiViewState &viewState) {
  const bool shiftActive = app_msc_shift_active();
  const bool sameStage = g_transientContSetupCache.valid &&
                         g_transientContSetupCache.stage == viewState.transientSetupStage;
  const bool sameValues = g_transientContSetupCache.lowCurrentA == viewState.transientLowCurrentA &&
                          g_transientContSetupCache.highCurrentA == viewState.transientHighCurrentA &&
                          g_transientContSetupCache.periodMs == viewState.transientPeriodMs;
  const bool sameInput = std::strcmp(g_transientContSetupCache.inputText, viewState.transientInputText) == 0;
  const bool sameShift = g_transientContSetupCache.shiftActive == shiftActive;

  if (sameStage && sameValues && sameInput && sameShift) {
    return;
  }

  if (!sameStage || !sameValues) {
    uiDisplayRenderTransientContSetup(viewState);
  } else {
    uiDisplayUpdateTransientContSetupValue(viewState);
  }

  g_transientContSetupCache.stage = viewState.transientSetupStage;
  g_transientContSetupCache.lowCurrentA = viewState.transientLowCurrentA;
  g_transientContSetupCache.highCurrentA = viewState.transientHighCurrentA;
  g_transientContSetupCache.periodMs = viewState.transientPeriodMs;
  std::strncpy(g_transientContSetupCache.inputText, viewState.transientInputText, sizeof(g_transientContSetupCache.inputText) - 1);
  g_transientContSetupCache.inputText[sizeof(g_transientContSetupCache.inputText) - 1] = '\0';
  g_transientContSetupCache.shiftActive = shiftActive;
  g_transientContSetupCache.valid = true;
}

void screen_enter_transient_cont_setup(const UiViewState &viewState) {
  g_transientContSetupCache.valid = false;
  draw_transient_cont_setup_if_needed(viewState);
}

void screen_update_transient_cont_setup(const UiViewState &viewState) {
  draw_transient_cont_setup_if_needed(viewState);
  uiDisplayUpdateSetupMetrics(viewState);
}

void screen_render_transient_cont_setup(const UiViewState &viewState) { (void)viewState; }

void draw_transient_list_setup_if_needed(const UiViewState &viewState) {
  const bool shiftActive = app_msc_shift_active();
  const bool sameStage = g_transientListSetupCache.valid &&
                         g_transientListSetupCache.stage == viewState.transientListSetupStage;
  const bool sameValues = g_transientListSetupCache.stepCount == viewState.transientListDraftStepCount &&
                          g_transientListSetupCache.stepIndex == viewState.transientListDraftStepIndex &&
                          g_transientListSetupCache.field == viewState.transientListDraftField &&
                          g_transientListSetupCache.currentA == viewState.transientListCurrentA &&
                          g_transientListSetupCache.periodMs == viewState.transientListCurrentPeriodMs;
  const bool sameInput = std::strcmp(g_transientListSetupCache.inputText, viewState.transientListInputText) == 0;
  const bool sameShift = g_transientListSetupCache.shiftActive == shiftActive;

  if (sameStage && sameValues && sameInput && sameShift) {
    return;
  }

  if (!sameStage || !sameValues) {
    uiDisplayRenderTransientListSetup(viewState);
  } else {
    uiDisplayUpdateTransientListSetupValue(viewState);
  }

  g_transientListSetupCache.stage = viewState.transientListSetupStage;
  g_transientListSetupCache.stepCount = viewState.transientListDraftStepCount;
  g_transientListSetupCache.stepIndex = viewState.transientListDraftStepIndex;
  g_transientListSetupCache.field = viewState.transientListDraftField;
  g_transientListSetupCache.currentA = viewState.transientListCurrentA;
  g_transientListSetupCache.periodMs = viewState.transientListCurrentPeriodMs;
  std::strncpy(g_transientListSetupCache.inputText, viewState.transientListInputText, sizeof(g_transientListSetupCache.inputText) - 1);
  g_transientListSetupCache.inputText[sizeof(g_transientListSetupCache.inputText) - 1] = '\0';
  g_transientListSetupCache.shiftActive = shiftActive;
  g_transientListSetupCache.valid = true;
}

void screen_enter_transient_list_setup(const UiViewState &viewState) {
  g_transientListSetupCache.valid = false;
  draw_transient_list_setup_if_needed(viewState);
}

void screen_update_transient_list_setup(const UiViewState &viewState) {
  draw_transient_list_setup_if_needed(viewState);
  uiDisplayUpdateSetupMetrics(viewState);
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

void screen_enter_menu_update(const UiViewState &viewState) {
  g_updateCache.valid = false;
  draw_update_if_needed(viewState);
}

void screen_update_menu_update(const UiViewState &viewState) {
  draw_update_if_needed(viewState);
}

void screen_render_menu_update(const UiViewState &viewState) { (void)viewState; }

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

void screen_enter_menu_clock(const UiViewState &viewState) {
  g_clockCache.valid = false;
  draw_clock_if_needed(viewState);
}

void screen_update_menu_clock(const UiViewState &viewState) {
  draw_clock_if_needed(viewState);
}

void screen_render_menu_clock(const UiViewState &viewState) { (void)viewState; }

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
    case UiScreen::MenuUpdate: screen_enter_menu_update(viewState); break;
    case UiScreen::MenuFwUpdate: screen_enter_menu_fw_update(viewState); break;
    case UiScreen::MenuFanSettings: screen_enter_menu_fan_settings(viewState); break;
    case UiScreen::MenuLimits: screen_enter_menu_limits(viewState); break;
    case UiScreen::MenuCalibration: screen_enter_menu_calibration(viewState); break;
    case UiScreen::MenuClock: screen_enter_menu_clock(viewState); break;
    default: break;
  }
}

void run_screen_update(UiScreen screen, const UiViewState &viewState) {
  const bool configChromeChanged = invalidate_active_config_screen_if_rtc_changed(screen, viewState);
  if (configChromeChanged) {
    switch (screen) {
      case UiScreen::MenuRoot:
      case UiScreen::MenuProtection:
      case UiScreen::MenuUpdate:
      case UiScreen::MenuFwUpdate:
      case UiScreen::MenuFanSettings:
      case UiScreen::MenuLimits:
      case UiScreen::MenuCalibration:
      case UiScreen::MenuClock:
        uiDisplayUpdateConfigChrome();
        break;
      default:
        break;
    }
  }
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
    case UiScreen::MenuUpdate: screen_update_menu_update(viewState); break;
    case UiScreen::MenuFwUpdate: screen_update_menu_fw_update(viewState); break;
    case UiScreen::MenuFanSettings: screen_update_menu_fan_settings(viewState); break;
    case UiScreen::MenuLimits: screen_update_menu_limits(viewState); break;
    case UiScreen::MenuCalibration: screen_update_menu_calibration(viewState); break;
    case UiScreen::MenuClock: screen_update_menu_clock(viewState); break;
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
    case UiScreen::MenuUpdate: screen_render_menu_update(viewState); break;
    case UiScreen::MenuFwUpdate: screen_render_menu_fw_update(viewState); break;
    case UiScreen::MenuFanSettings: screen_render_menu_fan_settings(viewState); break;
    case UiScreen::MenuLimits: screen_render_menu_limits(viewState); break;
    case UiScreen::MenuCalibration: screen_render_menu_calibration(viewState); break;
    case UiScreen::MenuClock: screen_render_menu_clock(viewState); break;
    default: break;
  }
}
}

void ui_state_machine_reset() {
  uiDisplayInvalidateHomeLayout();
  g_currentScreen = UiScreen::Home;
  g_lastMenuRootSelection = 0xFF;
  g_protectionCache.valid = false;
  g_updateCache.valid = false;
  g_fwUpdateCache.valid = false;
  g_fanSettingsCache.valid = false;
  g_limitsCache.valid = false;
  g_calibrationCache.valid = false;
  g_clockCache.valid = false;
  g_configRtcCache.valid = false;
  g_batterySetupCache.valid = false;
  g_transientContSetupCache.valid = false;
  g_transientListSetupCache.valid = false;
}

void ui_state_machine_tick(UiScreen targetScreen, const UiViewState &viewState) {
  if (targetScreen != g_currentScreen) {
    uiDisplayInvalidateHomeLayout();
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






