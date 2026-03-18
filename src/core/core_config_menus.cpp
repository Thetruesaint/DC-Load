#include "core_config_menus.h"

#include <Arduino.h>
#include <cstdlib>

#include "../config/system_constants.h"
#include "core_setup_flows.h"
#include "core_state_rules.h"

namespace {
ConfigMenu menu_root_menu_for_index(uint8_t index) {
  switch (index) {
    case 0: return ConfigMenu::Protection;
    case 1: return ConfigMenu::Calibration;
    case 2: return ConfigMenu::Update;
    case 3: return ConfigMenu::Clock;
    default: return ConfigMenu::None;
  }
}

uint8_t menu_root_index_for_menu(ConfigMenu menu) {
  if (menu == ConfigMenu::Calibration) return 1;
  if (menu == ConfigMenu::Update) return 2;
  if (menu == ConfigMenu::Clock) return 3;
  if (menu == ConfigMenu::None) return 4;
  return 0;
}

ConfigMenu protection_menu_for_index(uint8_t index) {
  switch (index) {
    case 0: return ConfigMenu::Limits;
    case 1: return ConfigMenu::FanSettings;
    default: return ConfigMenu::None;
  }
}

uint8_t config_menu_item_count(ConfigMenu menu) {
  switch (menu) {
    case ConfigMenu::Root: return 5;
    case ConfigMenu::Clock: return 7;
    case ConfigMenu::Protection: return 3;
    case ConfigMenu::Update: return 2;
    case ConfigMenu::FanSettings: return 4;
    case ConfigMenu::Calibration: return 5;
    default: return 0;
  }
}

uint8_t config_menu_selection_index(const SystemState &state, ConfigMenu menu) {
  if (menu == ConfigMenu::Root) return state.menuRootSelection;
  if (menu == ConfigMenu::Protection) return state.protectionMenuSelection;
  if (menu == ConfigMenu::Update) return state.updateMenuSelection;
  if (menu == ConfigMenu::FanSettings) return state.fanSettingsMenuSelection;
  if (menu == ConfigMenu::Clock) return state.clockMenuSelection;
  if (menu == ConfigMenu::Calibration) {
    return (state.calibrationMenuOption > 0) ? static_cast<uint8_t>(state.calibrationMenuOption - 1) : 0;
  }
  return 0;
}

void config_menu_set_selection_index(SystemState *state, ConfigMenu menu, uint8_t index) {
  if (state == nullptr) return;

  if (menu == ConfigMenu::Root) {
    state->menuRootSelection = index;
  } else if (menu == ConfigMenu::Protection) {
    state->protectionMenuSelection = index;
  } else if (menu == ConfigMenu::Update) {
    state->updateMenuSelection = index;
  } else if (menu == ConfigMenu::FanSettings) {
    state->fanSettingsMenuSelection = index;
  } else if (menu == ConfigMenu::Clock) {
    state->clockMenuSelection = index;
  } else if (menu == ConfigMenu::Calibration) {
    state->calibrationMenuOption = static_cast<uint8_t>(index + 1);
  }
}

ConfigMenu config_menu_selected_target(const SystemState &state, ConfigMenu menu) {
  if (menu == ConfigMenu::Root) {
    return menu_root_menu_for_index(state.menuRootSelection);
  }
  if (menu == ConfigMenu::Protection) {
    return protection_menu_for_index(state.protectionMenuSelection);
  }
  if (menu == ConfigMenu::Update) {
    if (state.updateMenuSelection == 0) return ConfigMenu::FwUpdate;
    return ConfigMenu::None;
  }
  return ConfigMenu::None;
}

void config_menu_enter(SystemState *state, ConfigMenu menu, ConfigMenu parent) {
  if (state == nullptr) return;
  state->currentConfigMenu = menu;
  state->parentConfigMenu = parent;
}

void config_menu_leave_to_parent(SystemState *state, ConfigMenu parent, ConfigMenu child) {
  if (state == nullptr) return;

  state->currentConfigMenu = parent;
  state->parentConfigMenu = (parent == ConfigMenu::Protection) ? ConfigMenu::Root : ConfigMenu::None;
  if (core_should_reset_mode_init(*state)) state->modeInitialized = false;

  if (parent == ConfigMenu::Root) {
    state->menuRootSelection = menu_root_index_for_menu(child);
  } else if (parent == ConfigMenu::Protection) {
    state->protectionMenuSelection = (child == ConfigMenu::FanSettings) ? 1 : (child == ConfigMenu::Update) ? 2 : 0;
  } else if (parent == ConfigMenu::Update) {
    state->updateMenuSelection = (child == ConfigMenu::FwUpdate) ? 1 : 0;
  }
}

void config_menu_close(SystemState *state) {
  if (state == nullptr) return;
  state->currentConfigMenu = ConfigMenu::None;
  state->parentConfigMenu = ConfigMenu::None;
  if (core_should_reset_mode_init(*state)) state->modeInitialized = false;
}

void limits_input_reset(SystemState *state) {
  if (state == nullptr) return;
  state->limitsInputText[0] = '\0';
  state->limitsInputLength = 0;
  state->limitsInputHasDecimal = false;
}

void fan_input_reset(SystemState *state) {
  if (state == nullptr) return;
  state->fanInputText[0] = '\0';
  state->fanInputLength = 0;
}

void clock_input_reset(SystemState *state) {
  if (state == nullptr) return;
  state->clockInputText[0] = '\0';
  state->clockInputLength = 0;
}

uint8_t days_in_month(uint8_t month, uint8_t year) {
  switch (month) {
    case 4:
    case 6:
    case 9:
    case 11:
      return 30;
    case 2: {
      const uint16_t fullYear = static_cast<uint16_t>(2000 + year);
      const bool leapYear = ((fullYear % 4U) == 0U && (fullYear % 100U) != 0U) || ((fullYear % 400U) == 0U);
      return leapYear ? 29 : 28;
    }
    default:
      return 31;
  }
}

void clock_menu_begin(SystemState *state) {
  if (state == nullptr) return;
  state->currentConfigMenu = ConfigMenu::Clock;
  state->parentConfigMenu = ConfigMenu::Root;
  state->clockMenuSelection = 0;
  state->clockDraftDay = (state->rtcDay > 0) ? state->rtcDay : 1;
  state->clockDraftMonth = (state->rtcMonth > 0) ? state->rtcMonth : 1;
  state->clockDraftYear = state->rtcYear;
  state->clockDraftHour = state->rtcHour;
  state->clockDraftMinute = state->rtcMinute;
  state->clockEditActive = false;
  clock_input_reset(state);
}

void clock_menu_finish(SystemState *state, bool save, bool returnToParent = false) {
  if (state == nullptr) return;
  const ConfigMenu parent = state->parentConfigMenu;
  state->clockSaveEvent = save;
  state->clockEditActive = false;
  clock_input_reset(state);

  if (returnToParent && parent != ConfigMenu::None) {
    config_menu_leave_to_parent(state, parent, ConfigMenu::Clock);
    return;
  }

  state->currentConfigMenu = ConfigMenu::None;
  state->parentConfigMenu = ConfigMenu::None;
  if (core_should_reset_mode_init(*state)) state->modeInitialized = false;
}

void clock_menu_start_edit(SystemState *state) {
  if (state == nullptr) return;
  if (state->clockMenuSelection >= 5) return;
  state->clockEditActive = true;
  clock_input_reset(state);
}

uint8_t clock_field_max_value(const SystemState &state) {
  switch (state.clockMenuSelection) {
    case 0: return days_in_month(state.clockDraftMonth, state.clockDraftYear);
    case 1: return 12;
    case 2: return 99;
    case 3: return 23;
    case 4: return 59;
    default: return 99;
  }
}

uint8_t clock_field_min_value(const SystemState &state) {
  switch (state.clockMenuSelection) {
    case 0:
    case 1:
      return 1;
    default:
      return 0;
  }
}

void clock_menu_cancel_edit_impl(SystemState *state) {
  if (state == nullptr) return;
  state->clockEditActive = false;
  clock_input_reset(state);
}

bool clock_menu_append_digit_impl(SystemState *state, char key) {
  if (state == nullptr) return false;
  return numeric_input_append_digit(state->clockInputText, &state->clockInputLength, 2, key);
}

bool clock_menu_backspace_impl(SystemState *state) {
  if (state == nullptr) return false;
  return numeric_input_backspace(state->clockInputText, &state->clockInputLength);
}

void clock_menu_commit_edit_impl(SystemState *state) {
  if (state == nullptr) return;
  if (state->clockInputLength == 0) {
    clock_menu_cancel_edit_impl(state);
    return;
  }
  const uint8_t rawValue = static_cast<uint8_t>(atoi(state->clockInputText));
  const uint8_t clampedValue = static_cast<uint8_t>(constrain(rawValue,
                                                              static_cast<int>(clock_field_min_value(*state)),
                                                              static_cast<int>(clock_field_max_value(*state))));

  switch (state->clockMenuSelection) {
    case 0: state->clockDraftDay = clampedValue; break;
    case 1:
      state->clockDraftMonth = clampedValue;
      state->clockDraftDay = min(state->clockDraftDay, days_in_month(state->clockDraftMonth, state->clockDraftYear));
      break;
    case 2:
      state->clockDraftYear = clampedValue;
      state->clockDraftDay = min(state->clockDraftDay, days_in_month(state->clockDraftMonth, state->clockDraftYear));
      break;
    case 3: state->clockDraftHour = clampedValue; break;
    case 4: state->clockDraftMinute = clampedValue; break;
    default: break;
  }

  clock_menu_cancel_edit_impl(state);
}

void clock_menu_adjust_field_impl(SystemState *state, int direction) {
  if (state == nullptr || direction == 0 || state->clockMenuSelection >= 5) return;

  uint8_t *field = nullptr;
  switch (state->clockMenuSelection) {
    case 0: field = &state->clockDraftDay; break;
    case 1: field = &state->clockDraftMonth; break;
    case 2: field = &state->clockDraftYear; break;
    case 3: field = &state->clockDraftHour; break;
    case 4: field = &state->clockDraftMinute; break;
    default: return;
  }

  int next = static_cast<int>(*field) + direction;
  const int minValue = static_cast<int>(clock_field_min_value(*state));
  const int maxValue = static_cast<int>(clock_field_max_value(*state));
  if (next > maxValue) next = minValue;
  if (next < minValue) next = maxValue;
  *field = static_cast<uint8_t>(next);

  if (state->clockMenuSelection == 1 || state->clockMenuSelection == 2) {
    state->clockDraftDay = min(state->clockDraftDay, days_in_month(state->clockDraftMonth, state->clockDraftYear));
  }
}

bool limits_field_allows_decimal(const SystemState &state) {
  return state.limitsMenuField != 2;
}

int limits_field_max_digits(const SystemState &state) {
  return (state.limitsMenuField == 2) ? 2 : 5;
}

void limits_menu_begin(SystemState *state) {
  if (state == nullptr) return;
  state->limitsMenuActive = true;
  state->currentConfigMenu = ConfigMenu::Limits;
  state->parentConfigMenu = ConfigMenu::Protection;
  state->limitsMenuField = 0;
  state->limitsDraftCurrentA = state->currentCutOffA;
  state->limitsDraftPowerW = state->powerCutOffW;
  state->limitsDraftTempC = state->tempCutOffC;
  state->limitsEditActive = false;
  limits_input_reset(state);
}

void fan_menu_begin(SystemState *state) {
  if (state == nullptr) return;
  state->currentConfigMenu = ConfigMenu::FanSettings;
  state->parentConfigMenu = ConfigMenu::Protection;
  state->fanSettingsMenuSelection = 0;
  state->fanDraftTempC = state->fanTempOnC;
  state->fanDraftHoldSeconds = state->fanHoldSeconds;
  state->fanEditActive = false;
  fan_input_reset(state);
}

void fan_menu_finish(SystemState *state, bool save, bool returnToParent = false) {
  if (state == nullptr) return;
  const ConfigMenu parent = state->parentConfigMenu;
  state->fanSaveEvent = save;
  state->fanEditActive = false;
  state->fanManualOverrideActive = false;
  state->fanManualStateOn = false;
  fan_input_reset(state);

  if (returnToParent && parent != ConfigMenu::None) {
    config_menu_leave_to_parent(state, parent, ConfigMenu::FanSettings);
    return;
  }

  state->currentConfigMenu = ConfigMenu::None;
  state->parentConfigMenu = ConfigMenu::None;
  if (core_should_reset_mode_init(*state)) state->modeInitialized = false;
}

void fan_menu_start_edit(SystemState *state) {
  if (state == nullptr) return;
  if (state->fanSettingsMenuSelection >= 2) return;
  state->fanEditActive = true;
  fan_input_reset(state);
}

void update_menu_begin(SystemState *state) {
  if (state == nullptr) return;
  state->currentConfigMenu = ConfigMenu::Update;
  state->parentConfigMenu = ConfigMenu::Root;
  state->updateMenuSelection = 0;
}

void update_menu_finish(SystemState *state, bool returnToParent = false) {
  if (state == nullptr) return;
  const ConfigMenu parent = state->parentConfigMenu;

  if (returnToParent && parent != ConfigMenu::None) {
    config_menu_leave_to_parent(state, parent, ConfigMenu::Update);
    return;
  }

  state->currentConfigMenu = ConfigMenu::None;
  state->parentConfigMenu = ConfigMenu::None;
  if (core_should_reset_mode_init(*state)) state->modeInitialized = false;
}

void calibration_menu_begin(SystemState *state) {
  if (state == nullptr) return;
  state->calibrationMenuActive = true;
  state->currentConfigMenu = ConfigMenu::Calibration;
  state->parentConfigMenu = ConfigMenu::Root;
  if (state->calibrationMenuOption < 1 || state->calibrationMenuOption > 5) {
    state->calibrationMenuOption = 1;
  }
}

void calibration_menu_finish(SystemState *state, bool apply, bool returnToParent = false) {
  if (state == nullptr) return;
  const ConfigMenu parent = state->parentConfigMenu;
  state->calibrationMenuApplyEvent = apply;
  state->calibrationMenuActive = false;

  if (returnToParent && parent != ConfigMenu::None) {
    config_menu_leave_to_parent(state, parent, ConfigMenu::Calibration);
    return;
  }

  state->currentConfigMenu = ConfigMenu::None;
  state->parentConfigMenu = ConfigMenu::None;
  if (core_should_reset_mode_init(*state)) state->modeInitialized = false;
}
}

ConfigMenu decode_config_root_target(int32_t raw) {
  return (raw == 2) ? ConfigMenu::Calibration : ConfigMenu::Protection;
}

void clear_config_runtime_flags(SystemState *state) {
  if (state == nullptr) return;
  state->openLimitsConfigEvent = false;
  state->openCalibrationConfigEvent = false;
  state->calibrationValueConfirmEvent = false;
  state->limitsMenuActive = false;
  state->limitsEditActive = false;
  limits_input_reset(state);
  state->calibrationMenuActive = false;
  state->fanEditActive = false;
  state->fanManualOverrideActive = false;
  state->fanManualStateOn = false;
  fan_input_reset(state);
  state->clockEditActive = false;
  clock_input_reset(state);
}

bool clock_menu_append_digit(SystemState *state, char key) {
  return clock_menu_append_digit_impl(state, key);
}

bool clock_menu_backspace(SystemState *state) {
  return clock_menu_backspace_impl(state);
}

void clock_menu_cancel_edit(SystemState *state) {
  clock_menu_cancel_edit_impl(state);
}

void clock_menu_commit_edit(SystemState *state) {
  clock_menu_commit_edit_impl(state);
}

void clock_menu_adjust_field(SystemState *state, int direction) {
  clock_menu_adjust_field_impl(state, direction);
}

void reset_config_navigation(SystemState *state) {
  if (state == nullptr) return;
  state->menuRootSelection = 0;
  state->protectionMenuSelection = 0;
  state->updateMenuSelection = 0;
  state->fanSettingsMenuSelection = 0;
  state->clockMenuSelection = 0;
  state->currentConfigMenu = ConfigMenu::None;
  state->parentConfigMenu = ConfigMenu::None;
}

void reset_mode_entry_state(SystemState *state, bool preserveBatteryModeValues) {
  if (state == nullptr) return;
  reset_config_navigation(state);
  clear_config_runtime_flags(state);
  battery_setup_begin(state);
  if (!preserveBatteryModeValues) {
    battery_type_set(state, "");
    state->batteryCutoffVolts = 0.0f;
  }
}

void config_menu_step_selection(SystemState *state, ConfigMenu menu, int direction) {
  if (state == nullptr || direction == 0) return;

  const int itemCount = config_menu_item_count(menu);
  if (itemCount == 0) return;

  int next = static_cast<int>(config_menu_selection_index(*state, menu)) + direction;
  if (next < 0) next = itemCount - 1;
  if (next >= itemCount) next = 0;
  config_menu_set_selection_index(state, menu, static_cast<uint8_t>(next));
  state->currentConfigMenu = menu;
}

void config_menu_select_index(SystemState *state, ConfigMenu menu, uint8_t index) {
  if (state == nullptr) return;

  const uint8_t itemCount = config_menu_item_count(menu);
  if (itemCount == 0) return;

  config_menu_set_selection_index(state, menu, (index < itemCount) ? index : 0);
  state->currentConfigMenu = menu;
}

void limits_menu_move_field(SystemState *state, int direction) {
  if (state == nullptr || direction == 0) return;

  int next = static_cast<int>(state->limitsMenuField) + direction;
  if (next < 0) next = 2;
  if (next > 2) next = 0;
  state->limitsMenuField = static_cast<uint8_t>(next);
}

void limits_menu_finish(SystemState *state, bool save, bool returnToParent) {
  if (state == nullptr) return;
  const ConfigMenu parent = state->parentConfigMenu;
  state->limitsSaveEvent = save;
  state->limitsMenuActive = false;
  state->limitsEditActive = false;
  limits_input_reset(state);

  if (returnToParent && parent != ConfigMenu::None) {
    config_menu_leave_to_parent(state, parent, ConfigMenu::Limits);
    return;
  }

  state->currentConfigMenu = ConfigMenu::None;
  state->parentConfigMenu = ConfigMenu::None;
  if (core_should_reset_mode_init(*state)) state->modeInitialized = false;
}

void limits_menu_start_edit(SystemState *state) {
  if (state == nullptr) return;
  state->limitsEditActive = true;
  limits_input_reset(state);
}

void limits_menu_cancel_edit(SystemState *state) {
  if (state == nullptr) return;
  state->limitsEditActive = false;
  limits_input_reset(state);
}

bool limits_menu_append_digit(SystemState *state, char key) {
  if (state == nullptr || key < '0' || key > '9') return false;
  const int maxDigits = limits_field_max_digits(*state);
  if (state->limitsInputLength >= maxDigits) return false;
  state->limitsInputText[state->limitsInputLength++] = key;
  state->limitsInputText[state->limitsInputLength] = '\0';
  return true;
}

bool limits_menu_append_decimal(SystemState *state) {
  if (state == nullptr || !limits_field_allows_decimal(*state)) return false;
  const int maxDigits = limits_field_max_digits(*state);
  if (state->limitsInputHasDecimal || state->limitsInputLength >= maxDigits) return false;
  state->limitsInputText[state->limitsInputLength++] = '.';
  state->limitsInputText[state->limitsInputLength] = '\0';
  state->limitsInputHasDecimal = true;
  return true;
}

bool limits_menu_backspace(SystemState *state) {
  if (state == nullptr || state->limitsInputLength == 0) return false;
  state->limitsInputLength--;
  if (state->limitsInputText[state->limitsInputLength] == '.') {
    state->limitsInputHasDecimal = false;
  }
  state->limitsInputText[state->limitsInputLength] = '\0';
  return true;
}

void limits_menu_commit_edit(SystemState *state) {
  if (state == nullptr || state->limitsInputLength == 0) return;
  const float value = static_cast<float>(atof(state->limitsInputText));

  if (state->limitsMenuField == 0) {
    state->limitsDraftCurrentA = constrain(value, 1.0f, static_cast<float>(MAX_CURRENT));
  } else if (state->limitsMenuField == 1) {
    state->limitsDraftPowerW = constrain(value, 1.0f, static_cast<float>(MAX_POWER));
  } else {
    state->limitsDraftTempC = constrain(value, 30.0f, static_cast<float>(MAX_TEMP));
  }

  limits_menu_cancel_edit(state);
}

bool fan_menu_append_digit(SystemState *state, char key) {
  if (state == nullptr) return false;
  return numeric_input_append_digit(state->fanInputText, &state->fanInputLength, 2, key);
}

bool fan_menu_backspace(SystemState *state) {
  if (state == nullptr) return false;
  return numeric_input_backspace(state->fanInputText, &state->fanInputLength);
}

void fan_menu_cancel_edit(SystemState *state) {
  if (state == nullptr) return;
  state->fanEditActive = false;
  fan_input_reset(state);
}

void fan_menu_commit_edit(SystemState *state) {
  if (state == nullptr || state->fanInputLength == 0) return;
  const int value = atoi(state->fanInputText);

  if (state->fanSettingsMenuSelection == 0) {
    state->fanDraftTempC = static_cast<float>(constrain(value, MIN_FAN_TEMP_ON_C, MAX_FAN_TEMP_ON_C));
  } else if (state->fanSettingsMenuSelection == 1) {
    state->fanDraftHoldSeconds =
        static_cast<float>(constrain(value,
                                     static_cast<int>(MIN_FAN_HOLD_MS / 1000UL),
                                     static_cast<int>(MAX_FAN_HOLD_MS / 1000UL)));
  }

  fan_menu_cancel_edit(state);
}

void config_menu_activate_selection(SystemState *state, ConfigMenu menu) {
  if (state == nullptr) return;

  if (menu == ConfigMenu::Calibration) {
    const uint8_t selection = config_menu_selection_index(*state, ConfigMenu::Calibration);
    if (selection == 4) {
      calibration_menu_finish(state, false, true);
    } else {
      calibration_menu_finish(state, true);
    }
    return;
  }

  if (menu == ConfigMenu::Clock) {
    const uint8_t selection = config_menu_selection_index(*state, ConfigMenu::Clock);
    if (state->clockEditActive) {
      clock_menu_commit_edit(state);
    } else if (selection == 5) {
      clock_menu_finish(state, true, true);
    } else if (selection == 6) {
      clock_menu_finish(state, false, true);
    } else {
      clock_menu_start_edit(state);
    }
    return;
  }

  if (menu == ConfigMenu::FanSettings) {
    const uint8_t selection = config_menu_selection_index(*state, ConfigMenu::FanSettings);
    if (state->fanEditActive) {
      fan_menu_commit_edit(state);
    } else if (selection == 2) {
      state->fanManualOverrideActive = true;
      state->fanManualStateOn = !state->fanManualStateOn;
    } else if (selection == 3) {
      fan_menu_finish(state, true, true);
    } else {
      fan_menu_start_edit(state);
    }
    return;
  }

  if (menu == ConfigMenu::Update) {
    const uint8_t selection = config_menu_selection_index(*state, ConfigMenu::Update);
    if (selection == 0) {
      config_menu_enter(state, ConfigMenu::FwUpdate, ConfigMenu::Update);
    } else {
      update_menu_finish(state, true);
    }
    return;
  }

  const ConfigMenu target = config_menu_selected_target(*state, menu);
  if (menu == ConfigMenu::Root && target == ConfigMenu::None) {
    config_menu_close(state);
    return;
  }

  if (menu == ConfigMenu::Protection && target == ConfigMenu::None) {
    config_menu_leave_to_parent(state, ConfigMenu::Root, ConfigMenu::Protection);
    return;
  }

  if (target == ConfigMenu::Protection) {
    config_menu_set_selection_index(state, ConfigMenu::Protection, 0);
    config_menu_enter(state, ConfigMenu::Protection, ConfigMenu::Root);
    return;
  }

  if (target == ConfigMenu::Update) {
    update_menu_begin(state);
    return;
  }

  if (target == ConfigMenu::Clock) {
    clock_menu_begin(state);
    return;
  }

  if (target == ConfigMenu::FwUpdate) {
    config_menu_enter(state, ConfigMenu::FwUpdate, ConfigMenu::Update);
    return;
  }

  if (target == ConfigMenu::FanSettings) {
    fan_menu_begin(state);
    return;
  }

  if (target == ConfigMenu::Limits) {
    limits_menu_begin(state);
    return;
  }

  if (target == ConfigMenu::Calibration) {
    calibration_menu_begin(state);
  }
}

void config_menu_back(SystemState *state, ConfigMenu menu) {
  if (state == nullptr) return;

  if (menu == ConfigMenu::Root) {
    config_menu_close(state);
    return;
  }

  if (menu == ConfigMenu::Protection) {
    config_menu_leave_to_parent(state, ConfigMenu::Root, ConfigMenu::Protection);
    return;
  }

  if (menu == ConfigMenu::FanSettings) {
    if (state->fanEditActive) {
      fan_menu_cancel_edit(state);
    } else {
      fan_menu_finish(state, true, true);
    }
    return;
  }

  if (menu == ConfigMenu::Update) {
    update_menu_finish(state, true);
    return;
  }

  if (menu == ConfigMenu::Clock) {
    if (state->clockEditActive) {
      clock_menu_cancel_edit(state);
    } else {
      clock_menu_finish(state, false, true);
    }
    return;
  }

  if (menu == ConfigMenu::FwUpdate) {
    config_menu_leave_to_parent(state, ConfigMenu::Update, ConfigMenu::FwUpdate);
    return;
  }

  if (menu == ConfigMenu::Calibration) {
    calibration_menu_finish(state, false, true);
  }
}
