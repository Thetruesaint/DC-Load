#include "core_engine.h"

#include <Arduino.h>
#include <cstdlib>
#include <cstring>

#include "../config/system_constants.h"
#include "core_modes.h"

namespace {
SystemState g_state = core_state_make_default();
unsigned long g_lastTickMs = 0;

ConfigSection decode_config_section(int32_t raw) {
  const ConfigSection section = static_cast<ConfigSection>(raw);
  if (section == ConfigSection::Limits || section == ConfigSection::Calibration) {
    return section;
  }
  return ConfigSection::None;
}

ConfigSection default_config_selection(ConfigSection section) {
  return (section == ConfigSection::Calibration) ? ConfigSection::Calibration : ConfigSection::Limits;
}

ConfigMenu menu_root_menu_for_index(uint8_t index) {
  switch (index) {
    case 0: return ConfigMenu::Protection;
    case 1: return ConfigMenu::Calibration;
    default: return ConfigMenu::None;
  }
}

uint8_t menu_root_index_for_menu(ConfigMenu menu) {
  return (menu == ConfigMenu::Calibration) ? 1 : (menu == ConfigMenu::None) ? 2 : 0;
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
    case ConfigMenu::Root: return 3;
    case ConfigMenu::Protection: return 3;
    case ConfigMenu::FanSettings: return 3;
    case ConfigMenu::Calibration: return 5;
    default: return 0;
  }
}

uint8_t config_menu_selection_index(const SystemState &state, ConfigMenu menu) {
  if (menu == ConfigMenu::Root) return state.menuRootSelection;
  if (menu == ConfigMenu::Protection) return state.protectionMenuSelection;
  if (menu == ConfigMenu::FanSettings) return state.fanSettingsMenuSelection;
  if (menu == ConfigMenu::Calibration) return (state.calibrationMenuOption > 0) ? static_cast<uint8_t>(state.calibrationMenuOption - 1) : 0;
  return 0;
}

void config_menu_set_selection_index(SystemState *state, ConfigMenu menu, uint8_t index) {
  if (state == nullptr) return;

  if (menu == ConfigMenu::Root) {
    state->menuRootSelection = index;
  } else if (menu == ConfigMenu::Protection) {
    state->protectionMenuSelection = index;
  } else if (menu == ConfigMenu::FanSettings) {
    state->fanSettingsMenuSelection = index;
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
  return ConfigMenu::None;
}

ConfigSection config_menu_pending_section_for_target(ConfigMenu target) {
  if (target == ConfigMenu::Calibration) {
    return ConfigSection::Calibration;
  }
  if (target == ConfigMenu::Protection || target == ConfigMenu::Limits || target == ConfigMenu::FanSettings) {
    return ConfigSection::Limits;
  }
  return ConfigSection::None;
}

void config_menu_sync_pending_section(SystemState *state, ConfigMenu menu) {
  if (state == nullptr) return;
  if (menu == ConfigMenu::Calibration) {
    state->pendingConfigSection = ConfigSection::Calibration;
    return;
  }
  state->pendingConfigSection = config_menu_pending_section_for_target(config_menu_selected_target(*state, menu));
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
  config_menu_sync_pending_section(state, menu);
}

void config_menu_select_index(SystemState *state, ConfigMenu menu, uint8_t index) {
  if (state == nullptr) return;

  const uint8_t itemCount = config_menu_item_count(menu);
  if (itemCount == 0) return;

  config_menu_set_selection_index(state, menu, (index < itemCount) ? index : 0);
  state->currentConfigMenu = menu;
  config_menu_sync_pending_section(state, menu);
}

void config_menu_enter(SystemState *state, ConfigMenu menu, ConfigMenu parent) {
  if (state == nullptr) return;
  state->currentConfigMenu = menu;
  state->parentConfigMenu = parent;
  config_menu_sync_pending_section(state, menu);
}

void config_menu_leave_to_parent(SystemState *state, ConfigMenu parent, ConfigMenu child) {
  if (state == nullptr) return;

  state->currentConfigMenu = parent;
  state->parentConfigMenu = (parent == ConfigMenu::Protection) ? ConfigMenu::Root : ConfigMenu::None;
  state->modeInitialized = false;

  if (parent == ConfigMenu::Root) {
    state->menuRootSelection = menu_root_index_for_menu(child);
  } else if (parent == ConfigMenu::Protection) {
    state->protectionMenuSelection = (child == ConfigMenu::FanSettings) ? 1 : 0;
  }

  config_menu_sync_pending_section(state, parent);
}

void config_menu_close(SystemState *state) {
  if (state == nullptr) return;
  state->pendingConfigSection = ConfigSection::None;
  state->currentConfigMenu = ConfigMenu::None;
  state->parentConfigMenu = ConfigMenu::None;
  state->modeInitialized = false;
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

bool numeric_input_append_digit(char *buffer, uint8_t *length, uint8_t maxDigits, char key) {
  if (buffer == nullptr || length == nullptr || key < '0' || key > '9') return false;
  if (*length >= maxDigits) return false;
  buffer[*length] = key;
  (*length)++;
  buffer[*length] = '\0';
  return true;
}

bool numeric_input_backspace(char *buffer, uint8_t *length) {
  if (buffer == nullptr || length == nullptr || *length == 0) return false;
  (*length)--;
  buffer[*length] = '\0';
  return true;
}

bool limits_field_allows_decimal(const SystemState &state) {
  return state.limitsMenuField != 2;
}

int limits_field_max_digits(const SystemState &state) {
  return (state.limitsMenuField == 2) ? 2 : 5;
}

void limits_menu_move_field(SystemState *state, int direction) {
  if (state == nullptr || direction == 0) return;

  int next = static_cast<int>(state->limitsMenuField) + direction;
  if (next < 0) next = 2;
  if (next > 2) next = 0;
  state->limitsMenuField = static_cast<uint8_t>(next);
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

void limits_menu_finish(SystemState *state, bool save, bool returnToParent = false) {
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

  state->pendingConfigSection = ConfigSection::None;
  state->currentConfigMenu = ConfigMenu::None;
  state->parentConfigMenu = ConfigMenu::None;
  state->modeInitialized = false;
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
  fan_input_reset(state);

  if (returnToParent && parent != ConfigMenu::None) {
    config_menu_leave_to_parent(state, parent, ConfigMenu::FanSettings);
    return;
  }

  state->pendingConfigSection = ConfigSection::None;
  state->currentConfigMenu = ConfigMenu::None;
  state->parentConfigMenu = ConfigMenu::None;
  state->modeInitialized = false;
}

void fan_menu_start_edit(SystemState *state) {
  if (state == nullptr) return;
  if (state->fanSettingsMenuSelection >= 2) return;
  state->fanEditActive = true;
  fan_input_reset(state);
}

void fan_menu_cancel_edit(SystemState *state) {
  if (state == nullptr) return;
  state->fanEditActive = false;
  fan_input_reset(state);
}

bool fan_menu_append_digit(SystemState *state, char key) {
  if (state == nullptr) return false;
  return numeric_input_append_digit(state->fanInputText, &state->fanInputLength, 2, key);
}

bool fan_menu_backspace(SystemState *state) {
  if (state == nullptr) return false;
  return numeric_input_backspace(state->fanInputText, &state->fanInputLength);
}

void fan_menu_commit_edit(SystemState *state) {
  if (state == nullptr || state->fanInputLength == 0) return;
  const int value = atoi(state->fanInputText);

  if (state->fanSettingsMenuSelection == 0) {
    state->fanDraftTempC = static_cast<float>(constrain(value, MIN_FAN_TEMP_ON_C, MAX_FAN_TEMP_ON_C));
  } else if (state->fanSettingsMenuSelection == 1) {
    state->fanDraftHoldSeconds = static_cast<float>(constrain(value, static_cast<int>(MIN_FAN_HOLD_MS / 1000UL), static_cast<int>(MAX_FAN_HOLD_MS / 1000UL)));
  }

  fan_menu_cancel_edit(state);
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

  state->pendingConfigSection = ConfigSection::None;
  state->currentConfigMenu = ConfigMenu::None;
  state->parentConfigMenu = ConfigMenu::None;
  state->modeInitialized = false;
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

  if (menu == ConfigMenu::FanSettings) {
    const uint8_t selection = config_menu_selection_index(*state, ConfigMenu::FanSettings);
    if (state->fanEditActive) {
      fan_menu_commit_edit(state);
    } else if (selection == 2) {
      fan_menu_finish(state, true, true);
    } else {
      fan_menu_start_edit(state);
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

  if (target == ConfigMenu::FanSettings) {
    fan_menu_begin(state);
    return;
  }

  if (target == ConfigMenu::Limits) {
    limits_menu_begin(state);
    state->pendingConfigSection = ConfigSection::None;
    return;
  }

  if (target == ConfigMenu::Calibration) {
    calibration_menu_begin(state);
    state->pendingConfigSection = ConfigSection::None;
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

  if (menu == ConfigMenu::Calibration) {
    calibration_menu_finish(state, false, true);
  }
}
}

void core_init() {
  g_lastTickMs = millis();
}

void core_sync_from_legacy(const SystemState &state) {
  const uint32_t actionCounter = g_state.actionCounter;
  const UiScreen uiScreen = g_state.uiScreen;
  const ConfigSection pendingConfigSection = g_state.pendingConfigSection;
  const uint8_t menuRootSelection = g_state.menuRootSelection;
  const uint8_t protectionMenuSelection = g_state.protectionMenuSelection;
  const uint8_t fanSettingsMenuSelection = g_state.fanSettingsMenuSelection;
  const ConfigMenu currentConfigMenu = g_state.currentConfigMenu;
  const ConfigMenu parentConfigMenu = g_state.parentConfigMenu;
  const int32_t lastEncoderDelta = g_state.lastEncoderDelta;
  const char lastKeyPressed = g_state.lastKeyPressed;
  const bool limitsMenuActive = g_state.limitsMenuActive;
  const uint8_t limitsMenuField = g_state.limitsMenuField;
  const float limitsDraftCurrentA = g_state.limitsDraftCurrentA;
  const float limitsDraftPowerW = g_state.limitsDraftPowerW;
  const float limitsDraftTempC = g_state.limitsDraftTempC;
  const bool limitsEditActive = g_state.limitsEditActive;
  char limitsInputText[8] = {0};
  std::strncpy(limitsInputText, g_state.limitsInputText, sizeof(limitsInputText) - 1);
  const uint8_t limitsInputLength = g_state.limitsInputLength;
  const bool limitsInputHasDecimal = g_state.limitsInputHasDecimal;
  const bool calibrationMenuActive = g_state.calibrationMenuActive;
  const uint8_t calibrationMenuOption = g_state.calibrationMenuOption;
  const float fanDraftTempC = g_state.fanDraftTempC;
  const float fanDraftHoldSeconds = g_state.fanDraftHoldSeconds;
  const bool fanEditActive = g_state.fanEditActive;
  char fanInputText[8] = {0};
  std::strncpy(fanInputText, g_state.fanInputText, sizeof(fanInputText) - 1);
  const uint8_t fanInputLength = g_state.fanInputLength;

  g_state = state;
  g_state.actionCounter = actionCounter;
  g_state.uiScreen = uiScreen;
  g_state.pendingConfigSection = pendingConfigSection;
  g_state.menuRootSelection = menuRootSelection;
  g_state.protectionMenuSelection = protectionMenuSelection;
  g_state.fanSettingsMenuSelection = fanSettingsMenuSelection;
  g_state.currentConfigMenu = currentConfigMenu;
  g_state.parentConfigMenu = parentConfigMenu;
  g_state.lastEncoderDelta = lastEncoderDelta;
  g_state.lastKeyPressed = lastKeyPressed;
  g_state.limitsMenuActive = limitsMenuActive;
  g_state.limitsMenuField = limitsMenuField;
  g_state.limitsDraftCurrentA = limitsDraftCurrentA;
  g_state.limitsDraftPowerW = limitsDraftPowerW;
  g_state.limitsDraftTempC = limitsDraftTempC;
  g_state.limitsEditActive = limitsEditActive;
  std::strncpy(g_state.limitsInputText, limitsInputText, sizeof(g_state.limitsInputText) - 1);
  g_state.limitsInputText[sizeof(g_state.limitsInputText) - 1] = '\0';
  g_state.limitsInputLength = limitsInputLength;
  g_state.limitsInputHasDecimal = limitsInputHasDecimal;
  g_state.calibrationMenuActive = calibrationMenuActive;
  g_state.calibrationMenuOption = calibrationMenuOption;
  g_state.fanDraftTempC = fanDraftTempC;
  g_state.fanDraftHoldSeconds = fanDraftHoldSeconds;
  g_state.fanEditActive = fanEditActive;
  std::strncpy(g_state.fanInputText, fanInputText, sizeof(g_state.fanInputText) - 1);
  g_state.fanInputText[sizeof(g_state.fanInputText) - 1] = '\0';
  g_state.fanInputLength = fanInputLength;

  core_mode_normalize_state(&g_state);
  core_mode_update_setpoints(&g_state);
  core_mode_update_ui_screen(&g_state);
}

void core_begin_cycle() {
  core_state_clear_one_shot_events(&g_state);
}

void core_dispatch(const UserAction &action) {
  switch (action.type) {
    case ActionType::EncoderDelta:
      g_state.lastEncoderDelta = action.value;

      if (g_state.uiScreen == UiScreen::MenuRoot) {
        const int direction = (action.value > 0) ? 1 : ((action.value < 0) ? -1 : 0);
        config_menu_step_selection(&g_state, ConfigMenu::Root, direction);
        break;
      }

      if (g_state.uiScreen == UiScreen::MenuProtection) {
        const int direction = (action.value > 0) ? 1 : ((action.value < 0) ? -1 : 0);
        config_menu_step_selection(&g_state, ConfigMenu::Protection, direction);
        break;
      }

      if (g_state.uiScreen == UiScreen::MenuFanSettings) {
        if (!g_state.fanEditActive) {
          const int direction = (action.value > 0) ? 1 : ((action.value < 0) ? -1 : 0);
          config_menu_step_selection(&g_state, ConfigMenu::FanSettings, direction);
        }
        break;
      }

      if (g_state.uiScreen == UiScreen::MenuLimits) {
        if (!g_state.limitsEditActive) {
          limits_menu_move_field(&g_state, (action.value > 0) ? 1 : ((action.value < 0) ? -1 : 0));
        }
        break;
      }

      if (g_state.uiScreen == UiScreen::MenuCalibration) {
        const int direction = (action.value > 0) ? 1 : ((action.value < 0) ? -1 : 0);
        config_menu_step_selection(&g_state, ConfigMenu::Calibration, direction);
        break;
      }

      core_mode_apply_encoder_delta(&g_state, (action.value > 0) ? 1 : ((action.value < 0) ? -1 : 0));
      break;

    case ActionType::EncoderButtonPress:
      if (g_state.uiScreen == UiScreen::MenuRoot) {
        config_menu_activate_selection(&g_state, ConfigMenu::Root);
        break;
      }

      if (g_state.uiScreen == UiScreen::MenuProtection) {
        config_menu_activate_selection(&g_state, ConfigMenu::Protection);
        break;
      }

      if (g_state.uiScreen == UiScreen::MenuFanSettings) {
        config_menu_activate_selection(&g_state, ConfigMenu::FanSettings);
        break;
      }

      if (g_state.uiScreen == UiScreen::MenuLimits) {
        if (g_state.limitsEditActive) {
          limits_menu_commit_edit(&g_state);
        } else {
          limits_menu_start_edit(&g_state);
        }
        break;
      }

      if (g_state.uiScreen == UiScreen::MenuCalibration) {
        config_menu_activate_selection(&g_state, ConfigMenu::Calibration);
        break;
      }

      core_mode_move_cursor(&g_state, 1);
      break;

    case ActionType::KeyPressed:
      g_state.lastKeyPressed = action.key;

      if (g_state.uiScreen == UiScreen::MenuRoot) {
        if (action.key == '1') {
          config_menu_select_index(&g_state, ConfigMenu::Root, 0);
        } else if (action.key == '2') {
          config_menu_select_index(&g_state, ConfigMenu::Root, 1);
        } else if (action.key == '3') {
          config_menu_select_index(&g_state, ConfigMenu::Root, 2);
        } else if (action.key == 'U' || action.key == 'L') {
          config_menu_step_selection(&g_state, ConfigMenu::Root, -1);
        } else if (action.key == 'D' || action.key == 'R') {
          config_menu_step_selection(&g_state, ConfigMenu::Root, 1);
        } else if (action.key == 'E') {
          config_menu_activate_selection(&g_state, ConfigMenu::Root);
        } else if (action.key == '<' || action.key == 'M') {
          config_menu_back(&g_state, ConfigMenu::Root);
        }
        break;
      }

      if (g_state.uiScreen == UiScreen::MenuProtection) {
        if (action.key == '1') {
          config_menu_select_index(&g_state, ConfigMenu::Protection, 0);
        } else if (action.key == '2') {
          config_menu_select_index(&g_state, ConfigMenu::Protection, 1);
        } else if (action.key == '3') {
          config_menu_select_index(&g_state, ConfigMenu::Protection, 2);
        } else if (action.key == 'U' || action.key == 'L') {
          config_menu_step_selection(&g_state, ConfigMenu::Protection, -1);
        } else if (action.key == 'D' || action.key == 'R') {
          config_menu_step_selection(&g_state, ConfigMenu::Protection, 1);
        } else if (action.key == 'E') {
          config_menu_activate_selection(&g_state, ConfigMenu::Protection);
        } else if (action.key == '<' || action.key == 'M') {
          config_menu_back(&g_state, ConfigMenu::Protection);
        }
        break;
      }

      if (g_state.uiScreen == UiScreen::MenuFanSettings) {
        if (g_state.fanEditActive) {
          if (action.key == '<') {
            if (!fan_menu_backspace(&g_state)) {
              fan_menu_cancel_edit(&g_state);
            }
          } else if (action.key == 'E') {
            fan_menu_commit_edit(&g_state);
          } else if (action.key == 'M') {
            fan_menu_cancel_edit(&g_state);
          } else {
            (void)fan_menu_append_digit(&g_state, action.key);
          }
          break;
        }

        if (action.key == '1') {
          config_menu_select_index(&g_state, ConfigMenu::FanSettings, 0);
        } else if (action.key == '2') {
          config_menu_select_index(&g_state, ConfigMenu::FanSettings, 1);
        } else if (action.key == '3') {
          config_menu_select_index(&g_state, ConfigMenu::FanSettings, 2);
        } else if (action.key == 'U' || action.key == 'L') {
          config_menu_step_selection(&g_state, ConfigMenu::FanSettings, -1);
        } else if (action.key == 'D' || action.key == 'R') {
          config_menu_step_selection(&g_state, ConfigMenu::FanSettings, 1);
        } else if (action.key == 'E') {
          config_menu_activate_selection(&g_state, ConfigMenu::FanSettings);
        } else if (action.key == '<' || action.key == 'M') {
          config_menu_back(&g_state, ConfigMenu::FanSettings);
        }
        break;
      }

      if (g_state.uiScreen == UiScreen::MenuLimits) {
        if (g_state.limitsEditActive) {
          if (!limits_menu_append_digit(&g_state, action.key) && action.key == '.') {
            (void)limits_menu_append_decimal(&g_state);
          } else if (action.key == '<') {
            if (!limits_menu_backspace(&g_state)) {
              limits_menu_cancel_edit(&g_state);
            }
          } else if (action.key == 'E') {
            limits_menu_commit_edit(&g_state);
          } else if (action.key == 'M') {
            limits_menu_cancel_edit(&g_state);
          }
          break;
        }

        if (action.key == '1') {
          g_state.limitsMenuField = 0;
        } else if (action.key == '2') {
          g_state.limitsMenuField = 1;
        } else if (action.key == '3') {
          g_state.limitsMenuField = 2;
        } else if (action.key == 'U' || action.key == 'L') {
          limits_menu_move_field(&g_state, -1);
        } else if (action.key == 'D' || action.key == 'R') {
          limits_menu_move_field(&g_state, 1);
        } else if (action.key == 'E') {
          limits_menu_start_edit(&g_state);
        } else if (action.key == '<' || action.key == 'M') {
          limits_menu_finish(&g_state, true, true);
        }
        break;
      }

      if (g_state.uiScreen == UiScreen::MenuCalibration) {
        if (action.key >= '1' && action.key <= '5') {
          config_menu_select_index(&g_state, ConfigMenu::Calibration, static_cast<uint8_t>(action.key - '1'));
        } else if (action.key == 'U' || action.key == 'L') {
          config_menu_step_selection(&g_state, ConfigMenu::Calibration, -1);
        } else if (action.key == 'D' || action.key == 'R') {
          config_menu_step_selection(&g_state, ConfigMenu::Calibration, 1);
        } else if (action.key == 'E') {
          config_menu_activate_selection(&g_state, ConfigMenu::Calibration);
        } else if (action.key == '<' || action.key == 'M') {
          config_menu_back(&g_state, ConfigMenu::Calibration);
        }
        break;
      }

      if (!core_mode_is_managed(g_state.mode)) {
        break;
      }

      if (action.key == 'U') {
        core_mode_apply_encoder_delta(&g_state, 1);
      } else if (action.key == 'D') {
        core_mode_apply_encoder_delta(&g_state, -1);
      } else if (action.key == 'L') {
        core_mode_move_cursor(&g_state, -1);
      } else if (action.key == 'R') {
        core_mode_move_cursor(&g_state, 1);
      }
      break;

    case ActionType::LoadToggle:
      g_state.loadEnabled = !g_state.loadEnabled;
      g_state.loadToggleEvent = true;
      if (!g_state.loadEnabled) {
        g_state.setCurrent_mA = 0.0f;
      }
      break;

    case ActionType::ModeSelect:
      core_mode_apply_selection(&g_state, action.value != 0, action.key);
      g_state.pendingConfigSection = ConfigSection::None;
      g_state.menuRootSelection = 0;
      g_state.protectionMenuSelection = 0;
      g_state.fanSettingsMenuSelection = 0;
      g_state.currentConfigMenu = ConfigMenu::None;
      g_state.parentConfigMenu = ConfigMenu::None;
      g_state.openLimitsConfigEvent = false;
      g_state.openCalibrationConfigEvent = false;
      g_state.calibrationValueConfirmEvent = false;
      g_state.limitsMenuActive = false;
      g_state.limitsEditActive = false;
      limits_input_reset(&g_state);
      g_state.calibrationMenuActive = false;
      g_state.fanEditActive = false;
      fan_input_reset(&g_state);
      break;

    case ActionType::ValueConfirm:
      if (g_state.mode == CA) {
        g_state.calibrationRealValue = static_cast<float>(action.value) / 1000.0f;
        g_state.calibrationValueConfirmEvent = true;
      } else {
        g_state.encoderPositionRaw = static_cast<float>(action.value);
      }
      break;

    case ActionType::OpenConfigSection:
      if (g_state.uiScreen != UiScreen::Home) {
        break;
      }
      g_state.pendingConfigSection = default_config_selection(decode_config_section(action.value));
      g_state.menuRootSelection = menu_root_index_for_menu(
          (decode_config_section(action.value) == ConfigSection::Calibration)
              ? ConfigMenu::Calibration
              : ConfigMenu::Protection);
      g_state.protectionMenuSelection = 0;
      g_state.fanSettingsMenuSelection = 0;
      g_state.currentConfigMenu = ConfigMenu::Root;
      g_state.parentConfigMenu = ConfigMenu::None;
      g_state.openLimitsConfigEvent = false;
      g_state.openCalibrationConfigEvent = false;
      g_state.limitsMenuActive = false;
      g_state.limitsEditActive = false;
      limits_input_reset(&g_state);
      g_state.calibrationMenuActive = false;
      g_state.fanEditActive = false;
      fan_input_reset(&g_state);
      break;

    case ActionType::None:
    default:
      return;
  }

  core_mode_update_setpoints(&g_state);
  core_mode_update_ui_screen(&g_state);
  g_state.actionCounter++;
}

void core_tick_10ms() {
  const unsigned long now = millis();
  if ((now - g_lastTickMs) < 10UL) return;
  g_lastTickMs = now;
}

const SystemState &core_get_state() {
  return g_state;
}
