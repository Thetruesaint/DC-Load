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

void menu_root_step_selection(SystemState *state, int direction) {
  if (state == nullptr || direction == 0) return;

  if (state->pendingConfigSection != ConfigSection::Limits &&
      state->pendingConfigSection != ConfigSection::Calibration) {
    state->pendingConfigSection = ConfigSection::Limits;
  }

  state->pendingConfigSection = (state->pendingConfigSection == ConfigSection::Limits)
                                    ? ConfigSection::Calibration
                                    : ConfigSection::Limits;
}

void limits_input_reset(SystemState *state) {
  if (state == nullptr) return;
  state->limitsInputText[0] = '\0';
  state->limitsInputLength = 0;
  state->limitsInputHasDecimal = false;
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

void limits_menu_adjust_value(SystemState *state, int direction) {
  if (state == nullptr || direction == 0) return;
  if (state->limitsEditActive) return;

  if (state->limitsMenuField == 0) {
    state->limitsDraftCurrentA = constrain(
        state->limitsDraftCurrentA + (direction * 0.1f),
        1.0f,
        static_cast<float>(MAX_CURRENT));
    return;
  }

  if (state->limitsMenuField == 1) {
    state->limitsDraftPowerW = constrain(
        state->limitsDraftPowerW + (direction * 1.0f),
        1.0f,
        static_cast<float>(MAX_POWER));
    return;
  }

  state->limitsDraftTempC = constrain(
      state->limitsDraftTempC + (direction * 1.0f),
      30.0f,
      static_cast<float>(MAX_TEMP));
}

void limits_menu_begin(SystemState *state) {
  if (state == nullptr) return;
  state->limitsMenuActive = true;
  state->limitsMenuField = 0;
  state->limitsDraftCurrentA = state->currentCutOffA;
  state->limitsDraftPowerW = state->powerCutOffW;
  state->limitsDraftTempC = state->tempCutOffC;
  state->limitsEditActive = false;
  limits_input_reset(state);
}

void limits_menu_finish(SystemState *state, bool save, bool returnToRoot = false) {
  if (state == nullptr) return;
  state->limitsSaveEvent = save;
  state->limitsMenuActive = false;
  state->limitsEditActive = false;
  limits_input_reset(state);
  state->pendingConfigSection = returnToRoot ? ConfigSection::Limits : ConfigSection::None;
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

void calibration_menu_step_option(SystemState *state, int direction) {
  if (state == nullptr || direction == 0) return;

  int option = static_cast<int>(state->calibrationMenuOption) + direction;
  if (option < 1) option = 4;
  if (option > 4) option = 1;
  state->calibrationMenuOption = static_cast<uint8_t>(option);
}

void calibration_menu_begin(SystemState *state) {
  if (state == nullptr) return;
  state->calibrationMenuActive = true;
  if (state->calibrationMenuOption < 1 || state->calibrationMenuOption > 4) {
    state->calibrationMenuOption = 1;
  }
}

void calibration_menu_finish(SystemState *state, bool apply, bool returnToRoot = false) {
  if (state == nullptr) return;
  state->calibrationMenuApplyEvent = apply;
  state->calibrationMenuActive = false;
  state->pendingConfigSection = returnToRoot ? ConfigSection::Calibration : ConfigSection::None;
  state->modeInitialized = false;
}
}

void core_init() {
  g_lastTickMs = millis();
}

void core_sync_from_legacy(const SystemState &state) {
  const uint32_t actionCounter = g_state.actionCounter;
  const UiScreen uiScreen = g_state.uiScreen;
  const ConfigSection pendingConfigSection = g_state.pendingConfigSection;
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

  g_state = state;
  g_state.actionCounter = actionCounter;
  g_state.uiScreen = uiScreen;
  g_state.pendingConfigSection = pendingConfigSection;
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
        menu_root_step_selection(&g_state, direction);
        break;
      }

      if (g_state.uiScreen == UiScreen::MenuLimits) {
        if (!g_state.limitsEditActive) {
          limits_menu_move_field(&g_state, (action.value > 0) ? 1 : ((action.value < 0) ? -1 : 0));
        }
        break;
      }

      if (g_state.uiScreen == UiScreen::MenuCalibration) {
        calibration_menu_step_option(&g_state, (action.value > 0) ? 1 : ((action.value < 0) ? -1 : 0));
        break;
      }

      core_mode_apply_encoder_delta(&g_state, (action.value > 0) ? 1 : ((action.value < 0) ? -1 : 0));
      break;

    case ActionType::EncoderButtonPress:
      if (g_state.uiScreen == UiScreen::MenuRoot) {
        if (g_state.pendingConfigSection == ConfigSection::Limits) {
          limits_menu_begin(&g_state);
          g_state.pendingConfigSection = ConfigSection::None;
        } else if (g_state.pendingConfigSection == ConfigSection::Calibration) {
          calibration_menu_begin(&g_state);
          g_state.pendingConfigSection = ConfigSection::None;
        }
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
        calibration_menu_finish(&g_state, true);
        break;
      }

      core_mode_move_cursor(&g_state, 1);
      break;

    case ActionType::KeyPressed:
      g_state.lastKeyPressed = action.key;

      if (g_state.uiScreen == UiScreen::MenuRoot) {
        if (action.key == '1') {
          g_state.pendingConfigSection = ConfigSection::Limits;
        } else if (action.key == '2') {
          g_state.pendingConfigSection = ConfigSection::Calibration;
        } else if (action.key == 'U' || action.key == 'L') {
          menu_root_step_selection(&g_state, -1);
        } else if (action.key == 'D' || action.key == 'R') {
          menu_root_step_selection(&g_state, 1);
        } else if (action.key == 'E') {
          if (g_state.pendingConfigSection == ConfigSection::Limits) {
            limits_menu_begin(&g_state);
          } else if (g_state.pendingConfigSection == ConfigSection::Calibration) {
            calibration_menu_begin(&g_state);
          }
          g_state.pendingConfigSection = ConfigSection::None;
        } else if (action.key == '<' || action.key == 'M') {
          g_state.pendingConfigSection = ConfigSection::None;
          g_state.modeInitialized = false;
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
        if (action.key >= '1' && action.key <= '4') {
          g_state.calibrationMenuOption = static_cast<uint8_t>(action.key - '0');
        } else if (action.key == 'U' || action.key == 'L') {
          calibration_menu_step_option(&g_state, -1);
        } else if (action.key == 'D' || action.key == 'R') {
          calibration_menu_step_option(&g_state, 1);
        } else if (action.key == 'E') {
          calibration_menu_finish(&g_state, true);
        } else if (action.key == '<' || action.key == 'M') {
          calibration_menu_finish(&g_state, false, true);
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
      g_state.openLimitsConfigEvent = false;
      g_state.openCalibrationConfigEvent = false;
      g_state.calibrationValueConfirmEvent = false;
      g_state.limitsMenuActive = false;
      g_state.limitsEditActive = false;
      limits_input_reset(&g_state);
      g_state.calibrationMenuActive = false;
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
      g_state.openLimitsConfigEvent = false;
      g_state.openCalibrationConfigEvent = false;
      g_state.limitsMenuActive = false;
      g_state.limitsEditActive = false;
      limits_input_reset(&g_state);
      g_state.calibrationMenuActive = false;
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



