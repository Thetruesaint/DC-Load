#include "core_engine.h"

#include <Arduino.h>

#include "../app/app_calibration_context.h"
#include "../app/app_ota.h"
#include "../config/system_constants.h"
#include "../ui/ui_state_machine.h"
#include "core_config_menus.h"
#include "core_modes.h"
#include "core_setup_flows.h"
#include "core_sync_bridge.h"

namespace {
SystemState g_state = core_state_make_default();
unsigned long g_lastTickMs = 0;

int action_direction(int32_t value) {
  return (value > 0) ? 1 : ((value < 0) ? -1 : 0);
}

bool is_transient_cont_setup_screen(UiScreen screen) {
  return screen == UiScreen::TransientContSetupLow ||
         screen == UiScreen::TransientContSetupHigh ||
         screen == UiScreen::TransientContSetupPeriod;
}

bool is_transient_list_setup_screen(UiScreen screen) {
  return screen == UiScreen::TransientListSetupCount ||
         screen == UiScreen::TransientListSetupStep;
}

bool can_open_config_from_screen(UiScreen screen) {
  return screen == UiScreen::Home ||
         screen == UiScreen::BatterySetupTask ||
         screen == UiScreen::BatterySetupCustomCutoff ||
         screen == UiScreen::BatterySetupCellCount ||
         screen == UiScreen::TransientContSetupLow ||
         screen == UiScreen::TransientContSetupHigh ||
         screen == UiScreen::TransientContSetupPeriod ||
         screen == UiScreen::TransientListSetupCount ||
         screen == UiScreen::TransientListSetupStep;
}

void leave_calibration_confirmation(bool accept) {
  if (accept) {
    app_calibration_accept_pending_result();
  } else {
    app_calibration_reject_pending_result();
  }
  app_calibration_finish_mode();
  g_state.mode = app_calibration_return_mode();
  g_state.functionIndex = app_calibration_return_function_index();
  g_state.modeConfigured = false;
  g_state.modeInitialized = false;
}

void dispatch_encoder_delta(int32_t value) {
  g_state.lastEncoderDelta = value;
  const int direction = action_direction(value);

  switch (g_state.uiScreen) {
    case UiScreen::MenuRoot:
      config_menu_step_selection(&g_state, ConfigMenu::Root, direction);
      return;
    case UiScreen::MenuProtection:
      config_menu_step_selection(&g_state, ConfigMenu::Protection, direction);
      return;
    case UiScreen::MenuUpdate:
      config_menu_step_selection(&g_state, ConfigMenu::Update, direction);
      return;
    case UiScreen::MenuFwUpdate:
      return;
    case UiScreen::MenuFanSettings:
      if (!g_state.fanEditActive) {
        config_menu_step_selection(&g_state, ConfigMenu::FanSettings, direction);
      }
      return;
    case UiScreen::MenuLimits:
      if (!g_state.limitsEditActive) {
        limits_menu_move_field(&g_state, direction);
      }
      return;
    case UiScreen::MenuCalibration:
      config_menu_step_selection(&g_state, ConfigMenu::Calibration, direction);
      return;
    default:
      core_mode_apply_encoder_delta(&g_state, direction);
      return;
  }
}

void dispatch_encoder_button_press() {
  if (g_state.uiScreen == UiScreen::BatterySetupCustomCutoff) {
    battery_setup_finish_custom(&g_state);
    return;
  }

  if (g_state.uiScreen == UiScreen::BatterySetupCellCount) {
    battery_setup_finish_cells(&g_state);
    return;
  }

  if (is_transient_cont_setup_screen(g_state.uiScreen)) {
    transient_cont_setup_commit(&g_state);
    return;
  }

  if (is_transient_list_setup_screen(g_state.uiScreen)) {
    transient_list_setup_commit(&g_state);
    return;
  }

  switch (g_state.uiScreen) {
    case UiScreen::MenuRoot:
      config_menu_activate_selection(&g_state, ConfigMenu::Root);
      return;
    case UiScreen::MenuProtection:
      config_menu_activate_selection(&g_state, ConfigMenu::Protection);
      return;
    case UiScreen::MenuUpdate:
      config_menu_activate_selection(&g_state, ConfigMenu::Update);
      return;
    case UiScreen::MenuFwUpdate:
      return;
    case UiScreen::MenuFanSettings:
      config_menu_activate_selection(&g_state, ConfigMenu::FanSettings);
      return;
    case UiScreen::MenuLimits:
      if (g_state.limitsEditActive) {
        limits_menu_commit_edit(&g_state);
      } else {
        limits_menu_start_edit(&g_state);
      }
      return;
    case UiScreen::MenuCalibration:
      config_menu_activate_selection(&g_state, ConfigMenu::Calibration);
      return;
    default:
      core_mode_move_cursor(&g_state, 1);
      return;
  }
}

void dispatch_key_for_battery_setup(char key) {
  if (g_state.uiScreen == UiScreen::BatterySetupTask) {
    if (key >= '1' && key <= '5') {
      battery_setup_select_task(&g_state, key);
    } else if (key == 'M') {
      core_mode_apply_selection(&g_state, false, key);
      reset_mode_entry_state(&g_state, g_state.mode == 3);
    }
    return;
  }

  if (g_state.uiScreen == UiScreen::BatterySetupCustomCutoff) {
    if (key == '<') {
      if (!battery_input_backspace(&g_state)) {
        g_state.batterySetupStage = 0;
        battery_input_reset(&g_state);
      }
    } else if (key == 'E') {
      battery_setup_finish_custom(&g_state);
    } else if (key == 'M') {
      g_state.batterySetupStage = 0;
      battery_input_reset(&g_state);
    } else {
      (void)battery_custom_append(&g_state, key);
    }
    return;
  }

  if (g_state.uiScreen == UiScreen::BatterySetupCellCount) {
    if (key == '<') {
      if (!numeric_input_backspace(g_state.batteryInputText, &g_state.batteryInputLength)) {
        g_state.batterySetupStage = 0;
        battery_input_reset(&g_state);
      }
    } else if (key == 'E') {
      battery_setup_finish_cells(&g_state);
    } else if (key == 'M') {
      g_state.batterySetupStage = 0;
      battery_input_reset(&g_state);
    } else {
      (void)numeric_input_append_digit(g_state.batteryInputText, &g_state.batteryInputLength, 1, key);
    }
  }
}

void dispatch_key_for_transient_setup(char key) {
  if (is_transient_cont_setup_screen(g_state.uiScreen)) {
    if (key == '<') {
      if (!transient_input_backspace(&g_state) && g_state.transientSetupStage > 0) {
        g_state.transientSetupStage--;
        transient_input_reset(&g_state);
      }
    } else if (key == 'E') {
      transient_cont_setup_commit(&g_state);
    } else if (key == 'M') {
      if (g_state.transientSetupStage > 0) {
        g_state.transientSetupStage--;
        transient_input_reset(&g_state);
      } else {
        core_mode_apply_selection(&g_state, false, key);
        reset_mode_entry_state(&g_state, g_state.mode == 3);
        if (g_state.mode == 4) {
          transient_cont_setup_begin(&g_state);
        } else {
          transient_input_reset(&g_state);
          g_state.transientSetupStage = 0;
        }
      }
    } else {
      (void)transient_input_append(&g_state, key);
    }
    return;
  }

  if (is_transient_list_setup_screen(g_state.uiScreen)) {
    if (key == '<') {
      if (!transient_list_input_backspace(&g_state)) {
        if (g_state.uiScreen == UiScreen::TransientListSetupStep) {
          if (g_state.transientListDraftField == 1) {
            g_state.transientListDraftField = 0;
          } else if (g_state.transientListDraftStepIndex > 0) {
            g_state.transientListDraftStepIndex--;
            g_state.transientListDraftField = 1;
          } else {
            g_state.transientListSetupStage = 0;
          }
        }
      }
    } else if (key == 'E') {
      transient_list_setup_commit(&g_state);
    } else if (key == 'M') {
      if (g_state.uiScreen == UiScreen::TransientListSetupCount) {
        core_mode_apply_selection(&g_state, false, key);
        reset_mode_entry_state(&g_state, false);
        transient_list_input_reset(&g_state);
        g_state.transientListSetupStage = 0;
        g_state.transientListDraftField = 0;
      } else if (g_state.transientListDraftField == 1) {
        g_state.transientListDraftField = 0;
        transient_list_input_reset(&g_state);
      } else if (g_state.transientListDraftStepIndex > 0) {
        g_state.transientListDraftStepIndex--;
        g_state.transientListDraftField = 1;
        transient_list_input_reset(&g_state);
      } else {
        g_state.transientListSetupStage = 0;
        transient_list_input_reset(&g_state);
      }
    } else {
      (void)transient_list_input_append(&g_state, key);
    }
  }
}

void dispatch_key_for_config_menu(char key) {
  switch (g_state.uiScreen) {
    case UiScreen::MenuRoot:
      if (key == '1') {
        config_menu_select_index(&g_state, ConfigMenu::Root, 0);
      } else if (key == '2') {
        config_menu_select_index(&g_state, ConfigMenu::Root, 1);
      } else if (key == '3') {
        config_menu_select_index(&g_state, ConfigMenu::Root, 2);
      } else if (key == '4') {
        config_menu_select_index(&g_state, ConfigMenu::Root, 3);
      } else if (key == 'U' || key == 'L') {
        config_menu_step_selection(&g_state, ConfigMenu::Root, -1);
      } else if (key == 'D' || key == 'R') {
        config_menu_step_selection(&g_state, ConfigMenu::Root, 1);
      } else if (key == 'E') {
        config_menu_activate_selection(&g_state, ConfigMenu::Root);
      } else if (key == '<' || key == 'M') {
        config_menu_back(&g_state, ConfigMenu::Root);
      }
      return;

    case UiScreen::MenuProtection:
      if (key == '1') {
        config_menu_select_index(&g_state, ConfigMenu::Protection, 0);
      } else if (key == '2') {
        config_menu_select_index(&g_state, ConfigMenu::Protection, 1);
      } else if (key == '3') {
        config_menu_select_index(&g_state, ConfigMenu::Protection, 2);
      } else if (key == 'U' || key == 'L') {
        config_menu_step_selection(&g_state, ConfigMenu::Protection, -1);
      } else if (key == 'D' || key == 'R') {
        config_menu_step_selection(&g_state, ConfigMenu::Protection, 1);
      } else if (key == 'E') {
        config_menu_activate_selection(&g_state, ConfigMenu::Protection);
      } else if (key == '<' || key == 'M') {
        config_menu_back(&g_state, ConfigMenu::Protection);
      }
      return;

    case UiScreen::MenuUpdate:
      if (key == '1') {
        config_menu_select_index(&g_state, ConfigMenu::Update, 0);
      } else if (key == '2') {
        config_menu_select_index(&g_state, ConfigMenu::Update, 1);
      } else if (key == 'U' || key == 'L') {
        config_menu_step_selection(&g_state, ConfigMenu::Update, -1);
      } else if (key == 'D' || key == 'R') {
        config_menu_step_selection(&g_state, ConfigMenu::Update, 1);
      } else if (key == 'E') {
        config_menu_activate_selection(&g_state, ConfigMenu::Update);
      } else if (key == '<' || key == 'M') {
        config_menu_back(&g_state, ConfigMenu::Update);
      }
      return;

    case UiScreen::MenuFwUpdate:
      if (key == 'E' && app_ota_has_error()) {
        app_ota_restart();
      } else if ((key == '<' || key == 'M') && !app_ota_is_uploading()) {
        config_menu_back(&g_state, ConfigMenu::FwUpdate);
      }
      return;

    case UiScreen::MenuFanSettings:
      if (g_state.fanEditActive) {
        if (key == '<') {
          if (!fan_menu_backspace(&g_state)) {
            fan_menu_cancel_edit(&g_state);
          }
        } else if (key == 'E') {
          fan_menu_commit_edit(&g_state);
        } else if (key == 'M') {
          fan_menu_cancel_edit(&g_state);
        } else {
          (void)fan_menu_append_digit(&g_state, key);
        }
        return;
      }

      if (key == '1') {
        config_menu_select_index(&g_state, ConfigMenu::FanSettings, 0);
      } else if (key == '2') {
        config_menu_select_index(&g_state, ConfigMenu::FanSettings, 1);
      } else if (key == '3') {
        config_menu_select_index(&g_state, ConfigMenu::FanSettings, 2);
      } else if (key == '4') {
        config_menu_select_index(&g_state, ConfigMenu::FanSettings, 3);
      } else if (key == 'U' || key == 'L') {
        config_menu_step_selection(&g_state, ConfigMenu::FanSettings, -1);
      } else if (key == 'D' || key == 'R') {
        config_menu_step_selection(&g_state, ConfigMenu::FanSettings, 1);
      } else if (key == 'E') {
        config_menu_activate_selection(&g_state, ConfigMenu::FanSettings);
      } else if (key == '<' || key == 'M') {
        config_menu_back(&g_state, ConfigMenu::FanSettings);
      }
      return;

    case UiScreen::MenuLimits:
      if (g_state.limitsEditActive) {
        if (!limits_menu_append_digit(&g_state, key) && key == '.') {
          (void)limits_menu_append_decimal(&g_state);
        } else if (key == '<') {
          if (!limits_menu_backspace(&g_state)) {
            limits_menu_cancel_edit(&g_state);
          }
        } else if (key == 'E') {
          limits_menu_commit_edit(&g_state);
        } else if (key == 'M') {
          limits_menu_cancel_edit(&g_state);
        }
        return;
      }

      if (key == '1') {
        g_state.limitsMenuField = 0;
      } else if (key == '2') {
        g_state.limitsMenuField = 1;
      } else if (key == '3') {
        g_state.limitsMenuField = 2;
      } else if (key == 'U' || key == 'L') {
        limits_menu_move_field(&g_state, -1);
      } else if (key == 'D' || key == 'R') {
        limits_menu_move_field(&g_state, 1);
      } else if (key == 'E') {
        limits_menu_start_edit(&g_state);
      } else if (key == '<' || key == 'M') {
        limits_menu_finish(&g_state, true, true);
      }
      return;

    case UiScreen::MenuCalibration:
      if (key >= '1' && key <= '5') {
        config_menu_select_index(&g_state, ConfigMenu::Calibration, static_cast<uint8_t>(key - '1'));
      } else if (key == 'U' || key == 'L') {
        config_menu_step_selection(&g_state, ConfigMenu::Calibration, -1);
      } else if (key == 'D' || key == 'R') {
        config_menu_step_selection(&g_state, ConfigMenu::Calibration, 1);
      } else if (key == 'E') {
        config_menu_activate_selection(&g_state, ConfigMenu::Calibration);
      } else if (key == '<' || key == 'M') {
        config_menu_back(&g_state, ConfigMenu::Calibration);
      }
      return;

    default:
      return;
  }
}

void dispatch_key_pressed(char key) {
  g_state.lastKeyPressed = key;

  if (g_state.uiScreen == UiScreen::Home && g_state.mode == CA && app_calibration_confirmation_active()) {
    if (key == 'E') {
      leave_calibration_confirmation(true);
    } else if (key == '<') {
      leave_calibration_confirmation(false);
    }
    return;
  }

  if (g_state.uiScreen == UiScreen::BatterySetupTask ||
      g_state.uiScreen == UiScreen::BatterySetupCustomCutoff ||
      g_state.uiScreen == UiScreen::BatterySetupCellCount) {
    dispatch_key_for_battery_setup(key);
    return;
  }

  if (is_transient_cont_setup_screen(g_state.uiScreen) || is_transient_list_setup_screen(g_state.uiScreen)) {
    dispatch_key_for_transient_setup(key);
    return;
  }

  switch (g_state.uiScreen) {
    case UiScreen::MenuRoot:
    case UiScreen::MenuProtection:
    case UiScreen::MenuUpdate:
    case UiScreen::MenuFwUpdate:
    case UiScreen::MenuFanSettings:
    case UiScreen::MenuLimits:
    case UiScreen::MenuCalibration:
      dispatch_key_for_config_menu(key);
      return;
    default:
      break;
  }

  if (!core_mode_is_managed(g_state.mode)) {
    return;
  }

  if (key == 'U') {
    core_mode_apply_encoder_delta(&g_state, 1);
  } else if (key == 'D') {
    core_mode_apply_encoder_delta(&g_state, -1);
  } else if (key == 'L') {
    core_mode_move_cursor(&g_state, -1);
  } else if (key == 'R') {
    core_mode_move_cursor(&g_state, 1);
  }
}

void dispatch_load_toggle() {
  if (g_state.uiScreen != UiScreen::Home) {
    return;
  }
  if (g_state.mode == CA && app_calibration_confirmation_active()) {
    return;
  }
  g_state.loadEnabled = !g_state.loadEnabled;
  g_state.loadToggleEvent = true;
}

void dispatch_open_config_section(int32_t rawSection) {
  if (!can_open_config_from_screen(g_state.uiScreen)) {
    return;
  }

  const ConfigSection section = decode_config_section(rawSection);
  g_state.loadEnabled = false;
  g_state.pendingConfigSection = default_config_selection(section);
  g_state.menuRootSelection = (section == ConfigSection::Calibration) ? 1 : 0;
  g_state.protectionMenuSelection = 0;
  g_state.updateMenuSelection = 0;
  g_state.fanSettingsMenuSelection = 0;
  g_state.currentConfigMenu = ConfigMenu::Root;
  g_state.parentConfigMenu = ConfigMenu::None;
  clear_config_runtime_flags(&g_state);
}
}  // namespace

void core_init() {
  g_lastTickMs = millis();
}

void core_sync_from_runtime(const RuntimeSnapshot &snapshot) {
  core_sync_merge_runtime_state(&g_state, snapshot);
  const bool calibrationMenuReturnRequested = app_calibration_consume_menu_return_request();
  if (calibrationMenuReturnRequested) {
    g_state.currentConfigMenu = ConfigMenu::Calibration;
    g_state.parentConfigMenu = ConfigMenu::Root;
    g_state.pendingConfigSection = ConfigSection::Calibration;
    g_state.calibrationMenuActive = true;
    if (g_state.calibrationMenuOption < 1 || g_state.calibrationMenuOption > 5) {
      g_state.calibrationMenuOption = 1;
    }
    ui_state_machine_reset();
  }

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
      dispatch_encoder_delta(action.value);
      break;

    case ActionType::EncoderButtonPress:
      dispatch_encoder_button_press();
      break;

    case ActionType::KeyPressed:
      dispatch_key_pressed(action.key);
      break;

    case ActionType::LoadToggle:
      dispatch_load_toggle();
      break;

    case ActionType::ModeSelect:
      core_mode_apply_selection(&g_state, action.value != 0, action.key);
      reset_mode_entry_state(&g_state, g_state.mode == 3);
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
      dispatch_open_config_section(action.value);
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
