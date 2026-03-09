#ifndef CORE_ACTIONS_H
#define CORE_ACTIONS_H

#include <stdint.h>

#include "core_state.h"

enum class ActionType : uint8_t {
  None = 0,
  EncoderDelta,
  EncoderButtonPress,
  KeyPressed,
  LoadToggle,
  ModeSelect,
  ValueConfirm,
  OpenConfigSection
};

struct UserAction {
  ActionType type;
  int32_t value;
  char key;
};

constexpr UserAction make_none_action() {
  return {ActionType::None, 0, '\0'};
}

constexpr UserAction make_encoder_delta_action(int32_t delta) {
  return {ActionType::EncoderDelta, delta, '\0'};
}

constexpr UserAction make_encoder_button_press_action() {
  return {ActionType::EncoderButtonPress, 0, '\0'};
}

constexpr UserAction make_key_pressed_action(char key) {
  return {ActionType::KeyPressed, 0, key};
}

constexpr UserAction make_load_toggle_action() {
  return {ActionType::LoadToggle, 0, '\0'};
}

constexpr UserAction make_mode_select_action(bool shiftSelection, char key = '\0') {
  return {ActionType::ModeSelect, shiftSelection ? 1 : 0, key};
}

constexpr UserAction make_value_confirm_action(int32_t valueMilli) {
  return {ActionType::ValueConfirm, valueMilli, '\0'};
}

constexpr UserAction make_open_config_section_action(ConfigSection section) {
  return {ActionType::OpenConfigSection, static_cast<int32_t>(section), '\0'};
}

constexpr UserAction make_open_limits_config_action() {
  return make_open_config_section_action(ConfigSection::Limits);
}

constexpr UserAction make_open_calibration_config_action() {
  return make_open_config_section_action(ConfigSection::Calibration);
}

#endif
