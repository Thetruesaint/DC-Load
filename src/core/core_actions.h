#ifndef CORE_ACTIONS_H
#define CORE_ACTIONS_H

#include <stdint.h>

enum class ActionType : uint8_t {
  None = 0,
  EncoderDelta,
  EncoderButtonPress,
  KeyPressed,
  LoadToggle,
  ModeSelect
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

#endif
