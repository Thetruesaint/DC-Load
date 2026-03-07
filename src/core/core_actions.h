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

#endif
