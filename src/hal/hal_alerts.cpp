#include "hal_alerts.h"

#include "../config/system_constants.h"

void hal_buzzer_set(bool enabled) {
  digitalWrite(BUZZER, enabled ? HIGH : LOW);
}
