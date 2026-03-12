#include "app_health_context.h"

namespace {
bool healthOk = true;
}

bool app_health_is_ok() {
  return healthOk;
}

void app_health_set_ok(bool ok) {
  healthOk = ok;
}

