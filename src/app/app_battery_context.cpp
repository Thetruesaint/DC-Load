#include "app_battery_context.h"

namespace {
float batteryLife = 0.0f;
float batteryLifePrevious = 0.0f;
float batteryCutoffVolts = 0.0f;
float batteryCurrent = 0.0f;
bool batteryDone = false;
String batteryType = "    ";
}

float& app_battery_life_ref() {
  return batteryLife;
}

float& app_battery_life_previous_ref() {
  return batteryLifePrevious;
}

float& app_battery_cutoff_volts_ref() {
  return batteryCutoffVolts;
}

float& app_battery_current_ref() {
  return batteryCurrent;
}

String& app_battery_type_ref() {
  return batteryType;
}

bool& app_battery_done_ref() {
  return batteryDone;
}

