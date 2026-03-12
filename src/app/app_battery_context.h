#ifndef APP_BATTERY_CONTEXT_H
#define APP_BATTERY_CONTEXT_H

#include <Arduino.h>

float& app_battery_life_ref();
float& app_battery_life_previous_ref();
float& app_battery_cutoff_volts_ref();
float& app_battery_current_ref();
String& app_battery_type_ref();

#endif

