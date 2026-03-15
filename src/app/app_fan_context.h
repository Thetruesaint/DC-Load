#ifndef APP_FAN_CONTEXT_H
#define APP_FAN_CONTEXT_H

#include <Arduino.h>

int app_fan_temp_on_c();
void app_fan_set_temp_on_c(int value);

unsigned long app_fan_hold_ms();
void app_fan_set_hold_ms(unsigned long value);

uint8_t app_fan_hold_seconds();
void app_fan_set_hold_seconds(uint8_t seconds);
void app_fan_save_settings(int tempOnC, uint8_t holdSeconds);

bool app_fan_output_is_on();
void app_fan_set_output_state(bool on);

bool app_fan_manual_override_active();
bool app_fan_manual_state_on();
void app_fan_set_manual_override(bool active, bool on);
void app_fan_clear_manual_override();

#endif
