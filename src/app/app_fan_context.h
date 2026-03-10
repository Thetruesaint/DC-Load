#ifndef APP_FAN_CONTEXT_H
#define APP_FAN_CONTEXT_H

#include <Arduino.h>

int app_fan_temp_on_c();
void app_fan_set_temp_on_c(int value);

unsigned long app_fan_hold_ms();
void app_fan_set_hold_ms(unsigned long value);

uint8_t app_fan_hold_seconds();
void app_fan_set_hold_seconds(uint8_t seconds);

#endif
