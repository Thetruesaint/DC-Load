#ifndef HAL_INPUTS_H
#define HAL_INPUTS_H

#include <stdint.h>

int32_t hal_encoder_count();
bool hal_load_button_low();
bool hal_encoder_button_low();
uint32_t hal_millis();
void hal_delay_ms(uint32_t ms);

#endif
