#ifndef APP_IO_CONTEXT_H
#define APP_IO_CONTEXT_H

#include <stdint.h>

int32_t app_io_encoder_count();
bool app_io_load_button_low();
bool app_io_encoder_button_low();
uint32_t app_io_millis();

#endif
