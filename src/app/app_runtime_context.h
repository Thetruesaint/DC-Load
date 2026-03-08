#ifndef APP_RUNTIME_CONTEXT_H
#define APP_RUNTIME_CONTEXT_H

float app_runtime_encoder_position();
void app_runtime_set_encoder_position(float value);
float app_runtime_encoder_step();
unsigned long app_runtime_encoder_max();

int app_runtime_cursor_position();
void app_runtime_set_cursor_position(int position);

#endif
