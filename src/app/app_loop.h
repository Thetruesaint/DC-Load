#ifndef APP_LOOP_H
#define APP_LOOP_H

#include "../core/core_actions.h"

void app_init();
void app_tick();
void app_push_action(const UserAction &action);

#endif