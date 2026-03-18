#ifndef APP_MSC_H
#define APP_MSC_H

#include "app_mode_keys.h"

MscKeyDecision app_handle_msc_key_decision(char key);
bool app_handle_msc_keys(char key);
bool app_msc_shift_active();

#endif
