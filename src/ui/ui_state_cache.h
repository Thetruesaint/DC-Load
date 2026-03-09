#ifndef UI_STATE_CACHE_H
#define UI_STATE_CACHE_H

#include "ui_view_state.h"

void ui_state_cache_set(const UiViewState &state);
const UiViewState &ui_state_cache_get();

#endif
