#ifndef UI_STATE_MACHINE_H
#define UI_STATE_MACHINE_H

#include "../core/core_state.h"
#include "ui_view_state.h"

void ui_state_machine_reset();
void ui_state_machine_tick(UiScreen targetScreen, const UiViewState &viewState);
UiScreen ui_state_machine_current_screen();

#endif
