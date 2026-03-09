#include "ui_cycle_render.h"

#include "../ui_lcd.h"

void ui_render_cycle() {
  Update_LCD();
#ifndef WOKWI_SIMULATION
  // Update_TFT();
#endif
}
