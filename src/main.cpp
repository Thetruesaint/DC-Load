#include "app/app_loop.h"
#include "app/app_runtime.h"
#include "app/app_startup.h"

void setup() {
  app_startup_run();
  app_init();
}

void loop() {
  app_run_cycle();
}
