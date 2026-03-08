#include "legacy_mode_transient.h"

#include "../variables.h"
#include "../ui_lcd.h"
#include "../funciones.h"

void legacy_transient_cont_mode() {
  if (!modeConfigured) {
    legacy_transient_cont_setup();
    return;
  }

  if (!modeInitialized) {
    printLCD_S(3, 2, String(LowCurrent, 3));
    writeLCD(byte(0));
    printLCD_S(14, 2, String(HighCurrent, 3));
    writeLCD(byte(0));
    printLCD(0, 0, F("TC LOAD"));
    printLCD(0, 2, F("I1>"));
    printLCD(11, 2, F("I2>"));
    printLCD(2, 3, F("Time: "));

    if (transientPeriod < 10) {
      Print_Spaces(7, 3, 4);
      printLCDRaw(transientPeriod);
    } else if (transientPeriod < 100) {
      Print_Spaces(7, 3, 3);
      printLCDRaw(transientPeriod);
    } else if (transientPeriod < 1000) {
      Print_Spaces(7, 3, 2);
      printLCDRaw(transientPeriod);
    } else if (transientPeriod < 10000) {
      Print_Spaces(7, 3);
      printLCDRaw(transientPeriod);
    } else {
      setCursorLCD(7, 3);
      printLCDRaw(transientPeriod);
    }

    printLCD(13, 3, F("mSecs"));
    setCurrent = 0;
    modeInitialized = true;
    Encoder_Status(false);
  }

  legacy_transcient_cont_timing();
}

void legacy_transient_cont_setup() {
  clearLCD();

  printLCD(3, 0, F("TRANSIENT CONT."));
  printLCD(5, 1, F("I1(A)"));
  printLCD(5, 2, F("I2(A)"));
  printLCD(4, 3, F("dt(mS)"));

  z = 11;
  r = 1;
  if (!Value_Input(z, r)) {
    return;
  }
  LowCurrent = min(x, CurrentCutOff);
  printLCDNumber(z, r, LowCurrent, 'A', 3);

  r = 2;
  if (!Value_Input(z, r)) {
    return;
  }
  HighCurrent = min(x, CurrentCutOff);
  printLCDNumber(z, r, HighCurrent, 'A', 3);

  r = 3;
  if (!Value_Input(z, r, 5, false)) {
    return;
  }
  transientPeriod = x;

  clearLCD();
  modeConfigured = true;
  modeInitialized = false;
}

void legacy_transcient_cont_timing() {
  static unsigned long last_time = 0;
  static bool transient_cont_toggle = false;

  if (!toggle) {
    last_time = 0;
    transient_cont_toggle = false;
    return;
  }

  current_time = micros();

  if ((current_time - last_time) >= (transientPeriod * 1000.0)) {
    last_time = current_time;

    if (!transient_cont_toggle) {
      setCurrent = LowCurrent * 1000;
    } else {
      setCurrent = HighCurrent * 1000;
    }

    transient_cont_toggle = !transient_cont_toggle;
  }
}

void legacy_transient_list_mode() {
  static unsigned int last_transientPeriod = -1;

  if (!modeConfigured) {
    legacy_transient_list_setup();
    return;
  }

  if (!modeInitialized) {
    printLCD(0, 0, F("TL LOAD"));
    printLCD(6, 2, F("Step: "));
    printLCD(13, 2, F("/"));
    printLCD_S(14, 2, String(total_steps));
    printLCD(4, 3, F("dt: "));
    printLCD(13, 3, F("mS"));
    modeInitialized = true;
    Encoder_Status(false);
  }

  if (modeConfigured) {
    printLCD_S(12, 2, String(current_step));
    if (transientPeriod != last_transientPeriod) {
      Print_Spaces(8, 3, 5);
      printLCD_S(8, 3, String(transientPeriod));
      last_transientPeriod = transientPeriod;
    }
  }

  legacy_transient_list_timing();
}

void legacy_transient_list_setup() {
  clearLCD();
  printLCD(3, 0, F("TRANSIENT LIST"));
  printLCD(4, 1, F("Steps(2-10)?"));

  do {
    z = 9;
    r = 2;
    printLCD(z - 1, r, F(">"));
    Print_Spaces(z, r, 2);
    if (!Value_Input(z, r, 2, false)) {
      return;
    }
  } while (x < 2 || x > 10);

  total_steps = x - 1;

  clearLCD();
  for (int i = 0; i <= total_steps; i++) {
    printLCD(3, 0, F("TRANSIENT LIST"));
    printLCD_S(5, 1, "Set step " + String(i));
    printLCD(0, 2, F("Current (A):"));
    printLCD(0, 3, F("Time (mSec):"));

    z = 13;
    r = 2;
    if (!Value_Input(z, r)) {
      return;
    }
    x = min(x, CurrentCutOff);
    printLCDNumber(z, r, x, 'A', 3);
    transientList[i][0] = x * 1000;

    z = 13;
    r = 3;
    if (!Value_Input(z, r, 5, false)) {
      return;
    }
    transientList[i][1] = x;
    clearLCD();
  }

  setCurrent = 0;
  current_step = 0;
  transientPeriod = transientList[current_step][1];
  modeConfigured = true;
  modeInitialized = false;
}

void legacy_transient_list_timing() {
  static unsigned long last_time = 0;

  if (!toggle) {
    current_step = 0;
    last_time = 0;
    transientPeriod = transientList[current_step][1];
    return;
  }

  current_time = micros();

  if (last_time == 0) {
    setCurrent = transientList[current_step][0];
    transientPeriod = transientList[current_step][1];
    last_time = current_time;
  }

  if ((current_time - last_time) >= transientPeriod * 1000) {
    current_step++;
    if (current_step > total_steps) {
      current_step = 0;
    }
    setCurrent = transientList[current_step][0];
    transientPeriod = transientList[current_step][1];
    last_time = current_time;
  }
}
