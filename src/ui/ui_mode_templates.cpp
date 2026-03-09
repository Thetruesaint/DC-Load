#include "ui_mode_templates.h"

#include "../ui_lcd.h"

void ui_draw_cc_template() {
  clearLCD();
  printLCD(0, 0, F("CC LOAD"));
  printLCD(1, 2, F("Set->"));
  printLCD(13, 2, F("A"));
  printLCD(0, 3, F(">"));
}

void ui_draw_cp_template() {
  clearLCD();
  printLCD(0, 0, F("CP LOAD"));
  printLCD(0, 2, F("Set->"));
  printLCD(11, 2, F("W"));
  printLCD(0, 3, F(">"));
}

void ui_draw_cr_template() {
  clearLCD();
  printLCD(0, 0, F("CR LOAD"));
  printLCD(0, 2, F("Set->"));
  printLCD_S(11, 2, String((char)0xF4));
  printLCD(0, 3, F(">"));
}

void ui_draw_bc_template(float cutoffVolts, float batteryLife, const String &batteryType) {
  clearLCD();
  printLCD(0, 0, F("BC LOAD"));
  setCursorLCD(13, 1);
  printLCDRaw(F(">"));
  printLCDNumber(14, 1, cutoffVolts, 'V', 2);
  printLCD(1, 2, F("Adj->"));
  printLCD(13, 2, F("A"));
  printLCDNumber(6, 3, batteryLife, ' ', 0);
  printLCDRaw(F("mAh"));
  printLCD_S(14, 3, batteryType);
}

void ui_draw_battery_task_menu() {
  clearLCD();
  printLCD(2, 0, F("Set Task & Batt"));
  printLCD(0, 1, F("Stor. 1)LiPo 2)LiIOn"));
  printLCD(0, 2, F("Disc. 3)LiPo 4)LiIOn"));
  printLCD(2, 3, F("5)Cutoff Voltage"));
}

void ui_draw_battery_custom_cutoff_prompt(const String &batteryType) {
  clearLCD();
  printLCD_S(3, 0, batteryType + " Batt");
  printLCD(2, 1, F("Voltage Cutoff?"));
  printLCD(5, 2, F("(0.1-25)V"));
}

void ui_draw_battery_cell_count_prompt(const String &batteryType) {
  clearLCD();
  printLCD_S(3, 0, batteryType + " Batt");
  printLCD(6, 1, F("(1-6)S?"));
}

void ui_draw_limits_config_template() {
  clearLCD();
  printLCD(4, 0, F("Set Limits"));
  printLCD(0, 1, F("Current(A):"));
  printLCD(0, 2, F("Power(W):"));
  printLCD(0, 3, F("Temp.("));
  printLCD_S(6, 3, String((char)0xDF) + "C):");
}

void ui_draw_limits_summary(float currentCutoff, float powerCutoff, float tempCutoff) {
  clearLCD();
  printLCD(1, 0, F("Limits"));
  printLCD(0, 1, F("Current:"));
  printLCDNumber(9, 1, currentCutoff, ' ', 3);
  printLCDRaw(F("A"));
  printLCD(0, 2, F("Power:"));
  printLCDNumber(9, 2, powerCutoff, 'W', 2);
  printLCD(0, 3, F("Temp.:"));
  printLCDNumber(9, 3, tempCutoff, ' ', 0);
  printLCDRaw(char(0xDF));
  printLCDRaw("C");
}

void ui_draw_calibration_setup_menu() {
  clearLCD();
  printLCD(4, 0, F("CALIBRATION"));
  printLCD(0, 1, F("1-Voltage 2-Current"));
  printLCD(0, 2, F("3-Load    4-Save"));
}

void ui_draw_calibration_mode_template(bool voltageMode, bool firstPointTaken) {
  clearLCD();
  printLCD(0, 0, voltageMode ? F("CA VOLT") : F("CA CURR"));
  printLCD(14, 1, firstPointTaken ? F("Set P2") : F("Set P1"));
  printLCD(1, 2, F("Adj->"));
  printLCD(13, 2, F("A"));
  printLCD(0, 3, F(">"));
  printLCD(7, 3, F("<Set real"));
}

void ui_draw_calibration_abort(bool pointsTooClose) {
  clearLCD();
  printLCD(0, 1, F("Calib Abort"));
  if (pointsTooClose) {
    printLCD(0, 2, F("P1/P2 too close"));
  } else {
    printLCD(0, 2, F("Set/Read >20%"));
  }
}

void ui_draw_calibration_success() {
  clearLCD();
  printLCD(4, 1, F("Calibrated!"));
}

void ui_draw_calibration_loaded_message() {
  printLCD(12, 3, F("Loaded!"));
}

void ui_draw_calibration_saved_message() {
  printLCD(12, 3, F("Saved!"));
}

void ui_draw_transient_cont_mode_template(float lowCurrent, float highCurrent, unsigned long periodMs) {
  clearLCD();
  printLCD(0, 0, F("TC LOAD"));
  printLCD(0, 2, F("I1>"));
  printLCD(11, 2, F("I2>"));
  printLCD(2, 3, F("Time: "));
  printLCD(13, 3, F("mSecs"));

  printLCD_S(3, 2, String(lowCurrent, 3));
  writeLCD(byte(0));
  printLCD_S(14, 2, String(highCurrent, 3));
  writeLCD(byte(0));

  printLCD_S(7, 3, String(periodMs));
}

void ui_draw_transient_cont_setup_template() {
  clearLCD();
  printLCD(3, 0, F("TRANSIENT CONT."));
  printLCD(5, 1, F("I1(A)"));
  printLCD(5, 2, F("I2(A)"));
  printLCD(4, 3, F("dt(mS)"));
}

void ui_draw_transient_list_mode_template(int totalSteps) {
  clearLCD();
  printLCD(0, 0, F("TL LOAD"));
  printLCD(6, 2, F("Step: "));
  printLCD(13, 2, F("/"));
  printLCD_S(14, 2, String(totalSteps));
  printLCD(4, 3, F("dt: "));
  printLCD(13, 3, F("mS"));
}

void ui_draw_transient_list_setup_template() {
  clearLCD();
  printLCD(3, 0, F("TRANSIENT LIST"));
  printLCD(4, 1, F("Steps(2-10)?"));
}

void ui_draw_transient_list_step_template(int stepIndex) {
  clearLCD();
  printLCD(3, 0, F("TRANSIENT LIST"));
  printLCD_S(5, 1, "Set step " + String(stepIndex));
  printLCD(0, 2, F("Current (A):"));
  printLCD(0, 3, F("Time (mSec):"));
}

void ui_draw_header_temperature(int tempC) {
  setCursorLCD(16, 0);
  if (tempC < 10) {
    printLCDRaw(" ");
  }
  printLCDRaw(tempC);
  printLCDRaw(char(0xDF));
  printLCDRaw("C");
}

void ui_blink_limit_alarm(const char *message, bool vlimit, bool ilimit, bool plimit, bool climit) {
  for (int i = 0; i < 6; i++) {
    printLCD_S(0, 3, message);
    if (vlimit) {
      Print_Spaces(12, 1);
    } else if (ilimit) {
      Print_Spaces(5, 1);
    } else if (plimit) {
      Print_Spaces(19, 1);
    } else if (climit) {
      Print_Spaces(19, 0);
    }
    delay(250);

    Print_Spaces(0, 3, 18);
    if (vlimit) {
      setCursorLCD(12, 1);
      printLCDRaw(F("v"));
    } else if (ilimit) {
      setCursorLCD(5, 1);
      writeLCD(byte(0));
    } else if (plimit) {
      setCursorLCD(19, 1);
      printLCDRaw(F("w"));
    } else if (climit) {
      setCursorLCD(19, 0);
      printLCDRaw(F("C"));
    }
    delay(250);
  }
}

void ui_set_setpoint_cursor(int cursorColumn) {
  setCursorLCD(cursorColumn, 2);
}
