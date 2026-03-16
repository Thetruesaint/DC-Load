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
  printLCD(5, 0, F("Set Limits"));
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

void ui_draw_config_root_menu(uint8_t selectedIndex) {
  clearLCD();
  printLCD(3, 0, F("Configuration"));
  printLCD(1, 1, F("1-Protection"));
  printLCD(1, 2, F("2-Calibration"));
  printLCD(1, 3, F("3-FW Update 4-Exit"));

  if (selectedIndex == 0) printLCD(0, 1, F(">"));
  if (selectedIndex == 1) printLCD(0, 2, F(">"));
  if (selectedIndex == 2) printLCD(0, 3, F(">"));
  if (selectedIndex == 3) printLCD(12, 3, F(">"));
}

void ui_draw_protection_menu(uint8_t selectedIndex) {
  clearLCD();
  printLCD(4, 0, F("Protection"));
  printLCD(1, 1, F("1-Limits"));
  printLCD(1, 2, F("2-Fan Setting"));
  printLCD(1, 3, F("3-Back"));

  if (selectedIndex <= 2) {
    printLCD(0, 1 + selectedIndex, F(">"));
  }
}

void ui_draw_tests_menu(uint8_t selectedIndex, bool fanOn) {
  clearLCD();
  printLCD(5, 0, F("FW Update"));
  printLCD(1, 1, F("1-Start OTA"));
  printLCD(1, 2, F("2-Back"));
  (void)fanOn;

  if (selectedIndex <= 1) {
    printLCD(0, 1 + selectedIndex, F(">"));
  }
}

void ui_draw_fw_update_screen(const char *statusLine, const char *detailLine, const char *hintLine) {
  clearLCD();
  printLCD(5, 0, F("FW Update"));
  printLCD_S(0, 1, String(statusLine));
  printLCD_S(0, 2, String(detailLine));
  printLCD_S(0, 3, String(hintLine));
}

void ui_draw_fan_settings_menu(uint8_t selectedIndex, float tempC, float holdSeconds, bool editActive, const char *inputText, bool fanOn) {
  clearLCD();
  printLCD(3, 0, F("Fan Setting"));
  printLCD(1, 1, F("1-Temp(C):"));
  printLCD(1, 2, F("2-Time(seg):"));
  printLCD(1, 3, F("3-Fan:"));
  printLCD(8, 3, fanOn ? F("ON ") : F("OFF"));
  printLCD(12, 3, F("4-Back"));

  if (selectedIndex == 0) printLCD(0, 1, F(">"));
  if (selectedIndex == 1) printLCD(0, 2, F(">"));
  if (selectedIndex == 2) printLCD(0, 3, F(">"));
  if (selectedIndex == 3) printLCD(11, 3, F(">"));

  Print_Spaces(12, 1, 3);
  if (selectedIndex == 0 && editActive) {
    printLCD_S(12, 1, String(inputText));
  } else {
    printLCDNumber(12, 1, tempC, ' ', 0);
  }

  Print_Spaces(13, 2, 2);
  if (selectedIndex == 1 && editActive) {
    printLCD_S(13, 2, String(inputText));
  } else {
    printLCDNumber(13, 2, holdSeconds, ' ', 0);
  }
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

void ui_draw_calibration_result(bool voltageMode, float sensorFactor, float sensorOffset, float outputFactor, float outputOffset) {
  clearLCD();
  printLCD(0, 0, voltageMode ? F("V Calibrated:") : F("I Calibrated:"));

  if (voltageMode) {
    printLCD(0, 1, F("SF:"));
    Print_Spaces(3, 1, 8);
    printLCD_S(3, 1, String(sensorFactor, 6));
    printLCD(0, 2, F("SO:"));
    Print_Spaces(3, 2, 8);
    printLCD_S(3, 2, String(sensorOffset, 6));
  } else {
    printLCD(0, 1, F("SF:"));
    Print_Spaces(3, 1, 7);
    printLCD_S(3, 1, String(sensorFactor, 4));
    printLCD(10, 1, F("SO:"));
    Print_Spaces(13, 1, 7);
    printLCD_S(13, 1, String(sensorOffset, 3));
    printLCD(0, 2, F("OF:"));
    Print_Spaces(3, 2, 7);
    printLCD_S(3, 2, String(outputFactor, 4));
    printLCD(10, 2, F("OO:"));
    Print_Spaces(13, 2, 7);
    printLCD_S(13, 2, String(outputOffset, 0));
  }

  printLCD(2, 3, F("E-OK  <=Cancel"));
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

void ui_draw_protection_modal(const char *message, char causeCode) {
  clearLCD();
  printLCD(0, 0, F("*** PROTECTION ***"));
  printLCD_S(0, 1, String(message));
  printLCD_S(0, 2, String(F("Cause: ")) + String(causeCode));
  printLCD(0, 3, F("Load: OFF  E=OK"));
}

void ui_set_setpoint_cursor(int cursorColumn) {
  setCursorLCD(cursorColumn, 2);
}

void ui_update_battery_life(float batteryLife) {
  printLCDNumber(6, 3, batteryLife, ' ', 0);
  printLCDRaw(F("mAh"));
  Print_Spaces(16, 2, 4);
}

void ui_show_battery_done() {
  printLCD(16, 2, F("Done"));
}

void ui_update_battery_timer(const String &timeText) {
  printLCD_S(0, 3, timeText);
}

void ui_update_transient_list_step(int step) {
  printLCD_S(12, 2, String(step));
}

void ui_update_transient_list_period(unsigned long periodMs) {
  Print_Spaces(8, 3, 5);
  printLCD_S(8, 3, String(periodMs));
}

void ui_prepare_value_input_prompt(int col, int row, int width) {
  printLCD(col - 1, row, F(">"));
  Print_Spaces(col, row, width);
}

void ui_show_value_number(int col, int row, float value, char unit, int decimals) {
  printLCDNumber(col, row, value, unit, decimals);
}

void ui_show_value_text(int col, int row, const String &text) {
  printLCD_S(col, row, text);
}

void ui_show_current_limit_value(int col, int row, float current) {
  printLCDNumber(col, row, current, ' ', 3);
  printLCDRaw(F("A"));
}

void ui_clear_mode_screen() {
  clearLCD();
}








