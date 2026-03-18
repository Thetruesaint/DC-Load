#include "ui_mode_templates.h"

#include "../ui_display.h"

void ui_draw_cc_template() {
  uiDisplayClear();
  uiGridPrint(0, 0, F("CC LOAD"));
  uiGridPrint(1, 2, F("Set->"));
  uiGridPrint(13, 2, F("A"));
  uiGridPrint(0, 3, F(">"));
}

void ui_draw_cp_template() {
  uiDisplayClear();
  uiGridPrint(0, 0, F("CP LOAD"));
  uiGridPrint(0, 2, F("Set->"));
  uiGridPrint(11, 2, F("W"));
  uiGridPrint(0, 3, F(">"));
}

void ui_draw_cr_template() {
  uiDisplayClear();
  uiGridPrint(0, 0, F("CR LOAD"));
  uiGridPrint(0, 2, F("Set->"));
  uiGridPrintString(11, 2, String((char)0xF4));
  uiGridPrint(0, 3, F(">"));
}

void ui_draw_bc_template(float cutoffVolts, float batteryLife, const String &batteryType) {
  uiDisplayClear();
  uiGridPrint(0, 0, F("BC LOAD"));
  uiGridSetCursor(13, 1);
  printLCDRaw(F(">"));
  uiGridPrintNumber(14, 1, cutoffVolts, 'V', 2);
  uiGridPrint(1, 2, F("Adj->"));
  uiGridPrint(13, 2, F("A"));
  uiGridPrintNumber(6, 3, batteryLife, ' ', 0);
  printLCDRaw(F("mAh"));
  uiGridPrintString(14, 3, batteryType);
}

void ui_draw_battery_task_menu() {
  uiDisplayClear();
  uiGridPrint(2, 0, F("Set Task & Batt"));
  uiGridPrint(0, 1, F("Stor. 1)LiPo 2)LiIOn"));
  uiGridPrint(0, 2, F("Disc. 3)LiPo 4)LiIOn"));
  uiGridPrint(2, 3, F("5)Cutoff Voltage"));
}

void ui_draw_battery_custom_cutoff_prompt(const String &batteryType) {
  uiDisplayClear();
  uiGridPrintString(3, 0, batteryType + " Batt");
  uiGridPrint(2, 1, F("Voltage Cutoff?"));
  uiGridPrint(5, 2, F("(0.1-25)V"));
}

void ui_draw_battery_cell_count_prompt(const String &batteryType) {
  uiDisplayClear();
  uiGridPrintString(3, 0, batteryType + " Batt");
  uiGridPrint(6, 1, F("(1-6)S?"));
}

void ui_draw_limits_config_template() {
  uiDisplayClear();
  uiGridPrint(5, 0, F("Set Limits"));
  uiGridPrint(0, 1, F("Current(A):"));
  uiGridPrint(0, 2, F("Power(W):"));
  uiGridPrint(0, 3, F("Temp.("));
  uiGridPrintString(6, 3, String((char)0xDF) + "C):");
}

void ui_draw_limits_summary(float currentCutoff, float powerCutoff, float tempCutoff) {
  uiDisplayClear();
  uiGridPrint(1, 0, F("Limits"));
  uiGridPrint(0, 1, F("Current:"));
  uiGridPrintNumber(9, 1, currentCutoff, ' ', 3);
  printLCDRaw(F("A"));
  uiGridPrint(0, 2, F("Power:"));
  uiGridPrintNumber(9, 2, powerCutoff, 'W', 2);
  uiGridPrint(0, 3, F("Temp.:"));
  uiGridPrintNumber(9, 3, tempCutoff, ' ', 0);
  printLCDRaw(char(0xDF));
  printLCDRaw("C");
}

void ui_draw_config_root_menu(uint8_t selectedIndex) {
  uiDisplayClear();
  uiGridPrint(3, 0, F("Configuration"));
  uiGridPrint(1, 1, F("1-Protection"));
  uiGridPrint(1, 2, F("2-Calibration"));
  uiGridPrint(1, 3, F("3-FW Update 4-Exit"));

  if (selectedIndex == 0) uiGridPrint(0, 1, F(">"));
  if (selectedIndex == 1) uiGridPrint(0, 2, F(">"));
  if (selectedIndex == 2) uiGridPrint(0, 3, F(">"));
  if (selectedIndex == 3) uiGridPrint(12, 3, F(">"));
}

void ui_draw_protection_menu(uint8_t selectedIndex) {
  uiDisplayClear();
  uiGridPrint(4, 0, F("Protection"));
  uiGridPrint(1, 1, F("1-Limits"));
  uiGridPrint(1, 2, F("2-Fan Setting"));
  uiGridPrint(1, 3, F("3-Back"));

  if (selectedIndex <= 2) {
    uiGridPrint(0, 1 + selectedIndex, F(">"));
  }
}

void ui_draw_tests_menu(uint8_t selectedIndex, bool fanOn) {
  uiDisplayClear();
  uiGridPrint(5, 0, F("FW Update"));
  uiGridPrint(1, 1, F("1-Start OTA"));
  uiGridPrint(1, 2, F("2-Back"));
  (void)fanOn;

  if (selectedIndex <= 1) {
    uiGridPrint(0, 1 + selectedIndex, F(">"));
  }
}

void ui_draw_fw_update_screen(const char *statusLine, const char *detailLine, const char *hintLine) {
  uiDisplayClear();
  uiGridPrint(5, 0, F("FW Update"));
  uiGridPrintString(0, 1, String(statusLine));
  uiGridPrintString(0, 2, String(detailLine));
  uiGridPrintString(0, 3, String(hintLine));
}

void ui_draw_fan_settings_menu(uint8_t selectedIndex, float tempC, float holdSeconds, bool editActive, const char *inputText, bool fanOn) {
  uiDisplayClear();
  uiGridPrint(3, 0, F("Fan Setting"));
  uiGridPrint(1, 1, F("1-Temp(C):"));
  uiGridPrint(1, 2, F("2-Time(seg):"));
  uiGridPrint(1, 3, F("3-Fan:"));
  uiGridPrint(8, 3, fanOn ? F("ON ") : F("OFF"));
  uiGridPrint(12, 3, F("4-Back"));

  if (selectedIndex == 0) uiGridPrint(0, 1, F(">"));
  if (selectedIndex == 1) uiGridPrint(0, 2, F(">"));
  if (selectedIndex == 2) uiGridPrint(0, 3, F(">"));
  if (selectedIndex == 3) uiGridPrint(11, 3, F(">"));

  uiClearCells(12, 1, 3);
  if (selectedIndex == 0 && editActive) {
    uiGridPrintString(12, 1, String(inputText));
  } else {
    uiGridPrintNumber(12, 1, tempC, ' ', 0);
  }

  uiClearCells(13, 2, 2);
  if (selectedIndex == 1 && editActive) {
    uiGridPrintString(13, 2, String(inputText));
  } else {
    uiGridPrintNumber(13, 2, holdSeconds, ' ', 0);
  }
}

void ui_draw_calibration_setup_menu() {
  uiDisplayRenderCalibrationSetupMenu("");
}

void ui_draw_calibration_mode_template(bool voltageMode, bool firstPointTaken) {
  (void)voltageMode;
  (void)firstPointTaken;
  uiDisplayInvalidateHomeLayout();
}

void ui_draw_calibration_abort(bool pointsTooClose) {
  uiDisplayRenderCalibrationAbortScreen(pointsTooClose);
}

void ui_draw_calibration_success() {
  uiDisplayClear();
  uiGridPrint(4, 1, F("Calibrated!"));
}

void ui_draw_calibration_result(bool voltageMode, float sensorFactor, float sensorOffset, float outputFactor, float outputOffset) {
  uiDisplayRenderCalibrationResultScreen(voltageMode, sensorFactor, sensorOffset, outputFactor, outputOffset);
}

void ui_draw_calibration_loaded_message() {
  uiDisplayRenderCalibrationNoticeScreen("CALIBRATION", "Loaded");
}

void ui_draw_calibration_saved_message() {
  uiDisplayRenderCalibrationNoticeScreen("CALIBRATION", "Saved");
}

void ui_draw_transient_cont_mode_template(float lowCurrent, float highCurrent, unsigned long periodMs) {
  uiDisplayClear();
  uiGridPrint(0, 0, F("TC LOAD"));
  uiGridPrint(0, 2, F("I1>"));
  uiGridPrint(11, 2, F("I2>"));
  uiGridPrint(2, 3, F("Time: "));
  uiGridPrint(13, 3, F("mSecs"));

  uiGridPrintString(3, 2, String(lowCurrent, 3));
  printLCDRaw('A');
  uiGridPrintString(14, 2, String(highCurrent, 3));
  printLCDRaw('A');
  uiGridPrintString(7, 3, String(periodMs));
}

void ui_draw_transient_cont_setup_template() {
  uiDisplayClear();
  uiGridPrint(3, 0, F("TRANSIENT CONT."));
  uiGridPrint(5, 1, F("I1(A)"));
  uiGridPrint(5, 2, F("I2(A)"));
  uiGridPrint(4, 3, F("dt(mS)"));
}

void ui_draw_transient_list_mode_template(int totalSteps) {
  uiDisplayClear();
  uiGridPrint(0, 0, F("TL LOAD"));
  uiGridPrint(6, 2, F("Step: "));
  uiGridPrint(13, 2, F("/"));
  uiGridPrintString(14, 2, String(totalSteps + 1));
  uiGridPrint(4, 3, F("dt: "));
  uiGridPrint(13, 3, F("mS"));
}

void ui_draw_transient_list_setup_template() {
  uiDisplayClear();
  uiGridPrint(3, 0, F("TRANSIENT LIST"));
  uiGridPrint(4, 1, F("Steps(2-10)?"));
}

void ui_draw_transient_list_step_template(int stepIndex) {
  uiDisplayClear();
  uiGridPrint(3, 0, F("TRANSIENT LIST"));
  uiGridPrintString(5, 1, "Set step " + String(stepIndex + 1));
  uiGridPrint(0, 2, F("Current (A):"));
  uiGridPrint(0, 3, F("Time (mSec):"));
}

void ui_draw_header_temperature(int tempC) {
  uiGridSetCursor(16, 0);
  if (tempC < 10) {
    printLCDRaw(" ");
  }
  printLCDRaw(tempC);
  printLCDRaw(char(0xDF));
  printLCDRaw("C");
}

void ui_draw_protection_modal(const char *message, char causeCode) {
  uiDisplayRenderProtectionModal(message, causeCode);
}

void ui_set_setpoint_cursor(int cursorColumn) {
  uiGridSetCursor(cursorColumn, 2);
}

void ui_update_battery_life(float batteryLife) {
  uiGridPrintNumber(6, 3, batteryLife, ' ', 0);
  printLCDRaw(F("mAh"));
  uiClearCells(16, 2, 4);
}

void ui_show_battery_done() {
  uiGridPrint(16, 2, F("Done"));
}

void ui_update_battery_timer(const String &timeText) {
  uiGridPrintString(0, 3, timeText);
}

void ui_update_transient_list_step(int step) {
  uiGridPrintString(12, 2, String(step + 1));
}

void ui_update_transient_list_period(unsigned long periodMs) {
  uiClearCells(8, 3, 5);
  uiGridPrintString(8, 3, String(periodMs));
}

void ui_prepare_value_input_prompt(int col, int row, int width) {
  uiGridPrint(col - 1, row, F(">"));
  uiClearCells(col, row, width);
}

void ui_show_value_number(int col, int row, float value, char unit, int decimals) {
  uiGridPrintNumber(col, row, value, unit, decimals);
}

void ui_show_value_text(int col, int row, const String &text) {
  uiGridPrintString(col, row, text);
}

void ui_show_current_limit_value(int col, int row, float current) {
  uiGridPrintNumber(col, row, current, ' ', 3);
  printLCDRaw(F("A"));
}

void ui_clear_mode_screen() {
  uiDisplayClear();
}








