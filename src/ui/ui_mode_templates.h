#ifndef UI_MODE_TEMPLATES_H
#define UI_MODE_TEMPLATES_H

#include <Arduino.h>

void ui_draw_cc_template();
void ui_draw_cp_template();
void ui_draw_cr_template();
void ui_draw_bc_template(float cutoffVolts, float batteryLife, const String &batteryType);
void ui_draw_battery_task_menu();
void ui_draw_battery_custom_cutoff_prompt(const String &batteryType);
void ui_draw_battery_cell_count_prompt(const String &batteryType);
void ui_draw_limits_config_template();
void ui_draw_limits_summary(float currentCutoff, float powerCutoff, float tempCutoff);
void ui_draw_config_root_menu(uint8_t selectedIndex);
void ui_draw_protection_menu(uint8_t selectedIndex);
void ui_draw_tests_menu(uint8_t selectedIndex, bool fanOn);
void ui_draw_fan_settings_menu(uint8_t selectedIndex, float tempC, float holdSeconds, bool editActive, const char *inputText);
void ui_draw_calibration_setup_menu();
void ui_draw_calibration_mode_template(bool voltageMode, bool firstPointTaken);
void ui_draw_calibration_abort(bool pointsTooClose);
void ui_draw_calibration_result(bool voltageMode, float sensorFactor, float sensorOffset, float outputFactor, float outputOffset);
void ui_draw_calibration_loaded_message();
void ui_draw_calibration_saved_message();
void ui_draw_transient_cont_mode_template(float lowCurrent, float highCurrent, unsigned long periodMs);
void ui_draw_transient_cont_setup_template();
void ui_draw_transient_list_mode_template(int totalSteps);
void ui_draw_transient_list_setup_template();
void ui_draw_transient_list_step_template(int stepIndex);
void ui_draw_header_temperature(int tempC);
void ui_draw_protection_modal(const char *message, char causeCode);
void ui_set_setpoint_cursor(int cursorColumn);
void ui_update_battery_life(float batteryLife);
void ui_show_battery_done();
void ui_update_battery_timer(const String &timeText);
void ui_update_transient_list_step(int step);
void ui_update_transient_list_period(unsigned long periodMs);
void ui_prepare_value_input_prompt(int col, int row, int width);
void ui_show_value_number(int col, int row, float value, char unit, int decimals);
void ui_show_value_text(int col, int row, const String &text);
void ui_show_current_limit_value(int col, int row, float current);
void ui_clear_mode_screen();

#endif

