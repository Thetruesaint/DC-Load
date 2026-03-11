#include <Arduino.h>

#include "app/app_inputs.h"
#include "app/app_keypad.h"
#include "app/app_loop.h"
#include "app/app_value_input.h"
#include "legacy/legacy_base_io.h"
#include "legacy/legacy_dac_control.h"
#include "legacy/legacy_mode_ca.h"
#include "legacy/legacy_mode_limits.h"
#include "legacy/legacy_mode_transient.h"
#include "legacy/legacy_safety_control.h"
#include "legacy/legacy_timing_buzzer.h"

void Load_OFF(void) {
  legacy_load_off();
}

void Encoder_Status(bool encOnOff, float limit) {
  legacy_encoder_status(encOnOff, limit);
}

void Read_Encoder() {
  legacy_read_encoder();
}

void Read_Keypad(int col, int row) {
  app_read_keypad(col, row);
}

void Read_Load_Button(void) {
  app_read_load_button();
}

bool Value_Input(int col, int row, int maxDigits, bool decimal) {
  return app_value_input(col, row, maxDigits, decimal);
}

void Temp_Control(void) {
  legacy_temp_control();
}

void Check_Limits() {
  legacy_check_limits();
}

void Cursor_Position(void) {
  legacy_cursor_position();
}

void Read_Volts_Current(void) {
  legacy_read_volts_current();
}

void DAC_Control(void) {
  legacy_dac_control();
}

void timer_start() {
  legacy_timer_start();
}

void timer_stop() {
  legacy_timer_stop();
}

void timer_reset() {
  legacy_timer_reset();
}

float timer_getTotalSeconds() {
  return legacy_timer_get_total_seconds();
}

String timer_getTime() {
  return legacy_timer_get_time();
}

void Reset_Input_Pointers() {
  app_reset_input_pointers();
}

void Transient_Cont_Mode(void) {
  legacy_transient_cont_mode();
}

void Transient_Cont_Setup(void) {
  legacy_transient_cont_setup();
}

void Transcient_Cont_Timing() {
  legacy_transcient_cont_timing();
}

void Transient_List_Mode(void) {
  legacy_transient_list_mode();
}

void Transient_List_Setup() {
  legacy_transient_list_setup();
}

void Transient_List_Timing(void) {
  legacy_transient_list_timing();
}

void Config_Limits(void) {
  legacy_config_limits();
}

void Show_Limits(void) {
  legacy_show_limits();
}

void beepBuzzer(void) {
  legacy_beep_buzzer();
}

void Calibration_Mode(void) {
  legacy_calibration_mode();
}

void Calibration_Setup(void) {
  legacy_calibration_setup();
}

void Calibrate(float realValue) {
  legacy_calibrate(realValue);
}
