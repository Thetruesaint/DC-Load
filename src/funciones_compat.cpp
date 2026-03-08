#include "variables.h"
#include "funciones.h"
#include "app/app_loop.h"
#include "app/app_keypad.h"
#include "app/app_inputs.h"
#include "app/app_value_input.h"
#include "legacy/legacy_mode_cc.h"
#include "legacy/legacy_mode_cp.h"
#include "legacy/legacy_mode_cr.h"
#include "legacy/legacy_mode_bc.h"
#include "legacy/legacy_mode_ca.h"
#include "legacy/legacy_mode_transient.h"
#include "legacy/legacy_mode_limits.h"
#include "legacy/legacy_timing_buzzer.h"
#include "legacy/legacy_base_io.h"
#include "legacy/legacy_safety_control.h"
#include "legacy/legacy_dac_control.h"

//----------------------------- Load ON Status ------------------------------------
void Load_OFF(void) {
  legacy_load_off();
}

//---------------------------- Encoder Status -------------------------------------
void Encoder_Status(bool encOnOff, float limit) {
  legacy_encoder_status(encOnOff, limit);
}

//---------------------------- Encoder Decoder ------------------------------------
void Read_Encoder() {
  legacy_read_encoder();
}

//---------------------------- Read Keypad Input ----------------------------------
void Read_Keypad(int col, int row) {
  app_read_keypad(col, row);
}

//----------------------- Toggle Current Load ON or OFF ----------------------------
void Read_Load_Button(void) {
  app_read_load_button();
}
//----------------------- Key input used for UserSetUp ------------------------------- 
bool Value_Input(int col, int row, int maxDigits, bool decimal) {
  return app_value_input(col, row, maxDigits, decimal);
}

//---------------------------- Temperature Control ----------------------------------
void Temp_Control(void) {
  legacy_temp_control();
}

//--------------------------- Check and Enforce Limits ------------------------------
void Check_Limits() {
  legacy_check_limits();
}

//------------------------------- Cursor Position -----------------------------------
void Cursor_Position(void) {
  legacy_cursor_position();
}

//--------------------------- Read Voltage and Current ------------------------------
void Read_Volts_Current(void) {
  legacy_read_volts_current();
}

//------------------------- DAC Control Voltage for Mosfet --------------------------
void DAC_Control(void) {
  legacy_dac_control();
}

//----------------------- Select Constant Current LCD set up ------------------------
void Const_Current_Mode(void) {
  legacy_const_current_mode();
}

//------------------------ Select Constant Power LCD set up -------------------------
void Const_Power_Mode(void) {
  legacy_const_power_mode();
}

//---------------------- Select Constant Resistance LCD set up ----------------------
void Const_Resistance_Mode(void) {
  legacy_const_resistance_mode();
}

//-------------------- Select Battery Capacity Testing LCD set up -------------------
void Battery_Mode(void) {
  legacy_battery_mode();
}

//-------------------- Battery Type Selection and Cutoff Setup ----------------------
void Battery_Type_Selec() {
  legacy_battery_type_selec();
}

//---------------------- Battery Capacity Discharge Routine -------------------------
bool Battery_Capacity() {
  return legacy_battery_capacity();
}

//---------------------------- Transcient Continuos Mode ----------------------------
void Transient_Cont_Mode(void) {
  legacy_transient_cont_mode();
}

//--------------------------------- Transient Mode ----------------------------------
void Transient_Cont_Setup(void) {
  legacy_transient_cont_setup();
}

//----------------------------- Transcient Continuos Timing -------------------------
void Transcient_Cont_Timing() {
  legacy_transcient_cont_timing();
}

//------------------------------ Transcient List Mode -------------------------------
void Transient_List_Mode(void) {
  legacy_transient_list_mode();
}

//------------------------------ Transcient List Setup -------------------------------
void Transient_List_Setup() {
  legacy_transient_list_setup();
}

//------------------------------ Transcient List Timing ------------------------------
void Transient_List_Timing(void) {
  legacy_transient_list_timing();
}

//------------------------------ User set up for limits ------------------------------
void Config_Limits(void)
{
  legacy_config_limits();
}

//----------------- Show limits Stored Data for Current, Power and Temp --------------
void Show_Limits(void) {
  legacy_show_limits();
}

//--------------------------------- Calibration Mode ---------------------------------
void Calibration_Mode() {
  legacy_calibration_mode();
}

//--------------------------------- Calibration Setup --------------------------------
void Calibration_Setup(void){
  legacy_calibration_setup();
}

//--------------------------------- Calibrate ---------------------------------------
void Calibrate(float realValue){
  legacy_calibrate(realValue);
}

//------------------------------- Reset Input Pointers --------------------------------
void Reset_Input_Pointers (void){
  app_reset_input_pointers();
}

//------------------------------- Handle Buzzer --------------------------------------
void beepBuzzer(void) {
  legacy_beep_buzzer();
}

//-------------------------- Funciones para el Timer (RTC) ---------------------------
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



















