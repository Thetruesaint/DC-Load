#ifndef FUNCIONES_H
#define FUNCIONES_H

#include <Arduino.h>
#include "ui_lcd.h"
#include "storage_eeprom.h"

/*
void Read_Encoder();
void drawFrame();
void Temp_Control();
void MostrarFechaHora();
void Test_Keypad();
*/

// Declaramos las funciones que vamos a utilizar
void setup();
void loop();
void Load_OFF(void);
void Read_Encoder();
void Read_Keypad(int col =1, int row = 3);
void Read_Load_Button();
void Cursor_Position(void);
void Read_Volts_Current(void);
void DAC_Control(void);
void Config_Limits(void);
bool Value_Input(int col, int row, int maxDigits = 5, bool decimal = true);
void timer_start();
void timer_stop();
void timer_reset();
float timer_getTotalSeconds();
String timer_getTime();
void Show_Limits();
void Check_Limits();
void Temp_Control();
void Reset_Input_Pointers();
void Encoder_Status(bool encOnOff, float limit = 0);
void beepBuzzer(void);
void Calibration_Mode(void);
void Calibration_Setup(void);
void Calibrate(float realValue);
void Test_Keypad();
void Update_TFT(void);
void printTFT_Number(int col, int row, float number, char unit = '\0', int decimals = 2);
#endif




