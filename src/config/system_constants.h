#ifndef SYSTEM_CONSTANTS_H
#define SYSTEM_CONSTANTS_H

#include <Arduino.h>

enum ModeType { CC, CP, CR, BC, TC, TL, CA, UNKNOWN };

//---------------- I/O Pins ----------------------------------------------
const uint8_t TEMP_SNSR = 36;
const uint8_t FAN_CTRL = 16;
const uint8_t BUZZER = 17;
const uint8_t ENC_A = 14;
const uint8_t ENC_B = 13;
const uint8_t ENC_BTN = 32;
const uint8_t LOADONOFF = 39;
const uint8_t MOSFONOFF = 25;
#ifndef WOKWI_SIMULATION
const uint8_t CRR_SNSR = 1;
const uint8_t VLTG_SNSR = 3;
#endif

//--------------- Operating constants ------------------------------------
const float MAX_VOLTAGE = 33.00f;
const float MAX_RESISTOR = 999.9f;
const float MAX_CURRENT = 10.000f;
const float MAX_POWER = 300.0f;
const float MAX_TEMP = 99.0f;

const float SNS_VOLT_FACT = 10.20408f;
const float SNS_CURR_FACT = 4.0f;
const float OUT_CURR_FACT = 0.3375f;

const unsigned long LCD_RFSH_TIME = 100;
#ifdef WOKWI_SIMULATION
const float SIM_DEFAULT_VOLTAGE = 12.0f;
#endif

const float LIPO_DISC_CELL_VLTG = 3.6f;
const float LION_DISC_CELL_VLTG = 3.5f;
const float LIPO_STOR_CELL_VLTG = 3.8f;
const float LION_STOR_CELL_VLTG = 3.7f;
const unsigned long CRR_STEP_RDCTN = 10;
const float VLTG_DROP_MARGIN = 0.02f;
const float MIN_DISC_CURR = 100.0f;

const int MIN_FAN_TEMP_ON_C = 30;
const int MAX_FAN_TEMP_ON_C = 90;
const int DEFAULT_FAN_TEMP_ON_C = 40;
const unsigned long MIN_FAN_HOLD_MS = 10000UL;
const unsigned long MAX_FAN_HOLD_MS = 90000UL;
const unsigned long DEFAULT_FAN_HOLD_MS = 60000UL;
const int TMP_CHK_TIME = 1000;

const float CAL_MIN_VOLTAGE_DELTA = 10.0f;
const float CAL_MIN_CURRENT_DELTA = 4.0f;
const float CAL_MAX_POINT_ERROR_RATIO = 0.20f;

#ifndef WOKWI_SIMULATION
#define TEMP_CONVERSION_FACTOR 0.02686202686202686f
#else
#define TEMP_CONVERSION_FACTOR 0.0244140625f
#endif

//--- EEPROM map ----------------------------------------------------------
const int ADD_CURRENT_CUT_OFF = 0;
const int ADD_POWER_CUT_OFF = 4;
const int ADD_TEMP_CUT_OFF = 8;
const int ADD_SNS_VOLT_FAC_CAL = 12;
const int ADD_SNS_CURR_FAC_CAL = 16;
const int ADD_OUT_CURR_FAC_CAL = 20;
const int ADD_SNS_VOLT_OFF_CAL = 24;
const int ADD_SNS_CURR_OFF_CAL = 28;
const int ADD_OUT_CURR_OFF_CAL = 32;
const int ADD_FAN_TEMP_ON = 36;
const int ADD_FAN_HOLD_MS = 40;

#endif

