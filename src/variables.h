#ifndef VARIABLES_H
#define VARIABLES_H

#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#ifndef WOKWI_SIMULATION
#include <Adafruit_ADS1X15.h>
#include <Adafruit_MCP4725.h>
#endif
#include <TFT_eSPI.h>
#include <SPI.h>
#include <RTClib.h>
#include <Keypad.h>
#include <ESP32Encoder.h>
#include <EEPROM.h>

// Objetos para el hardware
#ifndef WOKWI_SIMULATION
extern Adafruit_MCP4725 dac; 
extern Adafruit_ADS1115 ads;
#endif
extern LiquidCrystal_I2C lcd;
extern TFT_eSPI tft;
extern RTC_DS1307 rtc;
extern ESP32Encoder encoder;

//---------------- I/O Pins ----------------------------------------------

const uint8_t TEMP_SNSR = 36;  // Sensado de Temperatura
const uint8_t FAN_CTRL = 25;   // Endendido de Fans
const uint8_t BUZZER = 17;     // Control del Buzzer
const uint8_t ENC_A = 14;      // Encoder Pin A
const uint8_t ENC_B = 13;      // Encoder Pin B
const uint8_t ENC_BTN = 32;    // Encoder Boton
const uint8_t LOADONOFF = 16;  // Input A1 used as a digital pin to set Load ON/OFF
#ifdef WOKWI_SIMULATION
const uint8_t VSIM = 4;        // Pin para simular el voltaje de carga con un potenciómetro en la simulación. Ajusta el pin según tu conexión.
#else
const uint8_t CRR_SNSR = 1;    // Input A1 from ADC (real hardware ADS1115)
const uint8_t VLTG_SNSR = 3;   // Input A3 from ADC (real hardware ADS1115)
#endif

//--------------- Variables Encoder --------------------------------------

extern unsigned long lastButtonPress;     // Use this to store if the encoder button was pressed or not

//--------------- Variables de operacion --------------------------------- 

const float MAX_VOLTAGE = 33.00;           // Vltg sns hasta 30V limite del LM741
const float MAX_RESISTOR = 999.9;          // Máximo 999.9 Ω
const float MAX_CURRENT = 10.000;          // Máximo 10A
const float MAX_POWER = 300.0;             // Máximo 300W
const float MAX_TEMP = 99;                 // Máximo 99°C

const float SNS_VOLT_FACT = 10.20408;      // Factor de diseño para el ADC para 32V Max 

const float SNS_CURR_FACT = 4;             // Factor de diseño para Placa power V2 con Rshunt de 1ohm en cada Mosfet

const float OUT_CURR_FACT = 0.3375;        // Conviente 12000mA a 4050 para el DAC control


extern float CurrentCutOff;                // Corriente máxima de corte seteado o cargado de la EEPROM
extern float PowerCutOff;                  // Potencia de corte seteado o cargado de la EEPROM
extern float tempCutOff;                   // Temperatura máxima de corte seteado o cargado de la EEPROM
extern float ResistorCutOff;                      // Resistencia en Ω
enum ModeType { CC, CP, CR, BC, TC, TL, CA, UNKNOWN };
extern const char* ModeNames[];            // Modos Permitidos

//--------------- Modos CC, CR y CP --------------------------------------


//--------------- Variables para Keypad o entrada de valores -------------

const uint8_t ROWS = 5; // Cinco filas
const uint8_t COLS = 4; // Cuatro columnas

extern uint8_t rowPins[ROWS];          // Pineado de las filas del teclado
extern uint8_t colPins[COLS];          // Pineado de las columnas del teclado
extern char hexaKeys[ROWS][COLS];   // Distribución de Teclas

extern Keypad customKeypad;         // Mapeo de teclas
extern float x;                     // Auxiliar para carga de valores.

//---------------- Variables LCD y TFT ------------------------------------
const unsigned long LCD_RFSH_TIME = 100; // Tiempo de refresco del LCD en ms

extern byte amp_char[8];    // Caracter especial para LCD

// extern uint16_t cellW; // Ancho de celda TFT
// extern uint16_t cellH; // Alto de celda TFT

//---------------- Variables para Modo BC --------------------------------

const float LIPO_DISC_CELL_VLTG = 3.6;  // Voltage mínimo de descarga para baterías LiPo
const float LION_DISC_CELL_VLTG = 3.5;  // Voltage mínimo de descarga para baterías Li-Ion
const float LIPO_STOR_CELL_VLTG = 3.8;  // Voltage mínimo de almacenamiento para baterías LiPo
const float LION_STOR_CELL_VLTG = 3.7;  // Voltage mínimo de almacenamiento para baterías Li-Ion
const unsigned long CRR_STEP_RDCTN = 10;    // Reducción de corriente en 10mA 
const float VLTG_DROP_MARGIN = 0.02;      // ⚡ Margen por debajo de BatteryCutoffVolts para cortar
const float MIN_DISC_CURR = 100;     // 🔋 Corriente mínima antes de desconectar la carga (en mA)



//----------------- Variables para Control de Temperatura -------------------
const unsigned long FAN_ON_DRTN = 60000;                    // Tiempo en miliseg. para mantener los fans encendidos (60 segundos)
const int TMP_CHK_TIME = 1000;                              // Perdíodo de control de temperatura (miliseg.)
//----------------- Parametros de validacion para calibracion -------------------
const float CAL_MIN_VOLTAGE_DELTA = 10.0f;                  // Voltios minimos entre P1 y P2 en calibracion de voltaje
const float CAL_MIN_CURRENT_DELTA = 4.0f;                   // Amperes minimos entre P1 y P2 en calibracion de corriente/salida
const float CAL_MAX_POINT_ERROR_RATIO = 0.20f;              // Error relativo maximo permitido entre setpoint y medicion por punto

#ifndef WOKWI_SIMULATION
#define TEMP_CONVERSION_FACTOR 0.02686202686202686f // Convierte lectura de LM35
#else
#define TEMP_CONVERSION_FACTOR 0.09765625 // Hasta 100°C con el pote de 0 a 5V que simula sensor de temperatura
#endif


//----------------- Variables para Modos TC y TL Transient -------------------

//--- Posiciones reservadas en la EEMPROM cada 4 bytes para valores float ----
const int ADD_CURRENT_CUT_OFF = 0;      // 4 bytes (0-3)
const int ADD_POWER_CUT_OFF = 4;        // 4 bytes (0-3)
const int ADD_TEMP_CUT_OFF = 8;         // 4 bytes (8-11)
const int ADD_SNS_VOLT_FAC_CAL = 12;    // Dirección para Sns_Volt_Calib_Fact
const int ADD_SNS_CURR_FAC_CAL = 16;    // Dirección para Sns_Curr_Calib_Fact
const int ADD_OUT_CURR_FAC_CAL = 20;    // Dirección para Out_Curr_Calib_Fact
const int ADD_SNS_VOLT_OFF_CAL = 24;    // Dirección para Sns_Volt_Calib_Offs
const int ADD_SNS_CURR_OFF_CAL = 28;    // Dirección para Sns_Curr_Calib_Offs
const int ADD_OUT_CURR_OFF_CAL = 32;    // Dirección para Out_Curr_Calib_Offs

#endif

