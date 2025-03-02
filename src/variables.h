#ifndef VARIABLES_H
#define VARIABLES_H
//#undef WOKWI_SIMULATION

#include <Arduino.h>
#include <LiquidCrystal_I2C.h>
#ifndef WOKWI_SIMULATION
#include <Adafruit_ADS1X15.h>
#include <Adafruit_MCP4725.h>
#endif
#include <Keypad.h>
#include <RTClib.h>

// Objetos para el hardware
#ifndef WOKWI_SIMULATION
extern Adafruit_MCP4725 dac; 
extern Adafruit_ADS1115 ads;
#endif
extern LiquidCrystal_I2C lcd;
extern RTC_DS1307 rtc;

//---------------- I/O Pins ----------------------------------------------

const uint8_t TEMP_SNSR = A0;  // Sensado de Temperatura
const uint8_t FAN_CTRL = A2;   // Endendido de Fans
const uint8_t BUZZER = A6;     // Enable output control
const uint8_t ENC_A = 3;       // Encoder Pin A
const uint8_t ENC_B = 2;       // Encoder Pin B
const uint8_t ENC_BTN = 4;     // Encoder Boton
const uint8_t CRR_SNSR = 1;    // Input A1 from ADC
const uint8_t VLTG_SNSR = 3;   // Input A3 from ADC
const uint8_t LOADONOFF = 15;  // Input A1 used as a digital pin to set Load ON/OFF

//--------------- Variables Encoder --------------------------------------

extern unsigned long lastButtonPress;     // Use this to store if the encoder button was pressed or not
extern volatile float encoderPosition;    // Antes era volatile float
extern volatile float factor;             // Factor de escala del Encoder
extern volatile unsigned long maxEncoder; // sets maximum Rotary Encoder value allowed CAN BE CHANGED AS REQUIRED (was 50000)

//--------------- Variables de operacion --------------------------------- 

const float MAX_VOLTAGE = 50.00;           // Vltg sns hasta 51V, no veo utilidad en permitir mas (los MOSFET soportan hasta 100V).
const float MAX_RESISTOR = 999.9;          // Máximo 999.9 Ω
const float MAX_CURRENT = 10.000;          // Máximo 10A
const float MAX_POWER = 300.0;             // Máximo 300W
const float MAX_TEMP = 99;                 // Máximo 99°C


const float SNS_VOLT_FACT = 12.869;        // Factor de diseño para el ADC para 50V Max 
extern float Sns_Volt_Calib_Fact;          // Factor de calibración para el ADC de V
extern float Sns_Volt_Calib_Offs;          // Offset de calibracion de voltage sensado

const float SNS_CURR_FACT = 10;            // Factor de diseño para el ADC validar
extern float Sns_Curr_Calib_Fact;          // Factor de calibración para el ADC de I
extern float Sns_Curr_Calib_Offs;          // Offset de calibracion de corriente sensada

const float OUT_CURR_FACT = 0.40544;       // Factor de diseño para el DAC, 5V a 1.1V -> Corriente máxima 10.1A Teórica. Verificar
extern float Out_Curr_Calib_Fact;          // Factor de calibración para el DAC de I
extern float Out_Curr_Calib_Offs;          // Offset de calibracion de corriente máxima de salida


extern int16_t adcv, adci;                 // Objetos para los SDC valor en binario del ADC
extern unsigned long controlVoltage;       // Voltage de control para el DAC que controlara al MOSFET
extern float current;                      // Corriente de Carga
extern float voltage;                      // Voltage de Carga
extern int CuPo;                           // Posicion inicial del cursor
extern bool toggle;                        // Conmuta la carga On/Off
extern float reading;                      // Para tomar valores de encoder
extern float maxReading;                   // Máximo valor permitido para reading (en unidades, ej. A, W, Ω)
extern float CurrentCutOff;                // Corriente máxima de corte seteado o cargado de la EEPROM
extern float PowerCutOff;                  // Potencia de corte seteado o cargado de la EEPROM
extern float tempCutOff;                   // Temperatura máxima de corte seteado o cargado de la EEPROM
extern float ResistorCutOff;                      // Resistencia en Ω
enum ModeType { CC, CP, CR, BC, TC, TL, CA, UNKNOWN };
extern ModeType Mode;                      // Modo de operación, CC Default
extern const char* ModeNames[];            // Modos Permitidos
extern bool modeInitialized;               // Para reimplirmir la plantilla del modo y/o inicializar valores
extern bool modeConfigured;                // Para BC, TC o TL, indica que hay que seterarlos
extern int functionIndex;                  // Para seleccionar los Modos.

//--------------- Modos CC, CR y CP --------------------------------------

extern float setCurrent;                   // Variable para setear la corriente de carga
extern float setPower;                     // Variable para setear la potencia de carga
extern float setResistance;                // Variable para setear la resistencia de carga

//--------------- Variables para Keypad o entrada de valores -------------

const uint8_t ROWS = 4; // Cuatro filas
const uint8_t COLS = 4; // Cuatro columnas

extern uint8_t rowPins[ROWS];          // Pineado de las filas del teclado
extern uint8_t colPins[COLS];          // Pineado de las columnas del teclado
extern char hexaKeys[ROWS][COLS];   // Distribución de Teclas
extern Keypad customKeypad;         // Mapeo de teclas
extern char customKey;              // Auxiliar para tomar la tecla presionada
extern char decimalPoint;           // Flag de punto decimal en la entrada de teclado
extern char numbers[10];            // Guarda valor ingresado
extern byte index;                  // poicion para el caranter de la variable numbers
extern float x;                     // Auxiliar para carga de valores.

//---------------- Variables LCD -----------------------------------------
const unsigned long LCD_RFSH_TIME = 100; // Tiempo de refresco del LCD en ms

extern int z;       // Posición en renglón
extern int r;       // Renglon
extern byte amp_char[8];    // Caracter especial para LCD

//---------------- Variables para Modo BC --------------------------------

const float LIPO_DISC_CELL_VLTG = 3.6;  // Voltage mínimo de descarga para baterías LiPo
const float LION_DISC_CELL_VLTG = 3.5;  // Voltage mínimo de descarga para baterías Li-Ion
const float LIPO_STOR_CELL_VLTG = 3.8;  // Voltage mínimo de almacenamiento para baterías LiPo
const float LION_STOR_CELL_VLTG = 3.7;  // Voltage mínimo de almacenamiento para baterías Li-Ion
const unsigned long CRR_STEP_RDCTN = 5;    // Reducción de corriente en 100mA
const float VLTG_DROP_MARGIN = 0.02;      // ⚡ Margen por debajo de BatteryCutoffVolts para cortar
const float MIN_DISC_CURR = 100;     // 🔋 Corriente mínima antes de desconectar la carga (en mA)


extern float Seconds;               // Segundos usada en Battery Capacity Mode (BC)
extern bool timerStarted;           // Flag de estado de timer
extern DateTime startTime;          // Tiempo de inicio
extern float elapsedSeconds;        // Segundos transcurridos
extern float BatteryLife;           // Vida de la batería en mAh
extern float BatteryLifePrevious;   // Vida de la batería anterior
extern float BatteryCutoffVolts;    // Voltage de corte de descarga de batería
extern float BatteryCurrent;        // Corriente máxima de descargga de bateria
extern String BatteryType;          // Para definir el Tipo de Batería

//----------------- Variables para Control de Temperatura -------------------
const int FAN_ON_DRTN = 30000;      // Tiempo en miliseg. para mantener los fans encendidos (30 segundos)
const int TMP_CHK_TIME = 1000;      // Perdíodo de control de temperatura (miliseg.)

extern int temp;                    // Temp. del disipador de MOSFET, =1 porque cero puede coincidir con un error.

//----------------- Variables para Modos TC y TL Transient -------------------
extern float LowCurrent;                    // Configuración de corriente baja para el modo transitorio
extern float HighCurrent;                   // Configuración de corriente alta para el modo transitorio
extern unsigned long transientPeriod;       // Para almacenar el período de tiempo del pulso en el modo de pulso transitorio
extern unsigned long current_time;          // Para almacenar el tiempo actual en microsegundos
extern unsigned long transientList[10][2];  // Array para almacenar los datos de la lista transitoria
extern int total_steps;                     // Utilizado en el modo de Transient List Mode
extern int current_step;             // Utilizado en el modo de Transient List Mode

//--- Posiciones reservadas en la EEMPROM cada 4 bytes para valores float ----
const int ADD_CURRENT_CUT_OFF = 0;      // 4 bytes (0-3)
const int ADD_POWER_CUT_OFF = 4;        // 4 bytes (0-3)
const int ADD_TEMP_CUT_OFF = 8;         // 4 bytes (8-11)


#endif