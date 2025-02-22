#ifndef VARIABLES_H
#define VARIABLES_H

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

//-------------------I/O Pins------------------------------------------------------------

const byte TEMP_SNSR = A0;  // Sensado de Temperatura
const byte FAN_CTRL = A2;   // Endendido de Fans
const byte ENC_A = 3;       // Encoder Pin A
const byte ENC_B = 2;       // Encoder Pin B
const byte ENC_BTN = 4;     // Encoder Boton
const byte CRR_SNSR = 1;    // Input A1 from ADC
const byte VLTG_SNSR = 3;   // Input A3 from ADC
const byte LOADONOFF = 15;  // Input A1 used as a digital pin to set Load ON/OFF


//--------------- Variables para Encoder -------------------------------------------------

extern unsigned long lastButtonPress;     // Use this to store if the encoder button was pressed or not
extern volatile float encoderPosition;    // Antes era volatile float
extern volatile float factor;             // Factor de escala del Encoder
extern volatile unsigned long encoderMax; // sets maximum Rotary Encoder value allowed CAN BE CHANGED AS REQUIRED (was 50000)

//--------------- Variables de operacion y Modos CC, CR y CP----------------------------------------

extern int16_t adc1, adc3;                 // ADCs usados, adc0 y adc2 a GND
extern unsigned long controlVoltage;       // Voltage de control para el DAC que controlara al MOSFET
extern float current;                      // Corriente de Carga
extern float voltage;                      // Voltage de Carga
const float MAX_VOLTAGE = 90;              // Por diseño puede medir hasta 200V, pero los MOSFET soportan solo hasta 100V
extern int CuPo;                           // Posicion inicial del cursor
extern bool toggle;                        // Conmuta la carga On/Off
extern float reading;                      // Variable para Encoder dividido por 1000
extern float setCurrent;                   // Variable para setear la corriente de carga
extern float setPower;                     // Variable para setear la potencia de carga
extern float setResistance;                // Variable para setear la resistencia de carga
extern float Set_Curr_Dsgn_Fact;           // Factor de diseño para el DAC, paso a convertir 5a1 (5V a 1V de Ref.) osea Corriente máxima 10A V1.63 = 0.4095
extern int CurrentCutOff;                  // Corriente maxima de corte seteado o cargado de la EEPROM
extern int PowerCutOff;                    // Potencia de corte seteado o cargado de la EEPROM
extern int tempCutOff;                     // Temperatura máxima de corte seteado o cargado de la EEPROM
extern float ResistorCutOff;               // Maximo valor de resistencia para el modo CR
enum ModeType { CC, CP, CR, BC, TC, TL, UNKNOWN };
extern ModeType Mode;                      // Modo de operación, CC Default
extern const char* ModeNames[];            // Modos Permitidos
extern bool modeInitialized;               // Para reimplirmir la plantilla del modo y/o inicializar valores
extern bool modeConfigured;                // Para BC, TC o TL, indica que hay que seterarlos
const float VoltageDropMargin = 0.1;       // ⚡ Margen de voltaje donde empieza a reducir corriente (parametrizable)
const float MinDischargeCurrent = 90;      // 🔋 Corriente mínima antes de desconectar la carga (en mA)
const unsigned long CRR_STEP_RDCTN = 5;    // Reducción de corriente en 100mA

//----------------------------------------Variables para el Keypad-------------------------------------------

const byte ROWS = 4; // Cuatro filas
const byte COLS = 4; // Cuatro columnas

extern byte rowPins[ROWS];          // Pineado de las filas del teclado
extern byte colPins[COLS];          // Pineado de las columnas del teclado
extern char hexaKeys[ROWS][COLS];   // Distribución de Teclas
extern int functionIndex;           // Usado para seleccionar los Modos.
extern Keypad customKeypad;         // Mapeo de teclas
extern char customKey;              // Auxiliar para tomar la tecla presionada
extern char decimalPoint;           // Flag de punto decimal en la entrada de teclado
extern char numbers[10];            // Guarda valor ingresado
extern byte index;                  // poicion para el caranter de la variable numbers
extern float x;                     // Auxiliar para carga de valores.

//-----------------------------------Variables de coordenadas para ubicar valores en LCD--------------------
extern int z; // Posición en renglón (En CC,CP y CR dejo lugar para poner caracter ">"), aun no lo puse
extern int r; // Renglon

//---------------------------------------Inicializo Variables para Modo Baterias-----------------------------
extern float Seconds;               // Segundos usada en Battery Capacity Mode (BC)
extern bool timerStarted;           // Flag de estado de timer
extern DateTime startTime;          // Tiempo de inicio
extern float elapsedSeconds;        // Segundos transcurridos
extern float BatteryLife;           // Vida de la batería en mAh
extern float BatteryLifePrevious;   // Vida de la batería anterior
extern float BatteryCutoffVolts;    // Voltage de corte de descarga de batería
extern float BatteryCurrent;        // Corriente máxima de descargga de bateria
extern float LoadCurrent;           // Registra ultimo corriente seteada
extern float LiPoCutOffVoltage;  // Voltage mínimo de descarga para baterias LiPo
extern float LionCutOffVoltage;  // Voltage mínimo de descarga para baterias Lion
extern float LiPoStoragVoltage;  // Voltage mínimo de almacenamiento para baterias LiPo
extern float LionStoragVoltage;  // Voltage mínimo de almacenamiento para baterias Liom
extern String BatteryType;    // Para definir el Tipo de Batería
extern bool exitMode;          // Para salir de la selección de tipo de batería

//---------------------------------------Variables para Control de Temperatura-----------------------------
extern int temp;                   // Temp. del disipador de MOSFET, =1 porque cero puede coincidir con un error.
extern unsigned long Last_tmpchk;  // Tiempo desde el ùltimo chequeo de temperatura
extern unsigned long fan_on_time;  // Tiempo que lleva encendido el Fan
extern bool fans_on;           // Flag de stado del Cooler
extern bool new_temp;           // flag si hay nuevo valor de temperatura

//---------------------------------------Variables para Modo Transient------------------------------------
extern float LowCurrent;                 // Configuración de corriente baja para el modo transitorio
extern float HighCurrent;                // Configuración de corriente alta para el modo transitorio
extern unsigned long transientPeriod;        // Para almacenar el período de tiempo del pulso en el modo de pulso transitorio
extern unsigned long current_time;           // Para almacenar el tiempo actual en microsegundos
extern unsigned long transientList[10][2];           // Array para almacenar los datos de la lista transitoria
extern int total_instructions;               // Utilizado en el modo de Transient List Mode
extern int current_instruction;              // Utilizado en el modo de Transient List Mode

//------------------------------ Posiciones reservadas en la EEMPROM --------------------------------------
const int ADD_CURRENT_CUT_OFF = 0x00;
const int ADD_POWER_CUT_OFF = 0x20;
const int ADD_TEMP_CUT_OFF = 0x40;

//---------------------------------------Variables para Control de Temperatura-----------------------------
const int FAN_ON_DRTN = 30000;  // Tiempo en miliseg. para mantener los fans encendidos (30 segundos)
const int TMP_CHK_TIME = 800;   // Perdíodo de control de temperatura (miliseg.)

#endif