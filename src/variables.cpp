
#include "variables.h"


// Creamos los objetos que vamos a utilizar
#ifndef WOKWI_SIMULATION
Adafruit_MCP4725 dac; // Objeto DAC para el MP4725
Adafruit_ADS1115 ads; // Objeto ADS para el ADS1115
#endif
LiquidCrystal_I2C lcd(0x27, 20, 4); // Objeto LCD 20x4 Address 0x27
TFT_eSPI tft = TFT_eSPI(); // Objeto TFT para la pantalla
RTC_DS1307 rtc; // Objeto RTC para el DS1307
ESP32Encoder encoder; // Objeto Encoder para el ESP32Encoder

//--------------- Variables Encoder --------------------------------------

unsigned long lastButtonPress = 0;          // Use this to store if the encoder button was pressed or not

//--------------- Variables de operacion --------------------------------- 

int16_t adcv, adci;                     // Objetos para los SDC valor en binario del ADC
unsigned long setDAC = 0;               // Voltage de control para el DAC que controlara al MOSFET

float Sns_Volt_Calib_Fact = 1.0;        // Factor de calibración para el ADC de V
float Sns_Volt_Calib_Offs = 0.0;        // Offset de calibracion de voltage sensado

float Sns_Curr_Calib_Fact = 1.0;        // Factor de calibración para el ADC de I
float Sns_Curr_Calib_Offs = 0.0;        // Offset de calibracion de corriente sensada

float Out_Curr_Calib_Fact = 1.0;        // Factor de calibración para el DAC de I
float Out_Curr_Calib_Offs = 0.0;        // Offset de calibracion de corriente máxima de salida

bool toggle = false;                    // Conmuta la carga On/Off
float reading = 0;                      // Variable para Encoder dividido por 1000
float maxReading = 0;                   // Máximo valor permitido para reading (en unidades, ej. A, W, Ω)
float CurrentCutOff = MAX_CURRENT;      // Mantendra el valor de la corriente de corte seteado o cargado de la EEPROM
float PowerCutOff = MAX_POWER;          // Mantendra el valor de la potencia de corte seteado o cargado de la EEPROM
float tempCutOff = MAX_TEMP;            // Mantendra el valor de la temperatura de corte seteado o cargado de la EEPROM
float ResistorCutOff = MAX_RESISTOR;    // Maximo valor de resistencia en Ω
ModeType Mode = CC;                     // Modo de operación, CC Default
bool modeInitialized = false;           // Para reimprimir la plantilla del modo y/o inicializar valores
bool modeConfigured = false;            // Para BC, TC o TL, indica que hay que seterarlos
int functionIndex = 0;                  // Para seleccionar los Modos.
bool hlth = true;                       // Flag de Salud gral.

//--------------- Modos CC, CR y CP --------------------------------------

float setCurrent = 0;                   // Variable para setear la corriente de carga

//--------------- Variables para Keypad o entrada de valores ------------ 
// Definicion de las teclas del teclado

char hexaKeys[ROWS][COLS] = {
	{'1', '2', '3', 'M',},
	{'4', '5', '6', 'C'},
	{'7', '8', '9', 'S'},
	{'<', '0', '.', 'E'},
	{'U', 'D', 'L', 'R'}};
byte rowPins[ROWS] = {33, 5, 27, 12, 0};  
byte colPins[COLS] = {34, 35, 19, 26};

Keypad customKeypad = Keypad(makeKeymap(hexaKeys), rowPins, colPins, ROWS, COLS);

float x = 0;        // Valor que entrega la función Value_Input()

//---------------- Variables LCD y TFT ------------------------------------

byte amp_char[8] = {       // Unidad de Amperes con custom A, solo para indicar valores que pueden cambiar. Los de Set son con mayusculas comunes
    0b00000,  //      
    0b00000,  //      
    0b00100,  //   *  
    0b01010,  //  * *
    0b10001,  // *   *
    0b11111,  // *****
    0b10001,  // *   *
    0b00000,  //      
  };

// uint16_t cellW = 22;
// uint16_t cellH = 32;

//---------------- Variables para Modo BC --------------------------------
float Seconds = 0;              // Variable para los segundio usada en Battery Capacity Mode (BC)
bool mytimerStarted = false;      // Variable para saber si el timer esta corriendo
DateTime startTime;             // Variable para el tiempo de inicio
float elapsedSeconds = 0.0;     // Variable para los segundos transcurridos
float BatteryLife = 0;          // Variable para la vida de la batería
float BatteryLifePrevious = 0;  // Variable para la vida de la batería anterior
float BatteryCutoffVolts;       // Variable usada para indicar a que Voltage se puede descargar la batería
float BatteryCurrent;           // Variable usada para setear la corriente de descargga de la bateria

String BatteryType = "    ";    // Para definir el Tipo de Batería

//----------------- Variables para Control de Temperatura -------------------

//----------------- Variables para Modos TC y TL Transient -------------------
float LowCurrent = 0;                 // Configuración de corriente baja para el modo transitorio
float HighCurrent = 0;                // Configuración de corriente alta para el modo transitorio
unsigned long transientPeriod;        // Para almacenar el período de tiempo del pulso en el modo de pulso transitorio
unsigned long current_time;           // Para almacenar el tiempo actual en microsegundos
unsigned long transientList[10][2];   // Array para almacenar los datos de la lista transitoria
int total_steps;               // Utilizado en el modo de Transient List Mode
int current_step;              // Utilizado en el modo de Transient List Mode


