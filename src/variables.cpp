#include "variables.h"
//#undef WOKWI_SIMULATION

// Creamos los objetos que vamos a utilizar
#ifndef WOKWI_SIMULATION
Adafruit_MCP4725 dac; // Objeto DAC para el MP4725
Adafruit_ADS1115 ads; // Objeto ADS para el ADS1115
#endif
LiquidCrystal_I2C lcd(0x27, 20, 4); // Objeto LCD 20x4 Address 0x27
RTC_DS1307 rtc;                     // Objeto RTC para el DS1307

//--------------- Variables Encoder --------------------------------------

unsigned long lastButtonPress = 0;          // Use this to store if the encoder button was pressed or not
volatile float encoderPosition = 0;         // Antes era volatile float
volatile float factor = 0;                  // Factor de escala del Encoder
volatile unsigned long maxEncoder = 999000; // sets maximum Rotary Encoder value allowed CAN BE CHANGED AS REQUIRED (was 50000)

//--------------- Variables de operacion --------------------------------- 

int16_t adcv, adci;                     // Objetos para los SDC valor en binario del ADC
unsigned long setDAC = 0;       // Voltage de control para el DAC que controlara al MOSFET
float current = 0;                      // Corriente de Carga
float voltage = 0;                      // Voltage de Carga

float Sns_Volt_Calib_Fact = 1.0;        // Factor de calibración para el ADC de V
float Sns_Volt_Calib_Offs = 0.0;        // Offset de calibracion de voltage sensado

float Sns_Curr_Calib_Fact = 1.0;        // Factor de calibración para el ADC de I
float Sns_Curr_Calib_Offs = 0.0;        // Offset de calibracion de corriente sensada

float Out_Curr_Calib_Fact = 1.0;        // Factor de calibración para el DAC de I
float Out_Curr_Calib_Offs = 0.0;        // Offset de calibracion de corriente máxima de salida

int CuPo = 8;                           // Posicion del cursor, es 8 incialmente.
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
bool calibrateVoltage = false;          // True si estoy calibrando V, I si es false
bool firstPointTaken = false;           // Flag de pto uno tomado

//--------------- Modos CC, CR y CP --------------------------------------

float setCurrent = 0;                   // Variable para setear la corriente de carga
float setPower = 0;                     // Variable para setear la potencia de carga
float setResistance = 0;                // Variable para setear la resistencia de carga

//--------------- Variables para Keypad o entrada de valores ------------
// Definicion de las teclas del teclado

char hexaKeys[ROWS][COLS] = {
    {'1', '2', '3', 'M'},
    {'4', '5', '6', 'C'},
    {'7', '8', '9', 'S'},
    {'<', '0', '.', 'E'}};

uint8_t rowPins[ROWS] = {5, 6, 7, 8};    // Pineado de las filas del teclado
uint8_t colPins[COLS] = {9, 10, 11, 12}; // Pineado de las columnas del teclado
Keypad customKeypad = Keypad(makeKeymap(hexaKeys), rowPins, colPins, ROWS, COLS);
char customKey;
char decimalPoint; // Variable para el punto decimal en la entrada de teclado
char numbers[10];  // Variable para la entrada de teclado
byte index = 0;    // poicion para el caranter de la variable numbers
float x = 0;       // Valor que entrega la función Value_Input()

//---------------- Variables LCD -----------------------------------------
int z = 1;        // Posición en renglón (En CC,CP y CR dejo lugar para poner caracter ">"), aun no lo puse
int r = 0;        // Renglon

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

//---------------- Variables para Modo BC --------------------------------
float Seconds = 0;              // Variable para los segundio usada en Battery Capacity Mode (BC)
bool timerStarted = false;      // Variable para saber si el timer esta corriendo
DateTime startTime;             // Variable para el tiempo de inicio
float elapsedSeconds = 0.0;     // Variable para los segundos transcurridos
float BatteryLife = 0;          // Variable para la vida de la batería
float BatteryLifePrevious = 0;  // Variable para la vida de la batería anterior
float BatteryCutoffVolts;       // Variable usada para indicar a que Voltage se puede descargar la batería
float BatteryCurrent;           // Variable usada para setear la corriente de descargga de la bateria

String BatteryType = "    ";    // Para definir el Tipo de Batería

//----------------- Variables para Control de Temperatura -------------------
int temp = 1;                   // Temp. del disipador de MOSFET, =1 porque cero puede coincidir con un error.

//----------------- Variables para Modos TC y TL Transient -------------------
float LowCurrent = 0;                 // Configuración de corriente baja para el modo transitorio
float HighCurrent = 0;                // Configuración de corriente alta para el modo transitorio
unsigned long transientPeriod;        // Para almacenar el período de tiempo del pulso en el modo de pulso transitorio
unsigned long current_time;           // Para almacenar el tiempo actual en microsegundos
unsigned long transientList[10][2];   // Array para almacenar los datos de la lista transitoria
int total_steps;               // Utilizado en el modo de Transient List Mode
int current_step;              // Utilizado en el modo de Transient List Mode
