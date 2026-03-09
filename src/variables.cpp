
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





float CurrentCutOff = MAX_CURRENT;      // Mantendra el valor de la corriente de corte seteado o cargado de la EEPROM
float PowerCutOff = MAX_POWER;          // Mantendra el valor de la potencia de corte seteado o cargado de la EEPROM
float tempCutOff = MAX_TEMP;            // Mantendra el valor de la temperatura de corte seteado o cargado de la EEPROM
float ResistorCutOff = MAX_RESISTOR;    // Maximo valor de resistencia en Ω

//--------------- Modos CC, CR y CP --------------------------------------


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


//----------------- Variables para Control de Temperatura -------------------

//----------------- Variables para Modos TC y TL Transient -------------------

