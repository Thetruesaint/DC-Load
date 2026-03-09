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

//---------------- Variables LCD y TFT ------------------------------------
byte amp_char[8] = {
  0b00000,
  0b00000,
  0b00100,
  0b01010,
  0b10001,
  0b11111,
  0b10001,
  0b00000,
};

