#include "hw_objects.h"

#ifndef WOKWI_SIMULATION
Adafruit_MCP4725 dac; // Objeto DAC para el MP4725
Adafruit_ADS1115 ads; // Objeto ADS para el ADS1115
#endif
TFT_eSPI tft = TFT_eSPI(); // Objeto TFT para la pantalla
RTC_DS1307 rtc; // Objeto RTC para el DS1307
ESP32Encoder encoder; // Objeto Encoder para el ESP32Encoder
