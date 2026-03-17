#ifndef HW_OBJECTS_H
#define HW_OBJECTS_H

#include <Arduino.h>
#include <Wire.h>
#ifndef WOKWI_SIMULATION
#include <Adafruit_ADS1X15.h>
#include <Adafruit_MCP4725.h>
#endif
#include <TFT_eSPI.h>
#include <SPI.h>
#include <RTClib.h>
#include <ESP32Encoder.h>

#ifndef WOKWI_SIMULATION
extern Adafruit_MCP4725 dac;
extern Adafruit_ADS1115 ads;
#endif
extern TFT_eSPI tft;
extern RTC_DS1307 rtc;
extern ESP32Encoder encoder;

#endif

