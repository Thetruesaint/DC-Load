#ifndef UI_LCD_H
#define UI_LCD_H

#include <Arduino.h>

struct UiGridMetrics {
  uint16_t cols;
  uint16_t rows;
  uint16_t cellWidthPx;
  uint16_t cellHeightPx;
  uint16_t originXPx;
  uint16_t originYPx;
  uint8_t textSize;
};

void initLCD(void);
void clearLCD(void);
void setCursorLCD(int col, int row);
const UiGridMetrics &uiGridMetrics();
int uiGridPixelX(int col);
int uiGridPixelY(int row);
void uiClearCells(int col, int row, byte count = 1);
void printLCDRaw(const String &message);
void printLCDRaw(const char *message);
void printLCDRaw(const __FlashStringHelper *message);
void printLCDRaw(char value);
void printLCDRaw(int value);
void printLCDRaw(unsigned long value);
void printLCDRaw(float value, int decimals = 2);

void Update_LCD(void);
void printLCD_S(int col, int row, const String &message);
void printLCD(int col, int row, const __FlashStringHelper *message);
void printLCDNumber(int col, int row, float number, char unit = '\0', int decimals = 2);
void Print_Spaces(int col, int row, byte count = 1);

#endif
