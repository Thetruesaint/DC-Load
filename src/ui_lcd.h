#ifndef UI_LCD_H
#define UI_LCD_H

#include "variables.h"

void Update_LCD(void);
void printLCD_S(int col, int row, const String &message);
void printLCD(int col, int row, const __FlashStringHelper *message);
void printLCDNumber(int col, int row, float number, char unit = '\0', int decimals = 2);
void Print_Spaces(int col, int row, byte count = 1);

#endif
