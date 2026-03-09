#ifndef STORAGE_EEPROM_H
#define STORAGE_EEPROM_H

#include <EEPROM.h>

float Load_EEPROM(int address);
void Save_EEPROM(int address, float value);
void Load_Calibration(void);
void Save_Calibration(void);

#endif

