#ifndef UI_DISPLAY_H
#define UI_DISPLAY_H

#include <Arduino.h>

struct UiViewState;

struct UiGridMetrics {
  uint16_t cols;
  uint16_t rows;
  uint16_t cellWidthPx;
  uint16_t cellHeightPx;
  uint16_t originXPx;
  uint16_t originYPx;
  uint8_t textSize;
};

const UiGridMetrics &uiGridMetrics();
int uiGridPixelX(int col);
int uiGridPixelY(int row);
int uiDisplayWidthPx();
int uiDisplayHeightPx();
int uiDisplayTextWidth(const char *text, uint8_t textSize = 1, uint8_t textFont = 1);
int uiDisplayTextWidth(const String &text, uint8_t textSize = 1, uint8_t textFont = 1);
int uiDisplayFontHeight(uint8_t textSize = 1, uint8_t textFont = 1);

void uiDisplayInit(void);
void uiDisplayClear(void);
void uiDisplayUpdate(void);
void uiDisplayInvalidateHomeLayout(void);
void uiDisplayFillRect(int x, int y, int w, int h, uint16_t color);
void uiDisplayDrawRect(int x, int y, int w, int h, uint16_t color);
void uiDisplayFillCircle(int x, int y, int r, uint16_t color);
void uiDisplayDrawCircle(int x, int y, int r, uint16_t color);
void uiDisplayPrintAt(int x, int y, const char *text, uint16_t fg, uint16_t bg, uint8_t textSize);
void uiDisplayPrintAt(int x, int y, const String &text, uint16_t fg, uint16_t bg, uint8_t textSize);
void uiDisplayPrintStyledAt(int x, int y, const char *text, uint16_t fg, uint16_t bg, uint8_t textSize, uint8_t textFont);
void uiDisplayPrintStyledAt(int x, int y, const String &text, uint16_t fg, uint16_t bg, uint8_t textSize, uint8_t textFont);
void uiDisplayRenderBatterySetupTask(const UiViewState &state);
void uiDisplayRenderBatterySetupCustom(const UiViewState &state);
void uiDisplayRenderBatterySetupCells(const UiViewState &state);
void uiDisplayUpdateBatterySetupCustomValue(const UiViewState &state);
void uiDisplayUpdateBatterySetupCellsValue(const UiViewState &state);
void uiDisplayRenderTransientContSetup(const UiViewState &state);
void uiDisplayUpdateTransientContSetupValue(const UiViewState &state);
void uiDisplayRenderTransientListSetup(const UiViewState &state);
void uiDisplayUpdateTransientListSetupValue(const UiViewState &state);
void uiDisplayRenderStartupSplash(bool rtcDetected, int tempC);
void uiDisplayRenderStartupHealthCheck(bool dacDetected, bool adsDetected, bool rtcDetected, int tempC, bool sensorOk);
void uiDisplayRenderConfigRootMenu(const UiViewState &state);
void uiDisplayRenderProtectionMenu(const UiViewState &state);
void uiDisplayRenderTestsMenu(const UiViewState &state);
void uiDisplayRenderFwUpdateScreen(const char *statusLine, const char *detailLine, const char *hintLine);
void uiDisplayRenderFanSettingsMenu(const UiViewState &state);
void uiDisplayRenderLimitsMenu(const UiViewState &state);
void uiDisplayRenderCalibrationMenu(const UiViewState &state);
void uiDisplayRenderCalibrationSetupMenu(const char *inputText);
void uiDisplayRenderCalibrationResultScreen(bool voltageMode, float sensorFactor, float sensorOffset, float outputFactor, float outputOffset);
void uiDisplayRenderCalibrationAbortScreen(bool pointsTooClose);
void uiDisplayRenderCalibrationNoticeScreen(const char *title, const char *detail);
void uiDisplayRenderProtectionModal(const char *message, char causeCode);

void uiGridSetCursor(int col, int row);
void uiClearCells(int col, int row, byte count = 1);
void uiGridPrintString(int col, int row, const String &message);
void uiGridPrint(int col, int row, const __FlashStringHelper *message);
void uiGridPrintNumber(int col, int row, float number, char unit = '\0', int decimals = 2);

void printLCDRaw(const String &message);
void printLCDRaw(const char *message);
void printLCDRaw(const __FlashStringHelper *message);
void printLCDRaw(char value);
void printLCDRaw(int value);
void printLCDRaw(unsigned long value);
void printLCDRaw(float value, int decimals = 2);

// Legacy compatibility layer while the codebase migrates away from LCD naming.
void initLCD(void);
void clearLCD(void);
void setCursorLCD(int col, int row);
void Update_LCD(void);
void printLCD_S(int col, int row, const String &message);
void printLCD(int col, int row, const __FlashStringHelper *message);
void printLCDNumber(int col, int row, float number, char unit = '\0', int decimals = 2);
void Print_Spaces(int col, int row, byte count = 1);

#endif
