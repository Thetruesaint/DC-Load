#ifndef UI_DISPLAY_H
#define UI_DISPLAY_H

#include <Arduino.h>

struct UiViewState;

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
void uiDisplayRenderUpdateMenu(const UiViewState &state);
void uiDisplayRenderFwUpdateScreen(const char *statusLine, const char *detailLine, const char *hintLine);
void uiDisplayRenderFanSettingsMenu(const UiViewState &state);
void uiDisplayRenderLimitsMenu(const UiViewState &state);
void uiDisplayRenderCalibrationMenu(const UiViewState &state);
void uiDisplayRenderClockMenu(const UiViewState &state);
void uiDisplayRenderCalibrationSetupMenu(const char *inputText);
void uiDisplayRenderCalibrationResultScreen(bool voltageMode, float sensorFactor, float sensorOffset, float outputFactor, float outputOffset);
void uiDisplayRenderCalibrationAbortScreen(bool pointsTooClose);
void uiDisplayRenderCalibrationNoticeScreen(const char *title, const char *detail);
void uiDisplayRenderProtectionModal(const char *message, char causeCode);

#endif
