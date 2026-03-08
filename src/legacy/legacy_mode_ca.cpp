#include "legacy_mode_ca.h"

#include "../variables.h"
#include "../ui_lcd.h"
#include "../funciones.h"
#include "../app/app_value_input.h"

void legacy_calibration_mode() {
  if (!modeConfigured) {
    legacy_calibration_setup();
    return;
  }

  if (!modeInitialized) {
    if (x == 1) {
      calibrateVoltage = true;
      Sns_Volt_Calib_Fact = 1.0;
      Sns_Volt_Calib_Offs = 0.0;
    } else if (x == 2) {
      calibrateVoltage = false;
      Sns_Curr_Calib_Fact = 1.0;
      Sns_Curr_Calib_Offs = 0.0;
      Out_Curr_Calib_Fact = 1.0;
      Out_Curr_Calib_Offs = 0.0;
    }

    clearLCD();
    printLCD(0, 0, calibrateVoltage ? F("CA VOLT") : F("CA CURR"));
    printLCD(14, 1, firstPointTaken ? F("Set P2") : F("Set P1"));
    printLCD(1, 2, F("Adj->"));
    printLCD(13, 2, F("A"));
    printLCD(0, 3, F(">"));
    printLCD(7, 3, F("<Set real"));
    Encoder_Status(true, CurrentCutOff);
    modeInitialized = true;
  }

  reading = encoderPosition / 1000;
  reading = min(maxReading, max(0.0f, reading));
  encoderPosition = reading * 1000.0;
  Cursor_Position();

  if (!toggle) {
    return;
  }
  setCurrent = reading * 1000;
}

void legacy_calibration_setup() {
  clearLCD();
  printLCD(4, 0, F("CALIBRATION"));
  printLCD(0, 1, F("1-Voltage 2-Current"));
  printLCD(0, 2, F("3-Load    4-Save"));

  do {
    z = 1;
    r = 3;
    printLCD(z - 1, r, F(">"));
    Print_Spaces(z, r, 1);
    if (!Value_Input(z, r, 1, false)) {
      return;
    }
  } while (x < 1 || x > 4);

  modeConfigured = true;
  if (x == 3) {
    Load_Calibration();
    printLCD(12, 3, F("Loaded!"));
    delay(1500);
    modeConfigured = false;
  }
  if (x == 4) {
    Save_Calibration();
    printLCD(12, 3, F("Saved!"));
    delay(1500);
    modeConfigured = false;
  }

  firstPointTaken = false;
  modeInitialized = false;
}

void legacy_calibrate(float realValue) {
  static float measuredValue1 = 0, realValue1 = 0;
  static float measuredValue2 = 0, realValue2 = 0;
  static float setCurrent1 = 0;
  static float setCurrent2 = 0;

  float measuredValue = calibrateVoltage ? voltage : current;

  if (!firstPointTaken) {
    measuredValue1 = measuredValue;
    realValue1 = realValue;
    setCurrent1 = setCurrent / 1000;
    firstPointTaken = true;
    modeInitialized = false;
    return;
  }

  measuredValue2 = measuredValue;
  realValue2 = realValue;
  setCurrent2 = setCurrent / 1000;
  Load_OFF();
  firstPointTaken = false;

  float measuredDelta = fabsf(measuredValue2 - measuredValue1);
  float setCurrentDelta = fabsf(setCurrent2 - setCurrent1);

  bool pointsTooClose = calibrateVoltage
    ? (measuredDelta < CAL_MIN_VOLTAGE_DELTA)
    : (setCurrentDelta < CAL_MIN_CURRENT_DELTA);

  float errRatio1 = 0.0f;
  float errRatio2 = 0.0f;
  bool pointMismatch = false;
  if (!calibrateVoltage) {
    errRatio1 = fabsf(measuredValue1 - setCurrent1) / max(setCurrent1, 0.001f);
    errRatio2 = fabsf(measuredValue2 - setCurrent2) / max(setCurrent2, 0.001f);
    pointMismatch = (errRatio1 > CAL_MAX_POINT_ERROR_RATIO) || (errRatio2 > CAL_MAX_POINT_ERROR_RATIO);
  }

  if (pointsTooClose || pointMismatch) {
    clearLCD();
    printLCD(0, 1, F("Calib Abort"));
    if (pointsTooClose) {
      printLCD(0, 2, F("P1/P2 too close"));
    } else {
      printLCD(0, 2, F("Set/Read >20%"));
    }
    modeInitialized = false;
    delay(2000);
    return;
  }

  float factor = max(0.9f, min(1.1f, (realValue2 - realValue1) / (measuredValue2 - measuredValue1)));
  float offset = max(-0.1f, min(0.1f, realValue1 - (measuredValue1 * factor)));

  if (calibrateVoltage) {
    Sns_Volt_Calib_Fact = factor;
    Sns_Volt_Calib_Offs = offset;
  } else {
    Sns_Curr_Calib_Fact = factor;
    Sns_Curr_Calib_Offs = offset;
    Out_Curr_Calib_Fact = max(0.9f, min(1.1f, (realValue2 - realValue1) / (setCurrent2 - setCurrent1)));
    Out_Curr_Calib_Offs = max(-0.1f, min(0.1f, realValue1 - (setCurrent1 * Out_Curr_Calib_Fact))) * 1000;
  }

  clearLCD();
  printLCD(4, 1, F("Calibrated!"));
  modeConfigured = false;
  firstPointTaken = false;
  delay(2000);
}
