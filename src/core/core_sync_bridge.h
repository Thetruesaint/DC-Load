#ifndef CORE_SYNC_BRIDGE_H
#define CORE_SYNC_BRIDGE_H

#include "core_state.h"

struct RuntimeSnapshot {
  float setCurrent_mA;
  float setPower_W;
  float setResistance_Ohm;

  float measuredCurrent_A;
  float measuredVoltage_V;
  float measuredPower_W;
  float temp_C;

  float readingValue;
  float encoderPositionRaw;
  float encoderStep;
  float encoderMaxRaw;
  float currentCutOffA;
  float powerCutOffW;
  float tempCutOffC;
  float fanTempOnC;
  float fanHoldSeconds;
  bool fanManualOverrideActive;
  bool fanManualStateOn;
  float batteryCutoffVolts;
  float batteryLife;
  bool batteryDone;
  char batteryType[8];
  float transientLowCurrentA;
  float transientHighCurrentA;
  float transientPeriodMs;
  uint8_t transientListActiveStep;
  uint8_t transientListTotalSteps;

  int cursorPosition;
  int functionIndex;

  bool loadEnabled;
  uint8_t mode;
  bool modeInitialized;
  bool modeConfigured;
};

inline RuntimeSnapshot core_runtime_snapshot_make_default() {
  RuntimeSnapshot snapshot = {0};
  return snapshot;
}

void core_sync_merge_runtime_state(SystemState *current, const RuntimeSnapshot &incoming);

#endif
