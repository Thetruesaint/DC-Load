#include "legacy_base_io.h"

#include "../variables.h"
#include "../funciones.h"
#include "../app/app_inputs.h"
#include "../app/app_loop.h"
#include "../core/core_modes.h"

void legacy_load_off() {
#ifndef WOKWI_SIMULATION
  dac.setVoltage(0, false);
  toggle = false;
  setCurrent = 0;
#else
  toggle = false;
  setCurrent = 0;
#endif
}

void legacy_encoder_status(bool encOnOff, float limit) {
  if (encOnOff) {
    CuPo = 8;
    reading = 0;
    encoderPosition = 0;
    maxReading = limit;
    maxEncoder = maxReading * 1000;

    encoder.clearCount();
  } else {
    encoder.clearCount();
  }
}

void legacy_read_encoder() {
  app_read_encoder();
}

void legacy_cursor_position() {
  static uint32_t lastPressTime = 0;
  constexpr int unitPosition = 8;
  static int last_CuPo = -1;

  if (digitalRead(ENC_BTN) == LOW && millis() - lastPressTime > 200) {
    lastPressTime = millis();
    if (core_mode_is_managed(static_cast<uint8_t>(Mode))) {
      app_push_action(ActionType::EncoderButtonPress, 0, '\0');
    } else {
      CuPo++;
    }
  }

  if (last_CuPo == CuPo) return;

  if (core_mode_is_managed(static_cast<uint8_t>(Mode))) {
    last_CuPo = CuPo;
    setCursorLCD(CuPo, 2);
    return;
  }

  if (CuPo > last_CuPo && CuPo == unitPosition + 1) CuPo++;
  if (CuPo < last_CuPo && CuPo == unitPosition + 1) CuPo--;

  if ((Mode == CC || Mode == BC || Mode == CA) && CuPo > 12) CuPo = unitPosition;
  if ((Mode == CC || Mode == BC || Mode == CA) && CuPo < 8) CuPo = unitPosition + 4;
  if ((Mode == CP || Mode == CR) && CuPo > 10) CuPo = unitPosition - 2;
  if ((Mode == CP || Mode == CR) && CuPo < 6) CuPo = unitPosition + 2;

  switch (CuPo) {
    case 6: factor = 100000; break;
    case 7: factor = 10000; break;
    case 10: factor = 100; break;
    case 11: factor = 10; break;
    case 12: factor = 1; break;
    default: factor = 1000;
  }
  last_CuPo = CuPo;

  setCursorLCD(CuPo, 2);
}

void legacy_read_volts_current() {
#ifndef WOKWI_SIMULATION

  float raw_voltage;
  float raw_current;

  ads.setGain(GAIN_TWOTHIRDS);
  adcv = ads.readADC_SingleEnded(VLTG_SNSR);
  raw_voltage = ads.computeVolts(adcv) * SNS_VOLT_FACT;

  voltage = raw_voltage * Sns_Volt_Calib_Fact + Sns_Volt_Calib_Offs;

  ads.setGain(GAIN_ONE);
  adci = ads.readADC_SingleEnded(CRR_SNSR);
  raw_current = ads.computeVolts(adci) * SNS_CURR_FACT;

  current = raw_current * Sns_Curr_Calib_Fact + Sns_Curr_Calib_Offs;

#else

  int potValue = analogRead(VSIM);
  static float simulatedVoltage = 0;
  static unsigned long lastDecreaseTime = 0;
  unsigned long currentMillis = millis();

  if (Mode != BC && Mode != CA) {
    simulatedVoltage = map(potValue, 0, 1023, 550, 0) / 10.0;
    voltage = simulatedVoltage;
  } else if (Mode == BC) {
    if (toggle && (currentMillis - lastDecreaseTime >= 2000)) {
      lastDecreaseTime = currentMillis;
      simulatedVoltage -= 0.005;
      simulatedVoltage = max(simulatedVoltage, 0.0f);
      voltage = simulatedVoltage;
    } else if (!toggle) {
      simulatedVoltage = map(potValue, 0, 1023, 550, 0) / 10.0;
      voltage = simulatedVoltage;
    }
  } else if (Mode == CA) {
    simulatedVoltage = map(potValue, 0, 1023, 550, 0) / 10.0;
    float error_voltage = simulatedVoltage * 1.05 - 0.1;
    voltage = error_voltage * Sns_Volt_Calib_Fact + Sns_Volt_Calib_Offs;
  }

  if (toggle) {
    current = setCurrent / 1000;
  } else {
    current = 0;
  }

#endif
}
