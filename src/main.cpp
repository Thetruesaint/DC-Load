#include "config/system_constants.h"
#include "hw/hw_objects.h"
#include "funciones.h"
#include "app/app_loop.h"
#include "app/app_runtime.h"
#include "app/app_limits_context.h"
#include "app/app_measurements_context.h"
#include "app/app_health_context.h"
#include "app/app_fan_context.h"

//---------------------------------------Variables para el Set Up-----------------------------------------
void setup() {

  //-------------------------------------Inicializa I/O---------------------------------------------------
	encoder.attachFullQuad(ENC_B, ENC_A);                     // use pin ENC_B and ENC_A
  ESP32Encoder::useInternalWeakPullResistors = puType::up;  // Enable the weak pull up resistors
  encoder.clearCount();                                     // inicializa el contador del encoder a 0
  pinMode(ENC_BTN, INPUT_PULLUP);
  pinMode(TEMP_SNSR, INPUT);
  analogReadResolution(12);                                 // 12 bits = 0..4095
  analogSetPinAttenuation(TEMP_SNSR, ADC_0db);              // 0 dB -> Vmax ~1.1 V
  pinMode(LOADONOFF, INPUT_PULLUP);
  pinMode(FAN_CTRL, OUTPUT);
  pinMode (BUZZER, OUTPUT);

  #ifdef WOKWI_SIMULATION
  pinMode(VSIM, INPUT);   // Pin para simular el voltaje de carga con un potenciometro en la simulacion.
  #endif

  //-------------------------------------Inicializa perifericos-------------------------------------------
  EEPROM.begin(64);   // tamano en bytes (minimo 64)
  Serial.begin(115200);
  Wire.begin(21, 22);   // SDA=21, SCL=22, fuerza modo Master
  initLCD();
  beepBuzzer();   // Buzzer Test

  #ifndef WOKWI_SIMULATION

  if (dac.begin(0x60)){
    printLCD(0,0, F("dac OK"));
    Serial.print("dac OK");
    ads.setGain(GAIN_TWOTHIRDS);
  } else{
      printLCD(0,0, F("dac NDT")); app_health_set_ok(false);
      Serial.print("dac NDT");
    }

  if (ads.begin()){
      ads.setGain(GAIN_TWOTHIRDS);
      printLCD(0,1, F("ads OK"));
  } else{
      printLCD(0,1, F("ads NDT")); app_health_set_ok(false);
      Serial.print("ads NDT");
    }
 
  #endif

  if (rtc.begin()){
    printLCD(8, 0, F("RTC OK"));
    Serial.print("RTC  OK");
  } else{
      printLCD(8, 0, F("RTC NDT")); app_health_set_ok(false);
      Serial.print("RTC  NDT");
    }

  app_measurements_set_temp_c(static_cast<int>(analogRead(TEMP_SNSR) * TEMP_CONVERSION_FACTOR));
  printLCD_S(11, 1, String(app_measurements_temp_c()) + String((char)0xDF) + "C");

  if (app_health_is_ok() && app_measurements_temp_c() <= 99) {
    printLCD(0,2, F("Sensor Test OK"));
  } else {
    printLCD(0,2, F("Sensor Test FAIL"));
    }

  delay(2000);
 
  //----------------------------------------Configuraciones iniciales de unica vez----------------------------------------------

  if (!rtc.isrunning()) {
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

  //------------------------------------Pantalla Inicio--------------------------------------------------
  DateTime now = rtc.now();
  String date = String(now.day()) + "/" + String(now.month()) + "/" + String(now.year());
  String time = String(now.hour()) + ":" + String(now.minute()) + ":" + String(now.second());

  clearLCD();
  printLCD(0, 0, F("*DC Electronic Load*"));
  printLCD_S(0, 1, date + " - " + time);
  printLCD(0, 2, F("By Guy & Codex"));

  #ifndef WOKWI_SIMULATION
  printLCD(0, 3, F("v2.11"));
  delay (2000);
  #else
  printLCD(0, 3, F("v2.11 sim"));
  delay(1000);
  #endif

  //---------------------------------------Chequea y Muestra los limites configurados----------------------
  #ifndef WOKWI_SIMULATION
    app_limits_set_current_cutoff(Load_EEPROM(ADD_CURRENT_CUT_OFF));
    app_limits_set_power_cutoff(Load_EEPROM(ADD_POWER_CUT_OFF));
    app_limits_set_temp_cutoff(Load_EEPROM(ADD_TEMP_CUT_OFF));
    app_fan_set_temp_on_c(static_cast<int>(Load_EEPROM(ADD_FAN_TEMP_ON)));
    app_fan_set_hold_ms(static_cast<unsigned long>(Load_EEPROM(ADD_FAN_HOLD_MS)));

    const bool invalidLimits =
        app_limits_current_cutoff() <= 1 || app_limits_current_cutoff() > 10 ||
        app_limits_power_cutoff() <= 1 || app_limits_power_cutoff() > 300 ||
        app_limits_temp_cutoff() < 30 || app_limits_temp_cutoff() > 99;

    const bool invalidFan =
        app_fan_temp_on_c() < MIN_FAN_TEMP_ON_C || app_fan_temp_on_c() > MAX_FAN_TEMP_ON_C ||
        app_fan_hold_ms() < MIN_FAN_HOLD_MS || app_fan_hold_ms() > MAX_FAN_HOLD_MS;

    if (invalidLimits) {
      Config_Limits();
    }

    if (invalidFan) {
      app_fan_set_temp_on_c(DEFAULT_FAN_TEMP_ON_C);
      app_fan_set_hold_ms(DEFAULT_FAN_HOLD_MS);
      Save_EEPROM(ADD_FAN_TEMP_ON, static_cast<float>(app_fan_temp_on_c()));
      Save_EEPROM(ADD_FAN_HOLD_MS, static_cast<float>(app_fan_hold_ms()));
    }

    Show_Limits();
    Load_Calibration();
    delay(2000);
    clearLCD();

  #else
    app_limits_set_current_cutoff(10);
    app_limits_set_power_cutoff(300);
    app_limits_set_temp_cutoff(80);
    app_fan_set_temp_on_c(DEFAULT_FAN_TEMP_ON_C);
    app_fan_set_hold_ms(DEFAULT_FAN_HOLD_MS);
    Load_Calibration();
  #endif

  app_init();
}

//------------------------------------- Bucle Principal-------------------------------------------------
void loop() {
  app_run_cycle();
}
