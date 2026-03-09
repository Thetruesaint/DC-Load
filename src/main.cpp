
#include "config/system_constants.h"
#include "hw/hw_objects.h"
#include "funciones.h"
#include "app/app_loop.h"
#include "app/app_runtime.h"
#include "app/app_limits_context.h"
#include "app/app_measurements_context.h"
#include "app/app_health_context.h"

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
  pinMode(VSIM, INPUT);   // Pin para simular el voltaje de carga con un potenciómetro en la simulación. Ajusta el pin según tu conexión.
  #endif

  //-------------------------------------Inicializa perifericos-------------------------------------------
  EEPROM.begin(64);   // tamaño en bytes (mínimo 64)
  Serial.begin(115200);
  Wire.begin(21, 22);   // SDA=21, SCL=22, fuerza modo Master
  initLCD();
  beepBuzzer();   // Buzzer Test

  #ifndef WOKWI_SIMULATION

  if (dac.begin(0x60)){                 // initialize the dac with address 0x60
    printLCD(0,0, F("dac OK"));
    Serial.print("dac OK");
    ads.setGain(GAIN_TWOTHIRDS);              // Setea la ganancia del ADC a 2/3x gain +/- 6.144V  1 bit = 0.1875mV
  } else{
      printLCD(0,0, F("dac NDT")); app_health_set_ok(false);
      Serial.print("dac NDT");
    }

  if (ads.begin()){                             // initialize the ads, default address 0x48
      ads.setGain(GAIN_TWOTHIRDS);              // 2/3x gain +/- 6.144V  1 bit = 0.1875mV
      printLCD(0,1, F("ads OK"));
  } else{
      printLCD(0,1, F("ads NDT")); app_health_set_ok(false);
      Serial.print("ads NDT");
    }
 
  #endif

  if (rtc.begin()){                             // Inicializa el RTC en teoría en address 0x68
    printLCD(8, 0, F("RTC OK"));
    Serial.print("RTC  OK");
  } else{
      printLCD(8, 0, F("RTC NDT")); app_health_set_ok(false);
      Serial.print("RTC  NDT");
    }

  app_measurements_set_temp_c(static_cast<int>(analogRead(TEMP_SNSR) * TEMP_CONVERSION_FACTOR)); // Convertir a Celsius con factor para ADC@0dB (Vmax≈1.1V)
    
  printLCD_S(11, 1, String(app_measurements_temp_c()) + String((char)0xDF) + "C");

  if (app_health_is_ok() && app_measurements_temp_c() <= 99) {
    printLCD(0,2, F("Sensor Test OK"));
  } else {
    printLCD(0,2, F("Sensor Test FAIL"));
    }

  delay(2000);
 
  //----------------------------------------Configuraciones iniciales de única vez----------------------------------------------

  //dac.setVoltage(0,false);     // IMPORTANTE, grabar en 0 para que apague la carga apenas se encienca. Cambio a "True" para que guarde este valor en la Emprom.
  if (!rtc.isrunning()) {
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__))); // sincroniza con la hora de compilación
  }

  //------------------------------------Pantalla Inicio--------------------------------------------------
  DateTime now = rtc.now();                                                                   // Obtiene la fecha y hora actual del RTC
  String date = String(now.day()) + "/" + String(now.month()) + "/" + String(now.year());     // Formatea la fecha
  String time = String(now.hour()) + ":" + String(now.minute()) + ":" + String(now.second()); // Formatea la hora

  clearLCD();
  // tft.fillScreen(TFT_BLACK);
  printLCD(0, 0, F("*DC Electronic Load*"));
  printLCD_S(0, 1, date + " - " + time);
  // tft.setCursor(0 * cellW, 1 * cellH); tft.print(date + " - " + time);
  #ifndef WOKWI_SIMULATION
  printLCD(0, 2, F("By Guy Nardin"));
  #else
  printLCD(0, 2, F("SIMULACION"));
  #endif
  printLCD(0, 3, F("v2.10b")); // Prueba migración a TFT

  #ifndef WOKWI_SIMULATION
  delay (2000);
  #else
  delay(1000);     //Para probar mas rapido
  #endif

  //---------------------------------------Chequea y Muestra los límites configurados----------------------
  #ifndef WOKWI_SIMULATION
    app_limits_set_current_cutoff(Load_EEPROM(ADD_CURRENT_CUT_OFF)); // Carga CurrentCutOff desde la EEPROM
    app_limits_set_power_cutoff(Load_EEPROM(ADD_POWER_CUT_OFF));     // Carga PowerCutOff desde la EEPROM
    app_limits_set_temp_cutoff(Load_EEPROM(ADD_TEMP_CUT_OFF));       // Carga tempCutOff desde la EEPROM
    if (app_limits_current_cutoff() <= 1 || app_limits_current_cutoff() > 10 ||   // Chequea que los valores de los límites estén en el rango correcto, pudieron quedar en 1 si eran nan.
        app_limits_power_cutoff() <= 1 || app_limits_power_cutoff() > 300 ||
        app_limits_temp_cutoff() < 30 || app_limits_temp_cutoff() > 99)
    {
      Config_Limits();
    }
    Show_Limits();
    Load_Calibration();
    delay(2000);
    clearLCD();

  #else
    // Simula que se cargan los valores de la EEPROM
    app_limits_set_current_cutoff(10);
    app_limits_set_power_cutoff(300);
    app_limits_set_temp_cutoff(80);
    Load_Calibration();
  #endif

  app_init();
}

//------------------------------------- Bucle Principal-------------------------------------------------
void loop() {
  app_run_cycle();
}

