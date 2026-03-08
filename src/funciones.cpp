#include "variables.h"
#include "funciones.h"
#include "app/app_loop.h"
#include "app/app_msc.h"
#include "app/app_keypad.h"
#include "app/app_inputs.h"
#include "app/app_value_input.h"
#include "core/core_modes.h"
#include "legacy/legacy_mode_cc.h"
#include "legacy/legacy_mode_cp.h"
#include "legacy/legacy_mode_cr.h"
#include "legacy/legacy_mode_bc.h"
#include "legacy/legacy_mode_ca.h"
#include "legacy/legacy_mode_transient.h"
#include "legacy/legacy_mode_limits.h"
#include "legacy/legacy_timing_buzzer.h"

//----------------------------- Load ON Status ------------------------------------
void Load_OFF(void) {
  #ifndef WOKWI_SIMULATION
  dac.setVoltage( 0, false);    // Corta inmediatamente.
  toggle = false;               // Flag off
  setCurrent = 0;               // Resetea por las dudas.
  #else
  toggle = false;               // Flag off
  setCurrent = 0;               // Resetea por las dudas.
  #endif
}

//---------------------------- Encoder Status -------------------------------------
void Encoder_Status(bool encOnOff, float limit) {
  if (encOnOff) {
    CuPo = 8;                              // Posición inicial del cursor
    reading = 0;
    encoderPosition = 0;                   // Reinicia tu variable lógica
    maxReading = limit;
    maxEncoder = maxReading * 1000;        // Escala idéntica a la original

    encoder.clearCount();
  } else {
    encoder.clearCount();                 // Ver si hay otra forma de detener el encoder
  }
}

//---------------------------- Encoder Decoder ------------------------------------
void Read_Encoder() {
  app_read_encoder();
}

//---------------------------- Read Keypad Input ----------------------------------
void Read_Keypad(int col, int row) {
  app_read_keypad(col, row);
}

//----------------------- Toggle Current Load ON or OFF ----------------------------
void Read_Load_Button(void) {
  app_read_load_button();
}
//----------------------- Key input used for UserSetUp ------------------------------- 
bool Value_Input(int col, int row, int maxDigits, bool decimal) {
  return app_value_input(col, row, maxDigits, decimal);
}

//---------------------------- Temperature Control ----------------------------------
void Temp_Control(void) {

  static unsigned long fan_on_time = 0;  // Tiempo de encendido del ventilador
  static unsigned long last_tmpchk = 0;  // Última vez que se chequeó la temperatura
  static bool fans_on = false;           // Estado del ventilador

  unsigned long currentMillis = millis(); 
  if ((currentMillis - last_tmpchk) < TMP_CHK_TIME) return; // Solo cada TMP_CHK_TIME milisegundos
  
  last_tmpchk = currentMillis;                              // Actualizar el momento de chequeo de temperatura

  temp = analogRead(TEMP_SNSR) * TEMP_CONVERSION_FACTOR;    // Convertir a Celsius

  if (temp >= 40) {
    if (!fans_on) { // Solo encender si está apagado
      digitalWrite(FAN_CTRL, HIGH);
      fans_on = true;
    }
    fan_on_time = currentMillis; // Actualizar el tiempo de encendido
  } else if (fans_on && (currentMillis - fan_on_time) >= FAN_ON_DRTN) { // lo mantiene encendido por un tiempo, sino flickea el cooler
    digitalWrite(FAN_CTRL, LOW);
    fans_on = false;
  }
  // Actualiza la Temperatura solo cuando la chequea
  setCursorLCD( 16, 0);
  if (temp < 10){printLCDRaw(" ");}
  printLCDRaw(temp);
  printLCDRaw(char(0xDF)); printLCDRaw("C");
}

//--------------------------- Check and Enforce Limits ------------------------------
void Check_Limits() {
  char message[20] = "";
  float power = voltage * current;                       // Estima la potencia disipada en los MOSFET pero sin contar las Rshunt de cada Mosfet
  float maxpwrdis = constrain(249 - 1.4 * temp, 0, 214); // Limite por MOSFET IRFP250N, resumida de 214 - 1.4 * (temp - 25)
  float actpwrdis = max(0.0f, power / 4);                // Por los 4 MOSFET IRFP250N
  bool vlimit = false;
  bool ilimit = false;
  bool plimit = false;
  bool climit = false;

  if (voltage > MAX_VOLTAGE) {strcpy(message, "Max Voltage!      "); vlimit = true;}
  else if (current > CurrentCutOff * 1.01) {strcpy(message, "Current Cut Off!  "); ilimit = true;} // 1% adicional (Toleración de Calibración máxima)
  else if (power > PowerCutOff) {strcpy(message, "Power Cut Off!    "); plimit = true;}
  else if (temp > tempCutOff) {strcpy(message, "Over Temperature! "); climit = true;}
  else if (actpwrdis >= maxpwrdis) strcpy(message, "Max PWR Disipation");

  if (strlen(message) > 0){
    Load_OFF();                         // Si hubo mensaje, apagar la carga ASAP.
    reading = 0; encoderPosition = 0;   // Reset de inputs
    encoder.clearCount();
    setCurrent = 0;                     // Todo a 0 para asegurar el apagado
    for (int i = 0; i < 6; i++) {       // Parpaderá el mensaje tres veces
      printLCD_S(0, 3, message);
      if (vlimit){Print_Spaces(12, 1);}
      else if(ilimit){Print_Spaces(5, 1);}
      else if(plimit){Print_Spaces(19, 1);}
      else if(climit){Print_Spaces(19,0);}
      delay(250);
      Print_Spaces(0, 3, 18);
      if (vlimit){setCursorLCD(12,1); printLCDRaw(F("v"));}
      else if(ilimit){setCursorLCD(5,1); writeLCD(byte(0));}
      else if(plimit){setCursorLCD(19,1); printLCDRaw(F("w"));}
      else if(climit){setCursorLCD(19,0); printLCDRaw(F("C"));}
      delay(250);
    }
    Reset_Input_Pointers();           
    modeInitialized = false;      // Avisa a los modos que se deben inicializar dado que se execdio un limite
  }
}

//------------------------------- Cursor Position -----------------------------------
void Cursor_Position(void) {
  static uint32_t lastPressTime = 0;  // Para evitar bloqueo por delay()
  constexpr int unitPosition = 8;     // Posicion base del cursor.
  static int last_CuPo = -1;          // Posicion previa del cursor

  // Verifica boton del encoder y delega al core en modos ya desacoplados.
  if (digitalRead(ENC_BTN) == LOW && millis() - lastPressTime > 200) {
      lastPressTime = millis();
      if (core_mode_is_managed(static_cast<uint8_t>(Mode))) {
        app_push_action(ActionType::EncoderButtonPress, 0, '\0');
      } else {
        CuPo++;  // Legacy para modos aun no desacoplados.
      }
  }

  if (last_CuPo == CuPo) return;

  // En modos gestionados por core, cursor/factor se normalizan en core.
  if (core_mode_is_managed(static_cast<uint8_t>(Mode))) {
    last_CuPo = CuPo;
    setCursorLCD(CuPo, 2);
    return;
  }

  // Saltar el punto decimal
  if (CuPo > last_CuPo && CuPo == unitPosition + 1) CuPo++;
  if (CuPo < last_CuPo && CuPo == unitPosition + 1) CuPo--;

  // Volver a la posicion inicial si excede el rango permitido
  if ((Mode == CC || Mode == BC || Mode == CA) && CuPo > 12) CuPo = unitPosition;
  if ((Mode == CC || Mode == BC || Mode == CA) && CuPo < 8) CuPo = unitPosition + 4;
  if ((Mode == CP || Mode == CR) && CuPo > 10) CuPo = unitPosition - 2;
  if ((Mode == CP || Mode == CR) && CuPo < 6) CuPo = unitPosition + 2;

  // Asignar factor segun la posicion del cursor y el modo
  switch (CuPo) {
      case 6:   factor = 100000;  break;  // Centenas (Solo CP y CR)
      case 7:   factor = 10000;   break;  // Decenas (Solo CP y CR)
      case 10:  factor = 100;     break;  // Decimas
      case 11:  factor = 10;      break;  // Centesimas (Solo CC, BC y CA)
      case 12:  factor = 1;       break;  // Milesimas (Solo CC, BC y CA)
      default:  factor = 1000;            // Unidades por defecto CuPo = 8
  }
  last_CuPo = CuPo;

  setCursorLCD(CuPo, 2);
}

//--------------------------- Read Voltage and Current ------------------------------
void Read_Volts_Current(void) {

  #ifndef WOKWI_SIMULATION

  float raw_voltage;
  float raw_current;

  // static float multiplier = 0.1875F; /* ADS1115  @ +/- 6.144V gain (16-bit results) */
  // ads.setGain(GAIN_TWOTHIRDS);  // 2/3x gain +/- 6.144V  1 bit = 0.1875mV
  // ads.setGain(GAIN_ONE);        // 1x gain   +/- 4.096V  1 bit = 0.125mV
  // ads.setGain(GAIN_FOUR);       // 4x gain   +/- 1.024V  1 bit = 0.03125mV
  // ads.setGain(GAIN_EIGHT);      // 8x gain   +/- 0.512V  1 bit = 0.015625mV
  // ads.setGain(GAIN_SIXTEEN);    // 16x gain  +/- 0.256V  1 bit = 0.0078125mV
  
  ads.setGain(GAIN_TWOTHIRDS);    // Setea ganancia a 2/3 para medir hasta +-6.144V ya que Opamp 741 esta a 5V
  adcv = ads.readADC_SingleEnded(VLTG_SNSR);
  raw_voltage = ads.computeVolts(adcv) * SNS_VOLT_FACT; // Factor de diseño para el ADC para 32V Max

  voltage = raw_voltage * Sns_Volt_Calib_Fact + Sns_Volt_Calib_Offs;       // Calibracion fina de voltaje

  ads.setGain(GAIN_ONE);          // Setea ganancia a 1x para medir hasta +-4.096V que sería IMAX 16A
  adci = ads.readADC_SingleEnded(CRR_SNSR);
  raw_current = ads.computeVolts(adci) * SNS_CURR_FACT; // Factor de diseño para Placa power V2 con Rshunt de 1ohm en cada Mosfet

  current = raw_current  * Sns_Curr_Calib_Fact + Sns_Curr_Calib_Offs;      // Calibracion fina de corriente
  
  #else

  // Simulación de Voltaje sensado

  int potValue = analogRead(VSIM);  // Leer el potenciómetro en pin 33 para simular el voltaje de carga. Ajusta el pin según tu conexión.
  static float simulatedVoltage = 0;  // Variable persistente para almacenar el voltage simulado
  static unsigned long lastDecreaseTime = 0;  // Variable para medir el tiempo del último decremento
  unsigned long currentMillis = millis();
  
  if (Mode != BC && Mode != CA){                               // Para todos los demas modos
    simulatedVoltage = map(potValue, 0, 1023, 550, 0) / 10.0;  // Convertir el rango 0-1023 a 55V-0V
    voltage = simulatedVoltage;                                // Asigna el valor de v simulado
  }
  else if (Mode == BC) { 
    // En modo BC Si toggle es true, reduce el voltaje simulado en 0.01V. Luego ver si puede ser proporcional a la corriente
    if (toggle && (currentMillis - lastDecreaseTime >= 2000)) {
    lastDecreaseTime = currentMillis;               // Actualizar el último tiempo de decremento
    simulatedVoltage -= 0.005;                      // Reducir el voltaje a medida que pasa el tiempo
    simulatedVoltage = max(simulatedVoltage, 0.0f);  // Asegurar que no sea negativo
    voltage = simulatedVoltage;                     // Asigna el valor de v simulado
    }
    else if (!toggle) {
      simulatedVoltage = map(potValue, 0, 1023, 550, 0) / 10.0;  // Convertir el rango 0-1023 a 55V-0V
      voltage = simulatedVoltage; 
    }
  }
  else if (Mode == CA){
    simulatedVoltage = map(potValue, 0, 1023, 550, 0) / 10.0;              // Convertir el rango 0-1023 a 55V-0V
    float error_voltage = simulatedVoltage * 1.05 - 0.1;                   // Asigna el valor de v simulado con error del 5% por arriba y offset
    voltage = error_voltage * Sns_Volt_Calib_Fact + Sns_Volt_Calib_Offs;  
  }
  
  // Simulación de corriente sensada.

  if (toggle) {
    current = setCurrent / 1000;   // Lo pasa a Amperes.
  } else {
    current = 0;
    }

  #endif
}

//------------------------- DAC Control Voltage for Mosfet --------------------------
void DAC_Control(void) {
  #ifndef WOKWI_SIMULATION

  if (toggle) {
    //setDAC = setCurrent * OUT_CURR_FACT * Out_Curr_Calib_Fact + Out_Curr_Calib_Offs;
    setDAC = (unsigned long)(constrain(setCurrent * Out_Curr_Calib_Fact + Out_Curr_Calib_Offs, 0.0f, 12000.0f) * OUT_CURR_FACT);  // Calcula valor de salida para el DacI con los factores y offset
    dac.setVoltage(setDAC, false);        // Setea corriente máxima de salida por el factor y POR EL MOMENTO, no lo graba en la Eprom del DacI.
  } else {
    dac.setVoltage(0, false); // set DAC output voltage to 0 if Load Off selected
    setCurrent = 0;           // ##IMPORTANTE#  Que el modo se encargue de resetearlo si lo necesita.
  }

  #else
  if (!toggle) {setCurrent = 0;}
  #endif

}

//----------------------- Select Constant Current LCD set up ------------------------
void Const_Current_Mode(void) {
  legacy_const_current_mode();
}

//------------------------ Select Constant Power LCD set up -------------------------
void Const_Power_Mode(void) {
  legacy_const_power_mode();
}

//---------------------- Select Constant Resistance LCD set up ----------------------
void Const_Resistance_Mode(void) {
  legacy_const_resistance_mode();
}

//-------------------- Select Battery Capacity Testing LCD set up -------------------
void Battery_Mode(void) {
  legacy_battery_mode();
}

//-------------------- Battery Type Selection and Cutoff Setup ----------------------
void Battery_Type_Selec() {
  legacy_battery_type_selec();
}

//---------------------- Battery Capacity Discharge Routine -------------------------
bool Battery_Capacity() {
  return legacy_battery_capacity();
}

//---------------------------- Transcient Continuos Mode ----------------------------
void Transient_Cont_Mode(void) {
  legacy_transient_cont_mode();
}

//--------------------------------- Transient Mode ----------------------------------
void Transient_Cont_Setup(void) {
  legacy_transient_cont_setup();
}

//----------------------------- Transcient Continuos Timing -------------------------
void Transcient_Cont_Timing() {
  legacy_transcient_cont_timing();
}

//------------------------------ Transcient List Mode -------------------------------
void Transient_List_Mode(void) {
  legacy_transient_list_mode();
}

//------------------------------ Transcient List Setup -------------------------------
void Transient_List_Setup() {
  legacy_transient_list_setup();
}

//------------------------------ Transcient List Timing ------------------------------
void Transient_List_Timing(void) {
  legacy_transient_list_timing();
}

//------------------------------ User set up for limits ------------------------------
void Config_Limits(void)
{
  legacy_config_limits();
}

//----------------- Show limits Stored Data for Current, Power and Temp --------------
void Show_Limits(void) {
  legacy_show_limits();
}

//--------------------------------- Calibration Mode ---------------------------------
void Calibration_Mode() {
  legacy_calibration_mode();
}

//--------------------------------- Calibration Setup --------------------------------
void Calibration_Setup(void){
  legacy_calibration_setup();
}

//--------------------------------- Calibrate ---------------------------------------
void Calibrate(float realValue){
  legacy_calibrate(realValue);
}

//------------------------------- Reset Input Pointers --------------------------------
void Reset_Input_Pointers (void){
  app_reset_input_pointers();
}

//------------------------------- Handle Buzzer --------------------------------------
void beepBuzzer(void) {
  legacy_beep_buzzer();
}

//-------------------------- Funciones para el Timer (RTC) ---------------------------
void timer_start() {
  legacy_timer_start();
}

void timer_stop() {
  legacy_timer_stop();
}

void timer_reset() {
  legacy_timer_reset();
}

float timer_getTotalSeconds() {
  return legacy_timer_get_total_seconds();
}

String timer_getTime() {
  return legacy_timer_get_time();
}















