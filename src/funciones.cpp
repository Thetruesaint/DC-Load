#include "variables.h"
#include "funciones.h"
#include <EEPROM.h>
//#undef WOKWI_SIMULATION

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
    CuPo = 8;                              // Inbicializa la posicion del cursor, puede venir de otro modo.
    reading = 0; encoderPosition = 0;      // Resetea la posición del encoder y cualquier valor de reading
    maxReading = limit;                    // Asigna el limite
    maxEncoder = maxReading * 1000;        // Lo recalcula para el encoder
    attachInterrupt(digitalPinToInterrupt(ENC_A), Read_Encoder, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC_B), Read_Encoder, CHANGE);
  } else {
      detachInterrupt(digitalPinToInterrupt(ENC_A));
      detachInterrupt(digitalPinToInterrupt(ENC_B));
  }
}

//---------------------------- Encoder Decoder ------------------------------------
void Read_Encoder()
{
  static uint8_t old_AB = 3;
  static int8_t encval = 0;
  static const int8_t enc_states[] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};

  old_AB <<= 2;
  if (digitalRead(ENC_A)) old_AB |= 0x02;
  if (digitalRead(ENC_B)) old_AB |= 0x01;

  encval += enc_states[(old_AB & 0x0F)];

  if (abs(encval) > 3) {
    encoderPosition += (encval > 0 ? factor : -factor);
    encval = 0;
  }
  encoderPosition = constrain(encoderPosition, 0, maxEncoder);
}

//---------------------------- Read Keypad Input ----------------------------------
void Read_Keypad(int col, int row) {
  int maxDigits = (Mode == CA) ? 6 : 5;
  customKey = customKeypad.getKey();              // Escanea el teclado
  
  if (customKey == NO_KEY) return;                // Si no hay tecla presionada, sale de la función
  
  if (!Handle_MSC_Keys (customKey)) {return;};
  
  // Solo en los modos CC, CP y CR.

  if (Mode == BC || Mode == TC || Mode == TL) return;
  // Números
  if (customKey >= '0' && customKey <= '9' && index < maxDigits) {  // Si la tecla presionada es un número, se permiten hasta 5 caracteres
    printLCD_S(col + index, row, String(customKey));        // Muestra el número en el LCD
    numbers[index++] = customKey;                           // Almacena el número en la variable numbers
    numbers[index] = '\0';                                  // Agrega el caracter nulo al final de la cadena
  }
  // Pto. decimal
  if (customKey == '.' && decimalPoint != '*'&& index < maxDigits) {   // Si punto decimal y no se ha ingresado uno antes y si no se llego al limite de carga
    printLCD(col + index, 3, F("."));              // Muestra el punto decimal en el LCD
    numbers[index++] = '.';                        // Almacena el punto decimal en la variable numbers
    numbers[index] = '\0';                         // Agrega el caracter nulo al final de la cadena
    decimalPoint = '*';                            // Marca que se ingresó un punto decimal
  }
  // Enter 
  if (customKey == 'E' && index != 0) { // Confirmar entrada solo si hay un valor cargado.
    x = atof(numbers);                  // Convierte cadena de caracteres en número y lo asigna a reading 
    if (Mode != CA){
    reading = x;                        // Convierte cadena de caracteres en número y lo asigna a reading 
    encoderPosition = reading * 1000;   // Asigna el valor a la variable encoderPosition
    }
    else {Calibrate(x);} //usa el valor de x para calibrar
    Print_Spaces(col, row, maxDigits);
    Reset_Input_Pointers();             // Resetea el punto decimal y el indice
  }

  // **Manejo de borrado**
  if (customKey == '<' && index > 0) {  
    index--;  
    if (numbers[index] == '.') decimalPoint = ' '; // Si borramos un punto, permitimos otro  
    numbers[index] = '\0';  
    Print_Spaces(col + index, row);
  }
}

// ------------------------------ Mode Selection --------------------------------------
void Mode_Selection(bool shiftPressed, char key) {
  Load_OFF();      // Apaga la carga siempre al cambiar o resetear modo
  Reset_Input_Pointers();                     // Resetea el punto decimal y el indice
  modeInitialized = false;    // Fuerza reinicialización del modo

  if (!shiftPressed) {        // Cambio cíclico con tecla 'M'
    functionIndex = (functionIndex + 1) % 6;
    switch (functionIndex) {
      case 0: Mode = CC; break;
      case 1: Mode = CP; break;
      case 2: Mode = CR; break;
      case 3: Mode = BC; modeConfigured = false; break; // BC necesita reconfiguración
      case 4: Mode = TC; modeConfigured = false; break; // TC necesita reconfiguración
      case 5: Mode = TL; modeConfigured = false; break; // TL necesita reconfiguración
    }
  } else {                    // Shift está activo
    switch (key) {
      case '1': functionIndex = 0; Mode = CC; break;
      case '2': functionIndex = 1; Mode = CP; break;
      case '3': functionIndex = 2; Mode = CR; break;
      case '4': functionIndex = 3; Mode = BC; modeConfigured = false; break;
      case '5': functionIndex = 4; Mode = TC; modeConfigured = false; break;
      case '6': functionIndex = 5; Mode = TL; modeConfigured = false; break;
      case 'C': Mode = CA; modeConfigured = false; break; // Activa modo Calibración con Shift+C
      case '<': modeConfigured = false; break; // Solo resetea el modo actual
      default: return; // Ignora otras teclas con Shift
    }
  }
}

//----------------------- Toggle Current Load ON or OFF ----------------------------
void Read_Load_Button(void) {
  if (digitalRead(LOADONOFF) == LOW) {
    delay(200); // Anti-rebote
    toggle = !toggle;
    if (!toggle) {
      #ifndef WOKWI_SIMULATION
      dac.setVoltage( 0, false);    // Corta inmediatamente.
      #endif
      setCurrent = 0;               // Resetea por las dudas.
    } 
  }
}

//----------------------- Key input used for UserSetUp ------------------------------- 
bool Value_Input(int col, int row, int maxDigits, bool decimal) {  
  
  //static bool shiftPressed = false;

  Reset_Input_Pointers();         // Resetea los valores de entrada
  lcd.setCursor(col, row);        // Ubica el cursor en la posición especificada
   lcd.blink_on();   // Muestra el cursor para la entrada

  while (true) { 
      customKey = Wait_Key_Pressed(); // Leer entrada de teclado

      if (!Handle_MSC_Keys (customKey)) {return false;};

      if (customKey >= '0' && customKey <= '9') { 
        if (index < maxDigits) { 
          numbers[index++] = customKey;
          numbers[index] = '\0'; 
        }
      } 
      else if (customKey == '.' && decimalPoint != '*' && decimal) {  // solo si esta habilitado
        if (index < maxDigits) {
          numbers[index++] = '.';
          numbers[index] = '\0';
          decimalPoint = '*'; 
        }
      } 
      else if (customKey == '<' && index > 0) { // Borrar último carácter ingresado
          index--;  
          if (numbers[index] == '.') decimalPoint = ' '; 
          numbers[index] = '\0';
      } 
      else if (customKey == 'E') {  // Confirmar entrada
          if (index > 0) {  
              x = atof(numbers); 
              Reset_Input_Pointers();  
              lcd.noCursor(); lcd.blink_off();
              return true;  
          }
      }

      // Borra para evitar residuos en pantalla
      printLCD_S(col, row, String("     ").substring(0, maxDigits));
      printLCD_S(col, row, String(numbers)); 
  }
}

//------------ Calculate and Display Actual Voltage, Current, and Power ------------
void Update_LCD(void) {
  static unsigned long lastUpdateTime = 0;
  static int blink_cntr = 0;

  if(!modeInitialized) return;  // No actualiza el LCD hasta que el modo dibuje la plantilla y ponga modeInitialized = true

  // Esperar 100ms antes de actualizar el resto del codigo en el LCD
  if (millis() - lastUpdateTime < LCD_RFSH_TIME) return;  
  lastUpdateTime = millis();  // Actualizar el tiempo de referencia
  
  // Evitar valores negativos por errores de medición
  
  if (voltage < 0.011 && Mode != CA) voltage = 0.0; // Ruido electrico pero quiero verlo en CA
  if (current < 0.006 && Mode != CA) current = 0.0; // Ruido electrico pero quiero verlo en CA
  float power = voltage * current;

  printLCD(8, 0, toggle ? F("ON ") : F("OFF"));  // Indica el estado de la carga

  // Imprimir los valores actualizados, ojo con W que si se corre puede afectar a col 0, row 3
  printLCDNumber(0, 1, current, 'A', (current < 10.00) ? 3 : 2);
  printLCDNumber(7, 1, voltage, 'v', (voltage < 10.00) ? 3 : (voltage < 100.0) ? 2 : 1);
  if (Mode != BC && Mode != CA) {   // lo reemplazo por BatteryCutoffVolts y en modo CA muestro el Punto a muestrar.
    lcd.setCursor(14,1);
    if (power < 10) {Print_Spaces(14, 1); lcd.print(power, 2);} 
    else if (power < 100) {lcd.print(power, 2);} 
    else {lcd.print(power, 1);}
    lcd.setCursor(19,1);
    lcd.print(F("w"));
  }

  if (Mode != TC && Mode != TL) {  // Evitar mostrar el encoder en modos transitorios

    lcd.setCursor(6, 2);
    if (Mode == CC || Mode == BC || Mode == CA){
      if (reading < 100) Print_Spaces(6, 2);
      if (reading < 10) Print_Spaces(7, 2);
      lcd.print(reading, 3);
    } else {
      if (reading < 100) lcd.print("0");
      if (reading < 10) lcd.print("0");
      lcd.print(reading, 1);
    }
    lcd.setCursor(CuPo, 2); // Cursor en la unidad a modificar

    blink_cntr = (blink_cntr + 1) % 5;  
    lcd.setCursor(CuPo, 2);
    if (blink_cntr == 4) {  // Solo en el último ciclo de cada 500ms imprime espacio
      Print_Spaces(CuPo, 2);
    }
  }
}

//---------------------------- Temperature Control ----------------------------------
void Temp_Control(void) {

  static unsigned long fan_on_time = 0;  // Tiempo de encendido del ventilador
  static unsigned long last_tmpchk = 0;  // Última vez que se chequeó la temperatura
  static bool fans_on = false;           // Estado del ventilador

  unsigned long currentMillis = millis(); 
  if ((currentMillis - last_tmpchk) < TMP_CHK_TIME) return; // Solo cada TMP_CHK_TIME milisegundos
  
  last_tmpchk = currentMillis;                              // Actualizar el momento de chequeo de temperatura

  #ifndef WOKWI_SIMULATION
  #define TEMP_CONVERSION_FACTOR 0.48828125 // Convierte lectura de LM35
  #else
  #define TEMP_CONVERSION_FACTOR 0.09765625 // Hasta 100°C con el pote de 0 a 5V que simula sensor de temperatura
  #endif

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
  lcd.setCursor( 16, 0);
  if (temp < 10){lcd.print(" ");}
  lcd.print(temp);
  lcd.print(char(0xDF)); lcd.print("C");
}

//--------------------------- Check and Enforce Limits ------------------------------
void Check_Limits() {
  char message[20] = "";
  float power = voltage * current;
  float maxpwrdis = constrain(140 - 0.80 * temp, 0, 120);     // Limite por MOSFET IRF540
  float actpwrdis = max(0, power / 4);                        // Por los 4 MOSFET IRF540
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
    setCurrent = 0;                     // Todo a 0 para asegurar el apagado
    for (int i = 0; i < 6; i++) {       // Parpaderá el mensaje tres veces
      printLCD_S(0, 3, message);
      if (vlimit){Print_Spaces(12, 1);}
      else if(ilimit){Print_Spaces(5, 1);}
      else if(plimit){Print_Spaces(19, 1);}
      else if(climit){Print_Spaces(19,0);}
      delay(250);
      Print_Spaces(0, 3, 18);
      if (vlimit){lcd.setCursor(12,1); lcd.print(F("v"));}
      else if(ilimit){lcd.setCursor(5,1); lcd.write(byte(0));}
      else if(plimit){lcd.setCursor(19,1); lcd.print(F("w"));}
      else if(climit){lcd.setCursor(19,0); lcd.print(F("C"));}
      delay(250);
    }
    Reset_Input_Pointers();           
    modeInitialized = false;      // Avisa a los modos que se deben inicializar dado que se execdio un limite
  }
}

//------------------------------- Cursor Position -----------------------------------
void Cursor_Position(void) {
  static uint32_t lastPressTime = 0;  // Para evitar bloqueo por delay()
  constexpr int unitPosition = 8;     // Posición base del cursor, constexpr hace lo mismo que const pero optimizado.
  static int last_CuPo = -1;          // Posición previa del cursor

  // Verifica si el botón fue presionado y hace debounce
  if (digitalRead(ENC_BTN) == LOW && millis() - lastPressTime > 200) { 
      lastPressTime = millis();
      CuPo++;  // Corre el cursor un lugar a la derecha
  }

  if (last_CuPo == CuPo) return;

  // Saltar el punto decimal
  if (CuPo == unitPosition + 1) CuPo++;

  // Volver a la posición inicial si excede el rango permitido
  if ((Mode == CC || Mode == BC || Mode == CA) && CuPo > 12) CuPo = unitPosition;
  if ((Mode == CP || Mode == CR) && CuPo > 10) CuPo = unitPosition - 2;

  // Asignar factor según la posición del cursor y el modo
  switch (CuPo) {
      case 6:   factor = 100000;  break;  // Centenas (Solo CP y CR)
      case 7:   factor = 10000;  break;   // Decenas (Solo CP y CR)
      case 10:  factor = 100;   break;    // Décimas
      case 11:  factor = 10;    break;    // Centésimas (Solo CC, BC y CA)
      case 12:  factor = 1;     break;    // Milésimas (Solo CC, BC y CA)
      default:  factor = 1000;            // Unidades por defecto CuPo = 8
  }
  last_CuPo = CuPo;

  lcd.setCursor(CuPo, 2);
}

//--------------------------- Read Voltage and Current ------------------------------
void Read_Volts_Current(void) {

  #ifndef WOKWI_SIMULATION

  float raw_voltage;
  float raw_current;

  /*
  Si querés ajustar la suavidad, podés cambiar el valor de alpha:
  Mayor (0.2 - 0.3) → Más rápido pero menos estable.
  Menor (0.05 - 0.1) → Más estable pero más lento.
  */

  // static float multiplier = 0.1875F; /* ADS1115  @ +/- 6.144V gain (16-bit results) */
  // ads.setGain(GAIN_TWOTHIRDS);  // 2/3x gain +/- 6.144V  1 bit = 0.1875mV
  // ads.setGain(GAIN_ONE);        // 1x gain   +/- 4.096V  1 bit = 0.125mV
  // ads.setGain(GAIN_FOUR);       // 4x gain   +/- 1.024V  1 bit = 0.03125mV
  // ads.setGain(GAIN_EIGHT);      // 8x gain   +/- 0.512V  1 bit = 0.015625mV
  // ads.setGain(GAIN_SIXTEEN);    // 16x gain  +/- 0.256V  1 bit = 0.0078125mV
  
  ads.setGain(GAIN_ONE);                                                   // 1x gain   +/- 4.096V  1 bit = 0.125mV
  adcv = ads.readADC_SingleEnded(VLTG_SNSR);
  raw_voltage = ads.computeVolts(adcv) * SNS_VOLT_FACT;                    // Por ampl. dif. para sensado remoto de (Max. 50V).

  voltage = raw_voltage * Sns_Volt_Calib_Fact + Sns_Volt_Calib_Offs;       // Calibracion fina de voltaje

  ads.setGain(GAIN_EIGHT);
  adci = ads.readADC_SingleEnded(CRR_SNSR);
  raw_current = ads.computeVolts(adci) * SNS_CURR_FACT;                    // Por ampl. dif. para sensado remoto de (Max. 5A).

  current = raw_current  * Sns_Curr_Calib_Fact + Sns_Curr_Calib_Offs;      // Calibracion fina de corriente
  
  #else

  // Simulación de Voltaje sensado

  int potValue = analogRead(A2);  // Leer el potenciómetro en A2
  static float simulatedVoltage = 0;  // Variable persistente para almacenar el voltage simulado
  static unsigned long lastDecreaseTime = 0;  // Variable para medir el tiempo del último decremento
  unsigned long currentMillis = millis();
  
  if (Mode != BC && Mode != CA){                                             // Para todos los demas modos
    simulatedVoltage = map(potValue, 0, 1023, 300, 0) / 10.0;  // Convertir el rango 0-1023 a 30V-0V
    voltage = simulatedVoltage;                                // Asigna el valor de v simulado
  }
  else if (Mode == BC) { 
    // En modo BC Si toggle es true, reduce el voltaje simulado en 0.01V. Luego ver si puede ser proporcional a la corriente
    if (toggle && (currentMillis - lastDecreaseTime >= 2000)) {
    lastDecreaseTime = currentMillis;               // Actualizar el último tiempo de decremento
    simulatedVoltage -= 0.005;                      // Reducir el voltaje a medida que pasa el tiempo
    simulatedVoltage = max(simulatedVoltage, 0.0);  // Asegurar que no sea negativo
    voltage = simulatedVoltage;                     // Asigna el valor de v simulado
    }
    else if (!toggle) {
      simulatedVoltage = map(potValue, 0, 1023, 300, 0) / 10.0;  // Convertir el rango 0-1023 a 30V-0V
      voltage = simulatedVoltage; 
    }
  }
  else if (Mode == CA){
    simulatedVoltage = map(potValue, 0, 1023, 300, 0) / 10.0;              // Convertir el rango 0-1023 a 30V-0V
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
    setDAC = setCurrent * OUT_CURR_FACT * Out_Curr_Calib_Fact + Out_Curr_Calib_Offs;  // Calcula valor de salida para el DacI con los factores y offset
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

  if (!modeInitialized) {
    lcd.clear();
    printLCD(0, 0, F("CC LOAD"));          // Muestra el titulo del modo
    printLCD(1, 2, F("Set->"));            // Muestra el mensaje
    printLCD(13, 2, F("A"));               // Muestra el mensaje
    printLCD(0, 3, F(">"));                // Indica la posibilidad de ingresar valores.
    Encoder_Status(true, CurrentCutOff);   // Encoder, CuPo =8, inic. y calcula maxReading y maxEncoder
    modeInitialized = true;                // Modo inicializado
  }
  reading = encoderPosition / 1000;            // Toma el valor del encoder
  reading = min(maxReading, max(0, reading));  // Limita reading dinámicamente a CurrentCutOff
  encoderPosition = reading * 1000.0;          // Actualiza encoderPosition para mantener consistencia
  Cursor_Position();

  if (!toggle) return;
  setCurrent = reading * 1000;             // lo pasa a mA 
}

//------------------------ Select Constant Power LCD set up -------------------------
void Const_Power_Mode(void) {

  if (!modeInitialized) {
    lcd.clear();
    printLCD(0, 0, F("CP LOAD"));          // Muestra el titulo del modo
    printLCD(0, 2, F("Set->"));            // Muestra el mensaje
    printLCD(11, 2, F("W"));               // Muestra el mensaje
    printLCD(0, 3, F(">"));                // Indica la posibilidad de ingresar valores.
    Encoder_Status(true, PowerCutOff);     // Encoder, CuPo =8, inic. y calcula maxReading y maxEncoder
    modeInitialized = true;                // Modo inicializado
  }
  reading = encoderPosition / 1000;            // Toma el valor del encoder
  reading = min(maxReading, max(0, reading));  // Limita reading dinámicamente a CurrentCutOff
  encoderPosition = reading * 1000.0;          // Actualiza encoderPosition para mantener consistencia
  Cursor_Position(); 

  if (!toggle) return;
  setPower = reading * 1000;               // Conversión a mW
  setCurrent = setPower / voltage;         // Conversión a mA 
}

//---------------------- Select Constant Resistance LCD set up ----------------------
void Const_Resistance_Mode(void) {

 if (!modeInitialized) {
    lcd.clear();
    printLCD(0, 0, F("CR LOAD"));           // Muestra el titulo del modo
    printLCD(0, 2, F("Set->"));             // Muestra el mensaje
    printLCD_S(11, 2, String((char)0xF4));  // Muestra el Símbolo de Ohms
    printLCD(0, 3, F(">"));                 // Indica la posibilidad de ingresar valores.
    Encoder_Status(true, MAX_RESISTOR);     // Encoder, CuPo =8, inic. y calcula maxReading y maxEncoder
    encoderPosition = MAX_RESISTOR * 1000;  // Lo inicializa en 999.9 Ω, valor mas seguro
    modeInitialized = true;                 // Modo inicializado
  }
  reading = encoderPosition / 1000.0;           // Convierte a ohms (Ω) el valor del encoder
  reading = min(maxReading, max(0.1, reading)); // Evita resistencia 0, mínimo 0.1Ω, maximo, maxReading
  encoderPosition = reading * 1000;             // pasa el valor a encoder si lo limitó
  Cursor_Position();
  
  if (!toggle) return;
  setResistance =  reading;                       // en ohms (Ω)
  setCurrent = (voltage / setResistance) * 1000;  // convirte a mA
}

//-------------------- Select Battery Capacity Testing LCD set up -------------------
void Battery_Mode(void) {
  
  if (!modeConfigured) {Battery_Type_Selec(); return;}  // Probar bien si aca no tiene problemas, en algunas pruebas queda fliqueando con carga en ON

  if (!modeInitialized){
    lcd.clear();
    printLCD(0, 0, F("BC LOAD"));
    lcd.setCursor(13,1);lcd.print(F(">"));
    printLCDNumber(14, 1, BatteryCutoffVolts, 'V', 2);  // Muestro el Cutoff Voltage
    printLCD(1, 2, F("Adj->"));
    printLCD(13, 2, F("A"));
    timer_reset();                  // Resetea el timer
    BatteryLife = 0;                            // Inicializa  la medición
    BatteryLifePrevious = 0;                    // Resetea la vida de la batería
    printLCDNumber(6, 3, BatteryLife, ' ', 0);  // Mostrar sin decimales
    lcd.print(F("mAh"));
    printLCD_S(14, 3, BatteryType);             // Muestro el tipo de Bateria.
    Encoder_Status(true, CurrentCutOff);        // Encoder, CuPo =8, inic. y calcula maxReading y maxEncoder
    modeInitialized = true;                     // Modo inicializado
  }
  
  if (BatteryLife > BatteryLifePrevious) {      // Solo si cambia el valor
    printLCDNumber(6, 3, BatteryLife, ' ', 0);
    lcd.print(F("mAh"));
    Print_Spaces(16, 2, 4);                    // Borra "Done ", porque si cambia es porque hubo acción manual.
    BatteryLifePrevious = BatteryLife;
  }
  if (Battery_Capacity()){printLCD(16,2, F("Done"));}
  Cursor_Position(); 
}

//-------------------- Battery Type Selection and Cutoff Setup ----------------------
void Battery_Type_Selec() {

//  bool shiftPressed = false;

  lcd.clear();                            // Borra la pantalla del LCD
  printLCD(2, 0, F("Set Task & Batt"));   // Muestra el título
  printLCD(0, 1, F("Stor. 1)LiPo 2)LiIOn"));
  printLCD(0, 2, F("Disc. 3)LiPo 4)LiIOn"));
  printLCD(2, 3, F("5)Cutoff Voltage"));

while (true) {  // Bucle para evitar la salida accidental
    customKey = Wait_Key_Pressed(); 

    if (!Handle_MSC_Keys (customKey)) {return;};

    switch (customKey) {
        case '1': BatteryCutoffVolts = LIPO_STOR_CELL_VLTG; BatteryType = "Li-Po"; break;
        case '2': BatteryCutoffVolts = LION_STOR_CELL_VLTG; BatteryType = "Li-Ion"; break;
        case '3': BatteryCutoffVolts = LIPO_DISC_CELL_VLTG; BatteryType = "Li-Po"; break;
        case '4': BatteryCutoffVolts = LION_DISC_CELL_VLTG; BatteryType = "Li-Ion"; break;
        case '5': BatteryType = "Custom"; break;
        default: continue;  // Evita salir si la tecla es inválida
    }
    break;  // Sale del bucle si se ingresó una tecla válida
}

  // Pedir ingresar un voltaje de corte
  if (BatteryType == "Custom") {
    lcd.clear();
    printLCD_S(3, 0, BatteryType + " Batt");
    printLCD(2, 1, F("Voltage Cutoff?"));
    printLCD(5, 2, F("(0.1-25)V"));
    do {
      z = 7; r = 3;
      printLCD(z - 1, r, F(">"));
      Print_Spaces(z , r, 5); // Borra el espacio si hubo un valor fuera de rango
     
      if (!Value_Input(z, r)) return;  // Sale si hubo selección de nuevo modo o reset de modo
    } while (x > 25 || x < 0.1);          // Mintras este fuera de rango, repite
      BatteryCutoffVolts = x;
  } 

  // Pedir ingresar la cantidad de celdas
  if (BatteryType != "Custom") {
    lcd.clear();
    printLCD_S(3, 0, BatteryType + " Batt");
    printLCD(6, 1, F("(1-6)S?"));

    do {
        z = 9; r = 2;
        printLCD(z - 1, r, F(">"));
        Print_Spaces(z , r, 5);                   // Borra el espacio si hubo un valor fuera de rango
        if (!Value_Input(z, r, 1, false)) return; //1 digito, sin decimal.
    } while (x < 1 || x > 6);                     // Asegura que solo se ingrese un número entre 1 y 6
    BatteryCutoffVolts *= x;                      // Multiplica por la cantidad de celdas
  }
  modeConfigured = true;          // Indica que se seteo el modo.
  modeInitialized = false;        // Pinta la plantilla BC en el LCD
}

//---------------------- Battery Capacity Discharge Routine -------------------------
bool Battery_Capacity() {
  
  float LoadCurrent = 0;
  unsigned long currentMillis = millis();
  static unsigned long lastUpdate = 0;    // Guarda el tiempo de la última actualización del display

  if (toggle && voltage >= BatteryCutoffVolts && !timerStarted) { timer_start(); } // Inicia el timer si la carga está activa
  if (!toggle && voltage >= BatteryCutoffVolts && timerStarted) { timer_stop(); }  // Detiene el timer si la carga está inactiva
 
  if (currentMillis - lastUpdate >= 500) { // Actualizar cada 1 segundo para evitar flickering
    lastUpdate = currentMillis;

    printLCD_S(0, 3, timer_getTime()); // Mostrar tiempo en LCD

    Seconds = timer_getTotalSeconds();
    LoadCurrent = (!timerStarted) ? 0 : current;
    BatteryLife += (LoadCurrent * 1000) / 7200; // LoadCurrent A, por horas, por 1000 = mAh
  }

  reading = encoderPosition / 1000;            // Toma el valor del encoder
  reading = min(maxReading, max(0, reading));  // Limita reading dinámicamente a CurrentCutOff
  encoderPosition = reading * 1000.0;          // Actualiza encoderPosition para mantener consistencia

  if (!toggle) return false;  //Si OFF, sale, el resto solo si ON Devuelve falso porque no se termino los criterios de descarga

  setCurrent = reading * 1000; // Toma el valor del encoder, por si quise hacer un ajuste, y lo setea en la carga en mA

  // Reducción progresiva de corriente cuando el voltaje alcanza al de corte

  if (voltage <= BatteryCutoffVolts) {  // Fase 2: Estabilización
    setCurrent = max(setCurrent - CRR_STEP_RDCTN, MIN_DISC_CURR);
    reading = setCurrent / 1000;
    encoderPosition = reading * 1000;
  }

  // Si el voltaje ya cayo por debajo de VoltageDropMargin, corta la carga (esto es porque la lipo se recopera sin carga)
  if (voltage <= (BatteryCutoffVolts - VLTG_DROP_MARGIN)) { 
    BatteryCurrent = current;   // Toma nota de la corriente mínima con la que quedo
    reading = 0; encoderPosition = 0; setCurrent = 0; // Reinicia todo.
    Load_OFF();
    timer_stop(); 
    beepBuzzer(); return true; // Devuelve true porque completo la descarga
  }
  return false; // Por si pasa otra cosa, devuelve falso
}

//---------------------------- Transcient Continuos Mode ----------------------------
void Transient_Cont_Mode(void) {

if(!modeConfigured) {Transient_Cont_Setup(); return;}   // Si no esta configurado, lo configura. Sale si no se configuro

  if (!modeInitialized) {                       // Si es falso, prepara el LCD
    printLCD_S(3, 2, String(LowCurrent, 3)); lcd.write(byte(0));
    printLCD_S(14, 2, String(HighCurrent, 3)); lcd.write(byte(0));
    printLCD(0, 0, F("TC LOAD"));               // Muestra el titulo del modo
    printLCD(0, 2, F("I1>"));                   // Muestra el mensaje
    printLCD(11, 2, F("I2>"));                  // Muestra el mensaje
    printLCD(2, 3, F("Time: "));                // Muestra el mensaje
    
    if (transientPeriod < 10) {Print_Spaces(7, 3, 4); lcd.print(transientPeriod);} 
    else if (transientPeriod < 100) {Print_Spaces(7, 3, 3); lcd.print(transientPeriod);}
    else if (transientPeriod < 1000) {Print_Spaces(7, 3, 2); lcd.print(transientPeriod);}
    else if (transientPeriod < 10000) {Print_Spaces(7, 3); lcd.print(transientPeriod);}
    else {lcd.setCursor(7, 3); lcd.print(transientPeriod);}

    //printLCD_S(8, 3, String(transientPeriod));  // Muestra el valor del tiempo
    printLCD(13, 3, F("mSecs"));                // Muestra la unidad
    setCurrent = 0;                             // por si quedo seteada del modo anterior
    modeInitialized = true;                     // Modo inicializado.
    Encoder_Status(false);                      // Deshabilitar el encoder
  }
  Transcient_Cont_Timing(); 
}

//--------------------------------- Transient Mode ----------------------------------
void Transient_Cont_Setup(void) {
  lcd.clear(); // Apaga el cursor y borra la pantalla del LCD

  printLCD(3, 0, F("TRANSIENT CONT."));
  printLCD(5, 1, F("I1(A)"));
  printLCD(5, 2, F("I2(A)"));
  printLCD(4, 3, F("dt(mS)"));

  z = 11;   // Alinea la Col de los Inputs.
  r = 1;
  if (!Value_Input(z, r)) return;            // Obtiene el valor ingresado por el usuario o sale del modo
  LowCurrent = min(x, CurrentCutOff);        // Limita la corriente baja al valor de corte de corriente
  printLCDNumber(z, r, LowCurrent, 'A', 3);  // Muestra el valor de la corriente baja

  r = 2;
  if (!Value_Input(z, r)) return;             // Limita la corriente baja al valor de corte de corriente
  HighCurrent = min(x, CurrentCutOff);        // Limita la corriente alta al valor de corte de corriente
  printLCDNumber(z, r, HighCurrent, 'A', 3);  // Muestra el valor de la corriente alta con tres decimales

  r = 3;
  if (!Value_Input(z, r, 5, false)) return;   // 5 digitos, sin decimal.
  transientPeriod = x;                        // Guarda el valor del tiempo de transitorio

  lcd.clear();                                // Borra la pantalla del LCD
  modeConfigured = true;                      // Se configuro el modo TC
  modeInitialized = false;                    // Pinta la plantilla TL en el LCD
}

//----------------------------- Transcient Continuos Timing -------------------------
void Transcient_Cont_Timing() {

  static unsigned long last_time = 0;
  static bool transient_cont_toggle = false;

  if (!toggle) {  // Con carga apagada, reseteo estado, porque sino empieza donde quedo, no le veo practicidad, muy inprevisto.
    last_time = 0;
    transient_cont_toggle = false;
    return;} 

  current_time = micros();

  if ((current_time - last_time) >= (transientPeriod * 1000.0)) { 
    last_time = current_time; 

    if (!transient_cont_toggle) {
      setCurrent = LowCurrent * 1000 ; } // lo convierte a mA
    else {
      setCurrent = HighCurrent * 1000 ;} // lo convierte a mA

    transient_cont_toggle = !transient_cont_toggle; // 🔹 Alterna el estado
  }
}

//------------------------------ Transcient List Mode -------------------------------
void Transient_List_Mode(void) {
  static unsigned int last_transientPeriod = -1;

  if(!modeConfigured) {Transient_List_Setup(); return;} // Si no esta configurado, lo configura. Sale si no se configuro

  if (!modeInitialized) {                 // Si es falso, prepara el LCD
    printLCD(0, 0, F("TL LOAD"));         // Muestra el titulo del modo
    printLCD(6, 2, F("Step: "));
    printLCD(13, 2, F("/"));
    printLCD_S(14, 2, String(total_steps));
    printLCD(4, 3, F("dt: "));
    printLCD(13, 3, F("mS"));          // Muestra la unidad
    modeInitialized = true;               // Modo inicializado.
    Encoder_Status(false);                // Deshabilitar el encoder
  }

  if(modeConfigured) {
    printLCD_S(12, 2, String(current_step));            // Paso en curso, de 0 a 9, asi no tengo que manejar el LCD, generando mas delay
      if (transientPeriod != last_transientPeriod) {  // Solo si cambió, evitando flickering
      Print_Spaces(8, 3, 5);
      printLCD_S(8, 3, String(transientPeriod));        // Nuevo valor con espacio extra
      last_transientPeriod = transientPeriod;
    }
  }
  Transient_List_Timing(); 
}

//------------------------------ Transcient List Setup -------------------------------
void Transient_List_Setup() {
  lcd.clear(); // Apaga el cursor y borra la pantalla
  // Pregunta por cuantos saltos se desean cargar
  printLCD(3, 0, F("TRANSIENT LIST"));
  printLCD(4, 1, F("Steps(2-10)?"));
  do { // Bucle para garantizar entrada válida
    z = 9; r = 2;
    printLCD(z - 1, r, F(">"));
    Print_Spaces(z, r, 2);
    if (!Value_Input(z, r, 2, false)) return; // 2 digitos, sin decimal.
  } while (x < 2 || x > 10);

  total_steps = x - 1;   // Guarda el número total de instrucciones

  // configurar cada paso:
  lcd.clear();
  for (int i = 0; i <= total_steps; i++) {      // Bucle para obtener los valores de la lista
    printLCD(3, 0, F("TRANSIENT LIST"));        // Mantengo el titulo para que se vea bien el modo que se está configurando
    printLCD_S(5, 1, "Set step " + String(i));  // Muestra la instrucción a configurar
    printLCD(0, 2, F("Current (A):"));          // Pide el valor de corriente en Amperes
    printLCD(0, 3, F("Time (mSec):"));          // Pide el valor de tiempo en milisegundos

    z = 13; r = 2;
    if (!Value_Input(z, r)) return;     // Permitir 5 digitos, ej.: 1.234 o salir del Modo
    x = min(x, CurrentCutOff);          // Limita a CutOff
    printLCDNumber(z, r, x, 'A', 3);    // Muestra el valor de la corriente
    transientList[i][0] = x * 1000;     // Lo guarda en la lista en mA

    z = 13; r = 3;                            // Ubica la toma del valor de mSec
    if (!Value_Input(z, r, 5, false)) return; // 5 digitos, sin decimal.
    transientList[i][1] = x;                  // Guarda el valor del tiempo en ms
    lcd.clear();                              // Borra la pantalla, para configurar la siguiente instrucción
  }
  setCurrent = 0;           // por si quedo seteada del modo anterior
  current_step = 0;         // Resetea el contador de pasos porque finalizo la configuración
  transientPeriod = transientList[current_step][1];      // Tambien el periodo a mostrar
  modeConfigured = true;    // Se configuro el modo TC
  modeInitialized = false;  // Pinta la plantilla TC en el LCD
}

//------------------------------ Transcient List Timing ------------------------------
void Transient_List_Timing(void) {

  static unsigned long last_time = 0;

  if (!toggle) {
    current_step = 0;
    last_time = 0;
    transientPeriod = transientList[current_step][1];
    return;} // Reinicio la lista

    current_time = micros();

  if (last_time == 0){
    setCurrent = transientList[current_step][0];       // Ya esta en mA
    transientPeriod = transientList[current_step][1];
    last_time = current_time; 
  }

  if ((current_time - last_time) >= transientPeriod * 1000) {
    current_step++;
    if (current_step > total_steps) { current_step = 0; }
    setCurrent = transientList[current_step][0];       // Ya esta en mA
    transientPeriod = transientList[current_step][1];
    last_time = current_time;
  }
}

//------------------------------ User set up for limits ------------------------------
void Config_Limits(void)
{
  Load_OFF();            // Apaga la carga
  Show_Limits();         // Muestra los actuales
  delay(2000);
  lcd.clear();

  printLCD(4, 0, F("Set Limits"));
  printLCD(0, 1, F("Current(A):"));
  z = 12; r = 1;
  if (!Value_Input(z, r)) return;                 // 5 digitos, ej.: 1.234, 9.999 o 10.00 salir del Modo
  CurrentCutOff = constrain(x, 1, MAX_CURRENT);   // Límite entre 1.000A y 10.00A
  printLCDNumber(z, r, CurrentCutOff,' ',3);lcd.print(F("A")); // A normal porque es valor fijo.
  
  printLCD(0, 2, F("Power(W):"));
  r = 2; z = 12;
  if (!Value_Input(z, r)) return;                 // 5 digitos, ej.: 1.234, 50.00 o 300.0 salir del Modo
  PowerCutOff = constrain(x, 1, MAX_POWER);       // Límite entre 1.000W y 300.0W
  printLCDNumber(z, r, PowerCutOff, 'W',1);
 
  printLCD(0, 3, F("Temp.("));
  printLCD_S(6, 3, String((char)0xDF) + "C):");
  z = 12; r = 3;
  if (!Value_Input(z, r, 2)) return;              // Permitir 2 digitos, ej.: 10 a 99 o salir del Modo
  tempCutOff = constrain(x, 30.0, MAX_TEMP);      // Límite entre 30°C y 99°C  
  printLCD_S(z, r, String(tempCutOff));

  Save_Calibration(ADD_CURRENT_CUT_OFF, CurrentCutOff);
  Save_Calibration(ADD_POWER_CUT_OFF, PowerCutOff);
  Save_Calibration(ADD_TEMP_CUT_OFF, tempCutOff);

  Show_Limits();
  delay(2000);
  modeInitialized = false;   // Pero lo inicializa. Ojo con los modos no preparados para esto como BC, TC y TL
}

//----------------- Show limits Stored Data for Current, Power and Temp --------------
void Show_Limits(void) {
  lcd.clear();
  
  // Los lee de la EEPROM
  CurrentCutOff = loadFromEEPROM(ADD_CURRENT_CUT_OFF);
  PowerCutOff = loadFromEEPROM(ADD_POWER_CUT_OFF);
  tempCutOff = loadFromEEPROM(ADD_TEMP_CUT_OFF);

  // Los muestra
  printLCD(1, 0, F("Limits")); // Muestra el titulo
  printLCD(0, 1, F("Current:"));
  // 3 decimales, ej.: 1.234, 9.999 o 10.00
  printLCDNumber(9, 1, CurrentCutOff,' ',3);lcd.print(F("A")); // A normal porque es valor fijo.
  printLCD(0, 2, F("Power:"));
  printLCDNumber(9, 2, PowerCutOff, 'W', 2);    // 2 decimales, ej.: 1.234, 50.00 o 300.0
  printLCD(0, 3, F("Temp.:"));
  printLCDNumber(9, 3, tempCutOff, ' ', 0);     // 0 decimales, ej.: 10 a 99
  lcd.print(char(0xDF)); lcd.print("C");
}

//--------------------------------- Calibration Mode ---------------------------------
void Calibration_Mode() {

  if(!modeConfigured) {Calibration_Setup(); return;}   // Si no esta configurado, lo configura. Sale si no se configuro
  
  if (!modeInitialized) {                    // Si es falso, prepara el LCD
    if (x == 1) {                 // Resetea Volt Calibration
      calibrateVoltage = true;
      Sns_Volt_Calib_Fact = 1.0;
      Sns_Volt_Calib_Offs = 0.0;}
    else if (x == 2){             // Resetea Curr Calibration
      calibrateVoltage = false;
      Sns_Curr_Calib_Fact = 1.0;
      Sns_Curr_Calib_Offs = 0.0;
      Out_Curr_Calib_Fact = 1.0;
      Out_Curr_Calib_Offs = 0.0;
    }
    lcd.clear();    
    printLCD(0, 0, calibrateVoltage ? F("CA VOLT") : F("CA CURR"));
    printLCD(14, 1, firstPointTaken? F("Set P2") : F("Set P1"));
    printLCD(1, 2, F("Adj->"));
    printLCD(13, 2, F("A"));
    printLCD(0, 3, F(">"));                 // Indica la posibilidad de ingresar valores.
    printLCD(7, 3, F("<Set real"));
    Encoder_Status(true, CurrentCutOff);    // Encoder, CuPo =8, inic. y calcula maxReading y maxEncoder
    modeInitialized = true;                 // Modo inicializado
  }
  reading = encoderPosition / 1000;            // Toma el valor del encoder
  reading = min(maxReading, max(0, reading));  // Limita reading dinámicamente a CurrentCutOff
  encoderPosition = reading * 1000.0;          // Actualiza encoderPosition para mantener consistencia
  Cursor_Position();

  if (!toggle) return;
  setCurrent = reading * 1000;                // lo pasa a mA
}

//--------------------------------- Calibration Setup --------------------------------
void Calibration_Setup(void){
  lcd.clear();
  printLCD(4, 0, F("CALIBRATION"));
  printLCD(0, 1, F("1-Voltage 2-Current"));
  printLCD(0, 2, F("3-Load    4-Save"));
  do { // Bucle para garantizar entrada válida
    z = 1; r = 3;
    printLCD(z - 1, r, F(">"));
    Print_Spaces(z, r, 1);
    if (!Value_Input(z, r, 1, false)) return; // 1 digitos, sin decimal.
  } while (x < 1 || x > 4);
  modeConfigured = true;    // Se configuro el modo CA
  if (x == 3){
    Load_Calibration(ADD_SNS_VOLT_FAC_CAL, Sns_Volt_Calib_Fact);    // Carga Factor de Calibración de la EEPROM para sensado de Voltaje
    Load_Calibration(ADD_SNS_CURR_FAC_CAL, Sns_Curr_Calib_Fact);    // Carga Factor de Calibración de la EEPROM para sensado de Corriente
    Load_Calibration(ADD_OUT_CURR_FAC_CAL, Out_Curr_Calib_Fact);    // Carga Factor de Calibración de la EEPROM para seteo de Corriente
    Load_Calibration(ADD_SNS_VOLT_OFF_CAL, Sns_Volt_Calib_Offs);    // Carga Offset de Calibración de la EEPROM para sensado de Voltaje
    Load_Calibration(ADD_SNS_CURR_OFF_CAL, Sns_Curr_Calib_Offs);    // Carga Offset de Calibración de la EEPROM para sensado de Corriente
    Load_Calibration(ADD_OUT_CURR_OFF_CAL, Out_Curr_Calib_Offs);    // Carga Offset de Calibración de la EEPROM para seteo de Corriente
    printLCD(12, 3, F("Loaded!"));
    delay(1500);
    modeConfigured = false;  // Se vuelve a este menu con Calibraciónes cargadas.
  }
  if (x == 4){
    Save_Calibration(ADD_SNS_VOLT_FAC_CAL, Sns_Volt_Calib_Fact);    // Guarda Factor de Calibración de la EEPROM para sensado de Voltaje
    Save_Calibration(ADD_SNS_CURR_FAC_CAL, Sns_Curr_Calib_Fact);    // Guarda Factor de Calibración de la EEPROM para sensado de Corriente
    Save_Calibration(ADD_OUT_CURR_FAC_CAL, Out_Curr_Calib_Fact);    // Guarda Factor de Calibración de la EEPROM para seteo de Corriente
    Save_Calibration(ADD_SNS_VOLT_OFF_CAL, Sns_Volt_Calib_Offs);    // Guarda Offset de Calibración de la EEPROM para sensado de Voltaje
    Save_Calibration(ADD_SNS_CURR_OFF_CAL, Sns_Curr_Calib_Offs);    // Guarda Offset de Calibración de la EEPROM para sensado de Corriente
    Save_Calibration(ADD_OUT_CURR_OFF_CAL, Out_Curr_Calib_Offs);    // Guarda Offset de Calibración de la EEPROM para seteo de Corriente
    printLCD(12, 3, F("Saved!"));
    delay(1500);
    modeConfigured = false;  // Se vuelve a este menu con Calibraciónes cargadas.
  }
  firstPointTaken = false;  // Reinicia pto. 1
  modeInitialized = false;  // Pinta la plantilla TC en el LCD
}

//--------------------------------- Calibrate ---------------------------------------
void Calibrate(float realValue){

  static float measuredValue1 = 0, realValue1 = 0;
  static float measuredValue2 = 0, realValue2 = 0;
  static float setCurrent1 = 0;
  static float setCurrent2 = 0;

  float measuredValue = calibrateVoltage ? voltage : current;

  if (!firstPointTaken) {
    // Guardamos el primer punto
    measuredValue1 = measuredValue;
    realValue1 = realValue;
    setCurrent1 = setCurrent / 1000; //En Amperes
    firstPointTaken = true;     // Flag de P1
    modeInitialized = false;    // Reinicia pidiendo el P2
    return;
  } else {
    // Guardamos el segundo punto y calculamos
    measuredValue2 = measuredValue;
    realValue2 = realValue;
    setCurrent2 = setCurrent / 1000; //En Amperes
    Load_OFF();
    firstPointTaken = false;
    
    float factor = max(0.9, min(1.1, (realValue2 - realValue1) / (measuredValue2 - measuredValue1)));
    float offset = max(-0.1, min(0.1, realValue1 - (measuredValue1 * factor)));

    if (calibrateVoltage) {
        Sns_Volt_Calib_Fact = factor;
        Sns_Volt_Calib_Offs = offset;
    } else {
        Sns_Curr_Calib_Fact = factor;
        Sns_Curr_Calib_Offs = offset;
        Out_Curr_Calib_Fact = max(0.9, min(1.1, (realValue2 - realValue1) / (setCurrent2 - setCurrent1)));
        Out_Curr_Calib_Offs = max(-0.1, min(0.1, realValue1 - (setCurrent1 * Out_Curr_Calib_Fact)));
    }
  lcd.clear();
  printLCD(4, 1, F("Calibrated!"));
  modeConfigured = false;   // Vuelve al Menu de Calibración. 
  firstPointTaken = false;
  delay(2000);
  }
}

//-------------------------- Funciones para el Timer (RTC) ---------------------------

void timer_start() {
  if (!timerStarted) {
    startTime = rtc.now();        // Toma referencia de tiempo
    timerStarted = true;          // flag de que inició el cronometro
  }
}

void timer_stop() {
  if (timerStarted) {
    DateTime now = rtc.now();
    TimeSpan elapsedTime = now - startTime;
    elapsedSeconds += elapsedTime.totalseconds();
    timerStarted = false;
  }
}

void timer_reset() {
  elapsedSeconds = 0.0;
  timerStarted = false;
}

float timer_getTotalSeconds() {
  if (timerStarted) {
    DateTime now = rtc.now();
    TimeSpan elapsedTime = now - startTime;
    return elapsedSeconds + elapsedTime.totalseconds();
  }
  else { return elapsedSeconds; }
}

String timer_getTime() {
  int totalSeconds = static_cast<int>(timer_getTotalSeconds());

  int minutes = (totalSeconds / 60);
  int seconds = (totalSeconds % 60);

  String formattedTime = "";

  if (minutes < 10) { formattedTime += "0"; }
  formattedTime += String(minutes) + ":";

  if (seconds < 10) { formattedTime += "0"; }
  formattedTime += String(seconds);

  return formattedTime;
}

//--------------------------- Funciones para el LCD -----------------------------------
// Imprimir un mensaje de texto variable
void printLCD_S(int col, int row, const String &message) {
    lcd.setCursor(col, row);
    lcd.print(message);
  }

// Imprimir un mensaje con texto almacenado en FLASH
void printLCD(int col, int row, const __FlashStringHelper *message) {
  lcd.setCursor(col, row);
    lcd.print(message);
}

// Imprimir un mensaje con texto almacenado en FLASH
void printLCDNumber(int col, int row, float number, char unit, int decimals) {
  lcd.setCursor(col, row);
  lcd.print(number, decimals);  // Imprime el número con los decimales especificados
  
  if (unit != '\0' && unit != ' ' && unit != 'A') {  // Solo imprime la unidad si no es nula o espacio en blanco o si no es A
    lcd.print(unit);
  } else if (unit == 'A') {lcd.write(byte(0));}   // Escribe el carácter personalizado
}

// Imprimir n cantidad de espacios " "
void Print_Spaces(int col, int row, byte count) {
  lcd.setCursor(col ,row);
  for (byte i = 0; i < count; i++) {
    lcd.print(F(" "));
  }
}

//------------------------------- Wait Key Pressed ------------------------------------
char Wait_Key_Pressed() {
  char key;
  do {
      key = customKeypad.getKey();
  } while (key == NO_KEY);  
  return key;
}

//------------------------------- Reset Input Pointers --------------------------------
void Reset_Input_Pointers (void){
  index = 0;
  numbers[index] = '\0';
  decimalPoint = ' ';
}

//------------------------------- Handle Mode Keys -----------------------------------
bool Handle_MSC_Keys(char key) {
  static bool shiftPressed = false;  // Bandera para detectar Shift

  if (customKey == 'M') {   // Salir del modo si se presiona 'M'
    lcd.noCursor(); lcd.blink_off();
    Mode_Selection(false);  
    return false;
  } 

  else if (shiftPressed) {
    shiftPressed = false;
    Mode_Selection(true, customKey); // Llama con Shift activo y la tecla presionada
    lcd.noCursor(); lcd.blink_off();
    return false;     // ##Ojo## si Mode_Selection con con key C, ".", E, 7, 8, 9, 0, que no hacen nada, va a salir del modo y se va a reiniciar.
  }

  else if (customKey == 'S') { // Detectar Shift
    shiftPressed = true;
    return true; // Espera la próxima tecla
  }

  else if (customKey == 'C') {        // ojo que se puede llamar desde el mismo Config_Limits
    if (Mode != TC && Mode != TL) {
      Config_Limits();
    }
    return true; 
  }

  return true;
}

//------------------------------- Handle Buzzer -----------------------------------
void beepBuzzer(void) {
  for (int i = 0; i < 2; i++) {   // Repetir dos veces
      digitalWrite(BUZZER, HIGH); // Encender el buzzer  
      delay(200);                 // Pausa entre los pitidos
      digitalWrite(BUZZER, LOW);  // Encender el buzzer  
      delay(200);                 // Pausa entre los pitidos
  }
}

//--------------------------------- Lee de EEPROM -------------------------------------
float loadFromEEPROM(int address)
{
  float value;
  EEPROM.get(address, value);
  return value;
}

//--------------------------------------- Carga de la EEPRON factores de Calibration ------------------------------------------
void Load_Calibration(int address, float &variable) {

  EEPROM.get(address, variable);                    // Lee EEPROM

  if (address != ADD_SNS_VOLT_OFF_CAL && address != ADD_SNS_CURR_OFF_CAL && address != ADD_OUT_CURR_OFF_CAL) {  //si no es address de offset
    if (variable < 0.9 || variable > 1.1) {   // Validar que el valor esté dentro del rango válido (0.9 a 1.1)
      variable = 1.0;                         // Restaurar el valor por defecto
    }
  }
  else {
    if (variable < -0.1 || variable > 0.1) {  // Es de offset, valida que el valor esté dentro del rango válido (-0.1 a 0.1)
      variable = 0.0;                         // Restaurar el valor por defecto
    }
  }
}

//----------------------------------------- Validar Calibración y grabar en EEPROM --------------------------------------------
void Save_Calibration(int address, float value) {
  float eeprom_read_Cal;

  eeprom_read_Cal = loadFromEEPROM(address);    // Leer el valor de calibración guardado en EEPROM
  if (abs(value - eeprom_read_Cal) > 0.0001) {  // Verifica con tolerancia
    EEPROM.put(address, value);                 // Guardar nuevo valor
  }
}