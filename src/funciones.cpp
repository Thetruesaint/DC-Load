#include "variables.h"
#include "funciones.h"
#include <EEPROM.h>

//----------------------------- Load ON Status ------------------------------------
void Load_ON_status(bool loadonoff)
{
  toggle = loadonoff;
  #ifndef WOKWI_SIMULATION
  dac.setVoltage(loadonoff ? controlVoltage : 0, false); // Set DAC voltage based on load status
  #endif
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
  encoderPosition = constrain(encoderPosition, 0, encoderMax);
  if (Mode == CC || Mode == BC) {
    encoderPosition = constrain(encoderPosition, 0, 10000); 
  }
}

//---------------------------- Read Keypad Input ----------------------------------
void Read_Keypad(void) {

  static int zl = 1;

  customKey = customKeypad.getKey();              // Escanea el teclado   
  
  if (customKey == NO_KEY) return;                // Si no hay tecla presionada, sale de la función

  switch (customKey) {                            // Si hay una tecla presionada, la procesa
    case 'M':                                     // Cambio de Modo                    
      Load_ON_status(false);                      // Apaga la carga
      zl = 1;                                     // Resetea la posición en el renglón
      Reset_Input_Pointers();                     // Resetea el punto decimal y el indice
      Mode_Selection();
        break;                                      
      case 'C':                                   // Configuración de limites
        if (Mode != TC || Mode !=TL){             // Por ahora solo durante modos CC, CP y CR.
          Config_Limits(); }
        break;                                      
      case 'S':  // Uso futuro para Shift paa ir a un modo directo o a Calibración S+C?
        break;
  }

  // Solo permite entrada de valores en los modos CC, CP y CR
  if (Mode == BC || Mode == TC || Mode == TL) return;

  if (customKey >= '0' && customKey <= '9' && index < 5) {  // Si la tecla presionada es un número, se permiten hasta 5 caracteres
    numbers[index++] = customKey;                           // Almacena el número en la variable numbers
    numbers[index] = '\0';                                  // Agrega el caracter nulo al final de la cadena
    printLCD_S(zl++, 3, String(customKey));                 // Muestra el número en el LCD
  }

  if (customKey == '.' && decimalPoint != '*') {   // Si la tecla presionada es un punto decimal y no se ha ingresado uno antes
    numbers[index++] = '.';                        // Almacena el punto decimal en la variable numbers
    numbers[index] = '\0';                         // Agrega el caracter nulo al final de la cadena
    printLCD(zl++, 3, F("."));                     // Muestra el punto decimal en el LCD
    decimalPoint = '*';                            // Marca que se ingresó un punto decimal
  }

  if (customKey == 'E') {               // Confirmar entrada
    reading = atof(numbers);            // Convierte cadena de caracteres en número y lo asigna a reading 
    encoderPosition = reading * 1000;   // Asigna el valor a la variable encoderPosition
    numbers[index] = '\0';              // Resetea la cadena de caracteres
    zl = 1;                             // Resetea la posición en el renglón
    printLCD(1, 3, F("     "));         // Borra el renglón del LCD
    Reset_Input_Pointers();             // Resetea el punto decimal y el indice
  }

  // **Manejo de borrado**
  if (customKey == '<' && index > 0) {  
    index--;  
    if (numbers[index] == '.') decimalPoint = ' '; // Si borramos un punto, permitimos otro  
    numbers[index] = '\0';  
    zl--;  
    printLCD(zl, 3, F(" ")); // Borra visualmente en LCD  
  }
}

//----------------------- Toggle Current Load ON or OFF ----------------------------
void Read_Load_Button(void) {
  if (digitalRead(LOADONOFF) == LOW) {
    delay(200); // Anti-rebote
    toggle = !toggle;
    if (!toggle) {setCurrent = 0;} // Si la carga se apaga, resetear el valor de corriente
  }
}

//------------ Calculate and Display Actual Voltage, Current, and Power ------------
void Update_LCD(void) {
  static unsigned long lastUpdateTime = 0;

  if(!modeInitialized) return;  // No actualiza el LCD hasta que el modo dibuje la plantilla y ponga modeInitialized = true
  lcd.noCursor();

  // Esperar 100ms antes de actualizar el resto del codigo en el LCD
  if (millis() - lastUpdateTime < LCD_RFSH_TIME) return;

  lastUpdateTime = millis();  // Actualizar el tiempo de referencia
  
  //printLCD_S(18, 3, ModeNames[Mode]); // Actualiza el Modo, lo hace seguido porque a veces se sale y se entra al mismo modo

  // Evitar valores negativos por errores de medición
  float power = voltage * current;
  if (power < 0) power = 0;
  if (voltage < 0.0) voltage = 0.0;
  if (current < 0.0) current = 0.0;

  printLCD(8, 0, toggle ? F("ON ") : F("OFF"));  // Indica el estado de la carga

  // Imprimir los valores actualizados, ojo con W que si se corre puede afectar a col 0, row 3
  printLCDNumber(0, 1, current, 'A', (current < 10.0) ? 3 : 2);
  printLCDNumber(7, 1, voltage, 'V', (voltage < 10.0) ? 3 : (voltage < 100.0) ? 2 : 1);
  if (Mode != BC) {   // lo reemplazo por BatteryCutoffVolts
    lcd.setCursor(14,1);
    if (power < 10) { lcd.print(F(" ")); lcd.print(power, 2);} 
    else if (power < 100) {lcd.print(power, 2);} 
    else {lcd.print(power, 1);}
    lcd.setCursor(19,1);
    lcd.print(F("W"));
  }

  if (Mode != TC && Mode != TL) {  // Evitar mostrar el encoder en modos transitorios
    lcd.setCursor(8, 2);
    if ((Mode == CP || Mode == CR) && reading < 100) lcd.print(" ");
    if (reading < 10) lcd.print(" ");
    lcd.print((Mode == CP || Mode == CR) ? String(reading, 2) : String(reading, 3));
    
    lcd.setCursor(CuPo, 2); // Cursor en la unidad a modificar
    lcd.cursor();
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
  lcd.noCursor(); lcd.setCursor( 16, 0);
  if (temp < 10){lcd.print(" ");}
  lcd.print(temp);
  lcd.print(char(0xDF)); lcd.print("C");
}

//--------------------------- Check and Enforce Limits ------------------------------
void Check_Limits() {
  char message[20] = "";
  float power = voltage * current;
  float maxpwrdis = constrain(140 - 0.80 * temp, 0, 120);
  float actpwrdis = max(0, power / 4);

  if (voltage > MAX_VOLTAGE) strcpy(message, "Max Voltage!      ");
  else if (current > CurrentCutOff * 1.05) strcpy(message, "Current Cut Off!  ");
  else if (power > PowerCutOff) strcpy(message, "Power Cut Off!    ");
  else if (temp >= tempCutOff) strcpy(message, "Over Temperature! ");
  else if (actpwrdis >= maxpwrdis) strcpy(message, "Max PWR Disipation");
  else if ((Mode == CC && reading > CurrentCutOff) ||
          (Mode == CP && reading > PowerCutOff) ||
          (Mode == CR && reading > ResistorCutOff) ||
          (Mode == BC && reading > CurrentCutOff)) {
            reading = (Mode == CC) ? CurrentCutOff :
                      (Mode == CP) ? PowerCutOff :
                      (Mode == CR) ? ResistorCutOff : CurrentCutOff;
            encoderPosition = reading * 1000;
            return;
  }
  
  if (strlen(message) > 0){
    Load_ON_status(false);                    // Si hubo mensaje, apagar la carga ASAP.
    lcd.noCursor();
    for (int i = 0; i < 3; i++) {             // Parpaderá el mensaje tres veces
      printLCD_S(0, 3, message);
      delay(500);
      printLCD(0, 3, F("                  "));  // Borra todo salvo el indicador de modo
      delay(500);
    }
    reading = 0;
    encoderPosition = 0;
    setCurrent = 0;
    Reset_Input_Pointers();           
    modeInitialized = false;      // Avisa a los modos que se deben inicializar dado que se execdio un limite
  }
}

//------------------------------- Cursor Position -----------------------------------
void Cursor_Position(void) {
  
  int unitPosition = (Mode == CP || Mode == CR) ? 10 : 9; // Definir la posición de la unidad en base al modo
  if (digitalRead(ENC_BTN) == LOW) { delay(200); CuPo++;} // Corre el cursor un lugar a la derecha
  if (CuPo == unitPosition + 1) { CuPo++;}          // Salta el punto decimal
  if (CuPo > 13) {CuPo = unitPosition;}             // vuelve a las unidades
  if (CuPo == unitPosition)       factor = 1000;    // Unidades
  else if (CuPo == unitPosition + 2) factor = 100;  // Decenas
  else if (CuPo == unitPosition + 3) factor = 10;   // Centenas
  else if (CuPo == unitPosition + 4) factor = 1;    // milesimas
  lcd.setCursor(CuPo, 2);
  lcd.cursor();
}

//--------------------------- Read Voltage and Current ------------------------------
void Read_Volts_Current(void) {

  // static float multiplier = 0.1875F; /* ADS1115  @ +/- 6.144V gain (16-bit results) */
  // ads.setGain(GAIN_TWOTHIRDS);  // 2/3x gain +/- 6.144V  1 bit = 0.1875mV
  // ads.setGain(GAIN_ONE);        // 1x gain   +/- 4.096V  1 bit = 0.125mV
  // ads.setGain(GAIN_FOUR);       // 4x gain   +/- 1.024V  1 bit = 0.03125mV
  // ads.setGain(GAIN_EIGHT);      // 8x gain   +/- 0.512V  1 bit = 0.015625mV
  // ads.setGain(GAIN_SIXTEEN);    // 16x gain  +/- 0.256V  1 bit = 0.0078125mV
  #ifndef WOKWI_SIMULATION
  struct GainSetting
  { // Estructura para almacenar las configuraciones de ganancia
    float minVoltage;
    float maxVoltage;
    adsGain_t gain;
    float calibrationFactor;
  };

  const GainSetting voltageGains[] = {
      // Configuraciones de ganancia para la lectura de voltaje
      {0, 12, GAIN_SIXTEEN, 50.8346}, // Si es entre 0 y 12V, pongo la ganancia en x16 y vuelvo a leer para mejorar la presición
      {12, 25, GAIN_EIGHT, 49.6135},  // Si es entre 12 y 25V, pongo la ganancia en x8 y vuelvo a leer para mejorar la presición
      {25, 50, GAIN_FOUR, 49.6857},   // Si es entre 25 y 50V, pongo la ganancia en x4 y vuelvo a leer para mejorar la presición
      {50, 200, GAIN_ONE, 50.5589}    // Si es entre 50 y 200V, pongo la ganancia en x1
  };

  const GainSetting currentGains[] = {
      // Configuraciones de ganancia para la lectura de corriente
      {0.0, 1.9, GAIN_SIXTEEN, 10.000}, // Si es entre 0 y 1.9A, pongo la ganancia en x16 y vuelvo a leer para mejorar la presición
      {1.9, 4.9, GAIN_EIGHT, 9.9941},   // Si es entre 1.9 y 4.9A, pongo la ganancia en x8 y vuelvo a leer para mejorar la presición
      {4.9, 9.8, GAIN_FOUR, 9.7621},    // Si es entre 4.9 y 9.8A, pongo la ganancia en x4 y vuelvo a leer para mejorar la presición
      {9.8, 15.0, GAIN_ONE, 9.6774}     // Si es entre 9.8 y 15A, pongo la ganancia en x1
  };

  // Lectura de voltaje
  ads.setGain(GAIN_ONE);                                                // Por 50 por el divisor resistivo y ampl. dif. para sensado remoto de 50 a 1 (Max. 200V). Calibración promedio
  adc3 = ads.readADC_SingleEnded(VLTG_SNSR);                              // Lee el ADC
  voltage = ads.computeVolts(adc3) * voltageGains[3].calibrationFactor; // Calcula el voltaje

  for (const auto &setting : voltageGains)
  { // Itera sobre las configuraciones de ganancia
    if (voltage >= setting.minVoltage && voltage < setting.maxVoltage)
    {                                                               // Encuentra la configuración de ganancia correcta
      ads.setGain(setting.gain);                                    // Configura la ganancia correcta
      adc3 = ads.readADC_SingleEnded(VLTG_SNSR);                      // Lee el ADC
      voltage = ads.computeVolts(adc3) * setting.calibrationFactor; // Calcula el voltaje
      break;                                                        // Sale del bucle
    }
  }

  // Lectura de corriente
  ads.setGain(GAIN_ONE);                                                // Puede ser mas de 10A, pongo la ganancia en x1 por protección
  adc1 = ads.readADC_SingleEnded(CRR_SNSR);                              // Lee el ADC
  current = ads.computeVolts(adc1) * currentGains[3].calibrationFactor; // Calcula la corriente

  for (const auto &setting : currentGains)
  { // Itera sobre las configuraciones de ganancia
    if (current >= setting.minVoltage && current < setting.maxVoltage)
    {                                                               // Encuentra la configuración de ganancia correcta
      ads.setGain(setting.gain);                                    // Configura la ganancia correcta
      adc1 = ads.readADC_SingleEnded(CRR_SNSR);                      // Lee el ADC
      current = ads.computeVolts(adc1) * setting.calibrationFactor; // Calcula la corriente
      break;                                                        // Sale del bucle
    }
  }

  ads.setGain(GAIN_TWOTHIRDS); // Restaurar configuración predeterminada
  #else

  // Simulación de Voltaje sensado

  int potValue = analogRead(A2);  // Leer el potenciómetro en A2
  static float simulatedVoltage = 0;  // Variable persistente para almacenar el voltage simulado
  static unsigned long lastDecreaseTime = 0;  // Variable para medir el tiempo del último decremento
  unsigned long currentMillis = millis();
  
  if (Mode != BC){                                             // Para todos los demas modos
    simulatedVoltage = map(potValue, 0, 1023, 300, 0) / 10.0;  // Convertir el rango 0-1023 a 30V-0V
    voltage = simulatedVoltage;                                // Asigna el valor de v simulado
  } else { 
      // En modo BC Si toggle es true, reduce el voltaje simulado en 0.01V. Luego ver si puede ser proporcional a la corriente
      // ## SETEAR ANTES DE ENTRAR AL MODO##
      if (toggle && (currentMillis - lastDecreaseTime >= 2000)) {
      lastDecreaseTime = currentMillis;               // Actualizar el último tiempo de decremento
      simulatedVoltage -= 0.005;                      // Reducir el voltaje a medida que pasa el tiempo
      simulatedVoltage = max(simulatedVoltage, 0.0);  // Asegurar que no sea negativo
      voltage = simulatedVoltage;                     // Asigna el valor de v simulado
      }
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
  if (toggle) {
    #ifndef WOKWI_SIMULATION
    controlVoltage = setCurrent * Set_Curr_Dsgn_Fact; 
    dac.setVoltage(controlVoltage, false); // set DAC output voltage for Range selected
    #endif
  } else {
    #ifndef WOKWI_SIMULATION
    dac.setVoltage(0, false); // set DAC output voltage to 0 if Load Off selected
    #endif
  }
}

//----------------------- Select Constant Current LCD set up ------------------------
void Const_Current_Mode(void) {

  if (!modeInitialized) {
    lcd.clear();
    printLCD(0, 0, F("CC LOAD"));          // Muestra el titulo del modo
    printLCD(0, 2, F("Set I> "));         // Muestra el mensaje
    printLCD(14, 2, F("A"));               // Muestra el mensaje
    printLCD(0, 3, F(">"));                // Indica la posibilidad de ingresar valores.
    CuPo = 9;                              // Pone el cursor en la posición de las unidades de Amperes
    reading = 0; encoderPosition = 0;      // Resetea la posición del encoder y cualquier valor de reading
    modeInitialized = true;                // Modo inicializado
  }
  reading = encoderPosition / 1000;
  if (!toggle) return;
  setCurrent = reading * 1000;             // lo pasa a mA 
  controlVoltage = setCurrent * Set_Curr_Dsgn_Fact;
  
}

//------------------------ Select Constant Power LCD set up -------------------------
void Const_Power_Mode(void) {

  if (!modeInitialized) {
    lcd.clear();
    printLCD(0, 0, F("CP LOAD"));          // Muestra el titulo del modo
    printLCD(0, 2, F("Set W> "));         // Muestra el mensaje
    printLCD(14, 2, F("W"));               // Muestra el mensaje
    printLCD(0, 3, F(">"));                // Indica la posibilidad de ingresar valores.
    CuPo = 10;                             // Pone el cursor en la posición de las unidades de Potecia
    reading = 0; encoderPosition = 0;      // Resetea la posición del encoder y cualquier valor de reading
    modeInitialized = true;                // Modo inicializado
  }
  reading = encoderPosition / 1000;
  if (!toggle) return;
  setPower = reading * 1000;               // Conversión a mW
  setCurrent = setPower / voltage;         // Conversión a mA 
}

//---------------------- Select Constant Resistance LCD set up ----------------------
void Const_Resistance_Mode(void) {

 if (!modeInitialized) {
    lcd.clear();
    printLCD(0, 0, F("CR LOAD"));           // Muestra el titulo del modo
    printLCD(0, 2, F("Set R> "));          // Muestra el mensaje
    printLCD_S(14, 2, String((char)0xF4));  // Muestra el Símbolo de Ohms
    printLCD(0, 3, F(">"));                 // Indica la posibilidad de ingresar valores.
    CuPo = 10;                              // Pone el cursor en la posición de las unidades de Resistencia
    reading = 1.0;                          // Valor por default, 1o hm.
    encoderPosition = reading * 1000;       // Resetea la posición del encoder y cualquier valor de reading
    modeInitialized = true;                 // Modo inicializado
  }
  // Solo actualiza `reading` si el encoder cambió
  float newReading = encoderPosition / 1000.0;  // Convierte a ohms

  if (newReading != reading) {  
    reading = max(0.1, newReading);         // Evita resistencia 0, mínimo 0.1Ω
    encoderPosition = reading * 1000;       // Solo actualiza el encoder si el valor cambió
  }

  if (!toggle) return;
  setResistance =  reading;                 // en que unidad de ohms sería?     
  setCurrent = (voltage / setResistance) * 1000;  // convirte a mA
}

//-------------------- Select Battery Capacity Testing LCD set up -------------------
void Battery_Mode(void) {

  if (!modeConfigured) {Battery_Type_Selec(); return;}  // Probar bien si aca no tiene problemas, en algunas pruebas queda fliqueando con carga en ON

  if (!modeInitialized){
    lcd.clear();
    printLCD(0, 0, F("BC LOAD"));          // Muestra el titulo del modo
    lcd.setCursor(13,1);lcd.print(F(">")); 
    printLCDNumber(14, 1, BatteryCutoffVolts, 'V', 2); // Muestro el Cutoff Voltage
    printLCD(0, 2, F("Adj I> "));         // Muestra el mensaje
    printLCD(14, 2, F("A"));               // La unidad de corriente
    printLCDNumber(6, 3, BatteryLife, ' ', 0); // Mostrar sin decimales
    lcd.print(F("mAh"));
    printLCD_S(14, 3, BatteryType);        // Muestro el tipo de Bateria.
    CuPo = 9;                              // Pone el cursor en la posición de las unidades de Amperes
    reading = 0; encoderPosition = 0;      // Resetea la posición del encoder y cualquier valor de reading
    modeInitialized = true;                // Modo inicializado
  }
  lcd.noCursor();
  if (BatteryLife > BatteryLifePrevious) { // Actualizar LCD solo si cambia el valor
    printLCDNumber(9, 3, BatteryLife, ' ', 0);
    lcd.print(F("mAh"));
    BatteryLifePrevious = BatteryLife;
  }
  if (toggle && (voltage > BatteryCutoffVolts)){printLCD(16,2, F("    "));} // Borra "Done " si quedo de antes.
  else if (!toggle && (voltage <= BatteryCutoffVolts)) {printLCD(16,2, F("Done"));}
}

//-------------------- Battery Type Selection and Cutoff Setup ----------------------
void Battery_Type_Selec() {
  exitMode = false;                       // Resetea EXIT mode
  lcd.noCursor();                         // Apaga el cursor para este menú
  lcd.clear();                            // Borra la pantalla del LCD
  printLCD(2, 0, F("Set Task & Batt"));   // Muestra el título
  printLCD(0, 1, F("Stor. 1)LiPo 2)LiIOn"));
  printLCD(0, 2, F("Disc. 3)LiPo 4)LiIOn"));
  printLCD(2, 3, F("5)Cutoff Voltage"));

  while (true) {  // Bucle para evitar la salida accidental
    customKey = Wait_Key_Pressed(); 

    switch (customKey) {
        case '1': BatteryCutoffVolts = LiPoStoragVoltage; BatteryType = "Li-Po"; break;
        case '2': BatteryCutoffVolts = LionStoragVoltage; BatteryType = "Li-Ion"; break;
        case '3': BatteryCutoffVolts = LiPoCutOffVoltage; BatteryType = "Li-Po"; break;
        case '4': BatteryCutoffVolts = LionCutOffVoltage; BatteryType = "Li-Ion"; break;
        case '5': BatteryType = "Custom"; break;
        //case 'C': Config_Limits(); Battery_Type_Selec(); break; Fo funciona bien, se deshabilita por ahora
        case 'M': case '<': Mode_Selection(); return; // Salida del Modo y salta al proximo, ver como queda el functionIndex
        default: continue;  // Evita salir si la tecla es inválida
    }
    break;  //  Sale del bucle si se ingresó una tecla válida
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
      printLCD(z, r, F("     "));         // Borra el espacio si hubo un valor fuera de rango
      if (!Value_Input(z, r, 5)) return;  // Sale automáticamente si el usuario presiona 'M'
    } while (x > 25 || x < 0.1);          // Solo chequea si `x` está en rango
      BatteryCutoffVolts = x;
  } 

  // Pedir ingresar la cantidad de celdas
  if (BatteryType != "Custom") {
    lcd.clear();
    printLCD_S(3, 0, BatteryType + " Batt");
    printLCD(6, 1, F("(1-6)S?"));

    do {
        z = 9; r = 2; //y = 1;
        printLCD(z - 1, r, F(">"));
        printLCD(z, r, F(" "));             // Borra el espacio si hubo un valor fuera de rango
        if (!Value_Input(z, r, 1)) return;  // Sale automáticamente si el usuario presiona 'M'
    } while (x < 1 || x > 6);               // Asegura que solo se ingrese un número entre 1 y 6
    BatteryCutoffVolts *= x;                // Multiplica por la cantidad de celdas
  }

  timer_reset();                  // Resetea el timer
  BatteryLifePrevious = 0;        // Resetea la vida de la batería
  CuPo = 9;                       // Posiciona el cursor en la posición 9
  modeConfigured = true;          // Indica que se seteo el modo.
  modeInitialized = false;        // Pinta la plantilla BC en el LCD
}

//---------------------- Battery Capacity Discharge Routine -------------------------
void Battery_Capacity(void) {
  
  float LoadCurrent = 0;
  unsigned long currentMillis = millis();
  static unsigned long lastUpdate = 0;    // Guarda el tiempo de la última actualización del display

  if (toggle && voltage >= BatteryCutoffVolts && !timerStarted) { timer_start(); } // Inicia el timer si la carga está activa
  if (!toggle && voltage >= BatteryCutoffVolts && timerStarted) { timer_stop(); }  // Detiene el timer si la carga está inactiva
 
  if (currentMillis - lastUpdate >= 500) { // Actualizar cada 1 segundo para evitar flickering
    lastUpdate = currentMillis;

    lcd.noCursor();
    printLCD_S(0, 3, timer_getTime()); // Mostrar tiempo en LCD

    Seconds = timer_getTotalSeconds();
    LoadCurrent = (!timerStarted) ? 0 : current;
    BatteryLife += (LoadCurrent * 1000) / 7200; // LoadCurrent A, por horas, por 1000 = mAh
  }

  reading = encoderPosition / 1000;   // este modo se controla solo con el encoder.

  if (!toggle) return;  //Si esta desconectada, salir, el resto solo si esta conectada

  setCurrent = reading * 1000; // Toma el valor del encoder, por si quise hacer un ajuste, y lo setea en la carga en mA

  // Reducción progresiva de corriente cuando el voltaje alcanza al de corte

  if (voltage <= BatteryCutoffVolts) {  // Fase 2: Estabilización
    setCurrent = max(setCurrent - CRR_STEP_RDCTN, MinDischargeCurrent);
    reading = setCurrent / 1000;
    encoderPosition = reading * 1000;
  }

  // Si el voltaje ya cayo por debajo de VoltageDropMargin, corta la carga (esto es porque la lipo se recopera sin carga)
  if (voltage <= (BatteryCutoffVolts - VoltageDropMargin)) { 
    BatteryCurrent = current;   // Toma nota de la corriente mínima con la que quedo?
    Load_ON_status(false);
    timer_stop();
  }
}

//---------------------------- Transcient Continuos Mode ----------------------------
void Transient_Cont_Mode(void) {

  if(!modeConfigured) {Transient_Cont_Setup(); return;}   // Si no esta configurado, lo configura. Sale si no se configuro

  if (!modeInitialized) {                       // Si es falso, prepara el LCD
    lcd.noCursor();                             // switch Cursor OFF for this menu
    printLCD_S(3, 2, String(LowCurrent, 3));    // Muestra el valor de la corriente baja
    printLCD_S(14, 2, String(HighCurrent, 3));  // Muestra el valor de la corriente alta
    printLCD(0, 0, F("TC LOAD"));               // Muestra el titulo del modo
    printLCD(0, 2, F("Lo="));                   // Muestra el mensaje
    printLCD(8, 2, F("A"));                     // Muestra la unidad
    printLCD(11, 2, F("Hi="));                  // Muestra el mensaje
    printLCD(19, 2, F("A"));                    // Muestra la unidad
    printLCD(0, 3, F("Time: "));                // Muestra el mensaje
    printLCD_S(6, 3, String(transientPeriod));  // Muestra el valor del tiempo
    printLCD(11, 3, F("mSecs"));                // Muestra la unidad
    modeInitialized = true;                     // Modo inicializado.
  }
}

//--------------------------------- Transient Mode ----------------------------------
void Transient_Cont_Setup(void) {
  exitMode = false;
  lcd.clear(); // Apaga el cursor y borra la pantalla del LCD

  printLCD(3, 0, F("TRANSIENT CONT."));
  printLCD(0, 1, F("Low I (A):"));
  z = 13; r = 1;                      // Setea las posiciones de la pantalla del LCD
  if (!Value_Input(z, r, 5)) return;                // Obtiene el valor ingresado por el usuario o sale del modo
  LowCurrent = min(x, CurrentCutOff);         // Limita la corriente baja al valor de corte de corriente
  printLCDNumber(z, r, LowCurrent, 'A', 3);  // Muestra el valor de la corriente baja

  printLCD(0, 2, F("High I (A):"));
  z = 13; r = 2; 
  if (!Value_Input(z, r, 5)) return;                // Limita la corriente baja al valor de corte de corriente
  HighCurrent = min(x, CurrentCutOff);        // Limita la corriente alta al valor de corte de corriente
  printLCDNumber(z, r, HighCurrent, 'A', 3); // Muestra el valor de la corriente alta con tres decimales

  printLCD(0, 3, F("Delay(mSec):"));
  z = 13; r = 3;                              // Setea las posiciones de la pantalla del LCD
  if (!Value_Input(z, r, 5)) return;                // Limita la corriente baja al valor de corte de corriente
  transientPeriod = x;                        // Guarda el valor del tiempo de transitorio
  lcd.clear();                                // Borra la pantalla del LCD
  Load_ON_status(false);                      // Apaga la carga
  setCurrent = 0;                             // por si quedo seteada del modo anterior
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
    last_time = current_time; // 🔹 Actualiza tiempo directamente

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

  if(modeConfigured) {
    printLCD_S(13, 2, String(current_instruction + 1));     // Muestra la instrucción en curso
    if ((current_instruction + 1) < 10){lcd.print(F(" "));} // si quedo el cero del 10, lo borra
    // ✅ Actualiza `transientPeriod` solo si cambió, evitando flickering innecesario
    if (transientPeriod != last_transientPeriod) {
      printLCD(6, 3, F("     "));  // Borra los caracteres anteriores antes de imprimir
      printLCD_S(6, 3, String(transientPeriod)); // Imprime nuevo valor con espacio extra
      last_transientPeriod = transientPeriod;  // Actualiza el último valor mostrado
    }
  }

  if(!modeConfigured) {Transient_List_Setup(); return;} // Si no esta configurado, lo configura. Sale si no se configuro

  if (!modeInitialized) {                 // Si es falso, prepara el LCD
    lcd.noCursor();
    printLCD(0, 0, F("TL LOAD"));         // Muestra el titulo del modo
    printLCD(0, 2, F("Instruccion: "));   // Muestra el mensaje
    printLCD(15, 2, F("/"));   // Muestra el mensaje
    printLCD_S(16, 2, String(total_instructions + 1));
    printLCD(0, 3, F("Time: "));         // Muestra el mensaje
    printLCD(11, 3, F("mSecs"));          // Muestra la unidad
    modeInitialized = true;               // Modo inicializado.
  }
}

//------------------------------ Transcient List Setup -------------------------------
void Transient_List_Setup() {
  exitMode = false;
  lcd.noCursor();
  lcd.clear(); // Apaga el cursor y borra la pantalla
  // Pregunta por cuantos saltos se desean cargar
  printLCD(3, 0, F("TRANSIENT LIST"));
  printLCD(2, 1, F("Set Q (2 to 10)?"));
  do { // Bucle para garantizar entrada válida
    z = 9; r = 2; //y = 9;
    printLCD(z - 1, r, F(">"));
    printLCD(z, r, F("  "));   // Borra el espacio si hubo un valor fuera de rango
    if (!Value_Input(z, r, 2)) return; // Si se presiona 'M', sale del modo
  } while (x < 2 || x > 10);

  total_instructions = x - 1;   // Guarda el número total de instrucciones

  // Pide confifurar cada salto
  lcd.clear();                // Borra la pantalla del LCD
  for (int i = 0; i <= total_instructions; i++) {     // Bucle para obtener los valores de la lista
    printLCD(3, 0, F("TRANSIENT LIST"));              // Mantengo el titulo para que se vea bien el modo que se está configurando
    printLCD_S(0, 1, "Instruccion " + String(i + 1)); // Muestra la instrucción a configurar
    printLCD(0, 2, F("Current (A):"));                // Pide el valor de corriente en Amperes
    printLCD(0, 3, F("Time (mSec):"));                // Pide el valor de tiempo en milisegundos

    z = 13; r = 2;
    if (!Value_Input(z, r, 5)) return;  // Permitir 5 digitos, ej.: 1.234 o salir del Modo
    x = min(x, CurrentCutOff);          // Limita a CutOff
    printLCDNumber(z, r, x, 'A', 3);    // Muestra el valor de la corriente
    transientList[i][0] = x * 1000;     // Lo guarda en la lista en mA

    z = 13; r = 3;                      // Ubica la toma del valor de mSec
    if (!Value_Input(z, r, 5)) return;  // Permitir 5 digitos, ej.: 99999 o salir del Modo
    transientList[i][1] = x;            // Guarda el valor del tiempo en ms
    lcd.clear();                        // Borra la pantalla, para configurar la siguiente instrucción
  }
  lcd.noCursor();
  setCurrent = 0;          // por si quedo seteada del modo anterior
  current_instruction = 0; // Resetea el contador de instrucciones porque finalizo la configuración
  transientPeriod = transientList[current_instruction][1];      // Por las dudas tambien el periodo a mostrar
  modeConfigured = true;   // Se configuro el modo TC
  modeInitialized = false; // Pinta la plantilla TC en el LCD
}

//------------------------------ Transcient List Timing ------------------------------
void Transient_List_Timing(void) {

  static unsigned long last_time = 0;

  if (!toggle) {
    current_instruction = 0;
    last_time = 0;
    transientPeriod = transientList[current_instruction][1];
    return;} // Reinicio la lista

    current_time = micros();

  if (last_time == 0){
    //setCurrent = transientList[current_instruction][0] * 1000; // La convierte a mA
    setCurrent = transientList[current_instruction][0];       // Ya esta en mA
    transientPeriod = transientList[current_instruction][1];
    last_time = current_time; 
  }

  if ((current_time - last_time) >= transientPeriod * 1000) {
    current_instruction++;
    if (current_instruction > total_instructions) { current_instruction = 0; }
    //setCurrent = transientList[current_instruction][0] * 1000; // La convierte a mA
    setCurrent = transientList[current_instruction][0];       // Ya esta en mA
    transientPeriod = transientList[current_instruction][1];
    last_time = current_time;
  }
}

//------------------------------ User set up for limits ------------------------------
void Config_Limits(void)
{
  Load_ON_status(false);            // Apaga la carga
  Show_Limits();
  delay(2000);
  lcd.clear();

  printLCD(4, 0, F("Set Limits"));
  printLCD(0, 1, F("Current(A):"));
  z = 12; r = 1;
  if (!Value_Input(z, r, 4)) return;      // Permitir 4 digitos, ej.: 1.23 o 123 salir del Modo
  CurrentCutOff = min(x, 10);
  printLCDNumber(z, r, CurrentCutOff,' ');
  
  printLCD(0, 2, F("Power(W):"));
  r = 2; z = 12;
  if (!Value_Input(z, r, 3)) return;      // Permitir 3 digitos, ej.: 1.2 o 100 salir del Modo
  PowerCutOff = min(x, 300);
  printLCDNumber(r, r, PowerCutOff, ' ',0);
 
  printLCD(0, 3, F("Temp.("));
  printLCD_S(6, 3, String((char)0xDF) + "C):");
  z = 12; r = 3;
  if (!Value_Input(z, r, 2)) return;      // Permitir 2 digitos, ej.: 10 a 99 o salir del Modo
  tempCutOff = min(x, 99);
  printLCD_S(z, r, String(tempCutOff));

  saveToEEPROM(ADD_CURRENT_CUT_OFF, CurrentCutOff);
  saveToEEPROM(ADD_POWER_CUT_OFF, PowerCutOff);
  saveToEEPROM(ADD_TEMP_CUT_OFF, tempCutOff);

  Show_Limits();
  delay(2000);
  modeInitialized = false;   // Pero lo inicializa. Ojo con los modos no preparados para esto como BC, TC y TL
}

//----------------- Show limits Stored Data for Current, Power and Temp --------------
void Show_Limits(void) {
  lcd.noCursor();lcd.clear();
  
  // Los lee de la EEPROM
  CurrentCutOff = loadFromEEPROM(ADD_CURRENT_CUT_OFF);
  PowerCutOff = loadFromEEPROM(ADD_POWER_CUT_OFF);
  tempCutOff = loadFromEEPROM(ADD_TEMP_CUT_OFF);

  // Los muestra
  printLCD(1, 0, F("Limits")); // Muestra el titulo
  printLCD(0, 1, F("Current:"));
  printLCDNumber(9, 1, CurrentCutOff, 'A');

  printLCD(0, 2, F("Power:"));
  printLCDNumber(9, 2, PowerCutOff, 'W', 0);
  printLCD(0, 3, F("Temp.:"));
  printLCDNumber(9, 3, tempCutOff, ' ', 0);
  lcd.print(char(0xDF)); lcd.print("C");
}

//----------------------- Key input used for UserSetUp ------------------------------- 
bool Value_Input(int col, int row, int maxDigits) {  
  // Resetea los valores de entrada
  Reset_Input_Pointers();  

  lcd.setCursor(col, row);  // Ubica el cursor en la posición especificada
  lcd.cursor();             // Muestra el cursor para la entrada
  lcd.blink();

  while (true) { 
      customKey = Wait_Key_Pressed(); // Leer entrada de teclado

      if (customKey >= '0' && customKey <= '9') { 
        if (index < maxDigits) { 
          numbers[index++] = customKey;
          numbers[index] = '\0'; 
        }
      } 
      else if (customKey == '.' && decimalPoint != '*') { 
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
              lcd.noCursor(); lcd.noBlink();
              return true;  
          }
      }
      else if (customKey == 'M') {   // Salir del modo si se presiona 'M'
        exitMode = true;
        lcd.noCursor(); lcd.noBlink();
        Mode_Selection();  
        return false;  
      }  
      else if (customKey == 'C' || customKey == 'S') {  
        continue;  // Ignora 'C' y 'S'
      }

      // Borra la línea antes de reescribir para evitar residuos en pantalla
      printLCD_S(col, row, String("     ").substring(0, maxDigits));

      // Escribe la nueva entrada correctamente
      printLCD_S(col, row, String(numbers)); 
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
// Función para imprimir un mensaje de texto variable
void printLCD_S(int col, int row, const String &message) {
    lcd.setCursor(col, row);
    lcd.print(message);
  }

// Función para imprimir un mensaje con texto almacenado en FLASH
void printLCD(int col, int row, const __FlashStringHelper *message) {
  lcd.setCursor(col, row);
  lcd.print(message);
}

// Función para imprimir un mensaje con texto almacenado en FLASH
void printLCDNumber(int col, int row, float number, char unit, int decimals) {
  lcd.setCursor(col, row);
  lcd.print(number, decimals);  // Imprime el número con los decimales especificados
  
  if (unit != '\0' && unit != ' ') {  // Solo imprime la unidad si no es nula o espacio en blanco
    lcd.print(unit);
  }
}

//-------------------------------- Graba en EEPROM ------------------------------------
void saveToEEPROM(int address, float value)
{
  float PreviousValue;
  EEPROM.get(address, PreviousValue);

  if (PreviousValue != value)
  { // Solo escribe si el valor ha cambiado
    EEPROM.put(address, value);
  }
}

//--------------------------------- Lee de EEPROM -------------------------------------
float loadFromEEPROM(int address)
{
  float value;
  EEPROM.get(address, value);
  return value;
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

// ------------------------------ Mode Selection --------------------------------------
void Mode_Selection(void){

  functionIndex = (functionIndex + 1) % 6;    // Incrementa el índice de función 
  modeInitialized = false;                    // Indica a los Modos que es la 1era vez que se los llama y deben Inicializarse.
  modeConfigured = false;                     // El modo se debe configurar si corresponde
  switch (functionIndex) {                    // Cambia el modo de operación
    case 0: Mode = CC; break;                 // Selecciona Const. current Mode
    case 1: Mode = CP; break;                 // Selecciona Const. Power Mode
    case 2: Mode = CR; break;                 // Selecciona Const. Resistance Mode
    case 3: Mode = BC; break;
    case 4: Mode = TC; break;
    case 5: Mode = TL; break;
  }
}