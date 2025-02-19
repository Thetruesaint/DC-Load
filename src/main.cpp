#define WOKWI_SIMULATION

#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#ifndef WOKWI_SIMULATION
#include <Adafruit_ADS1X15.h>
#include <Adafruit_MCP4725.h>
#endif
#include <Keypad.h>
#include <EEPROM.h>
#include <math.h>
#include <RTClib.h>

// Declaramos las funciones que vamos a utilizar
void setup();
void loop();
void Load_ON_status(bool loadonoff);
void Read_Encoder();
void Read_Keypad();
void Read_Load_Button();
void Update_LCD();
void Cursor_Position(void);
void Read_Volts_Current(void);
void DAC_Control(void);
void Constant_Current(void);
void Constant_Power(void);
void Constant_Resistance(void);
void Battery_Capacity(void);
void Battery_Mode(void);
void Battery_Type_Selec(void);
void Config_Limits(void);
void Value_Input(int maxDigits);
void timer_start();
void timer_stop();
void timer_reset();
float timer_getTotalSeconds();
String timer_getTime();
void Show_Limits();
void Transient_Mode_Setup();
void Transient_Mode_Selection();
void Transient();
void Transient_List_Setup();
void Transient_Toggle_Timing();
void Transient_Toggle_Load(float current_setting, bool toggle_status);
void printLCD_S(int col, int row, const String &message);
void printLCD(int col, int row, const __FlashStringHelper *message);
void saveToEEPROM(int address, float value);
float loadFromEEPROM(int address);
void Check_Limits();
char getKeyPressed();
void Temp_Control();
void Reset_Input_Pointers();
void printLCDNumber (int col, int row, float number, char unit, int decimals = 2);

// Creamos los objetos que vamos a utilizar
#ifndef WOKWI_SIMULATION
Adafruit_MCP4725 dac; // Objeto dac para el MP4725
Adafruit_ADS1115 ads; // Objeto ads para el ADS115
#endif
LiquidCrystal_I2C lcd(0x27, 20, 4); // Objeto LCD 20x4 Address 0x27
RTC_DS1307 rtc;                     // Objeto RTC para el DS1307

//-------------------I/O Pins------------------------------------------------------------

const byte TEMP_SNSR = A0;  // Sensado de Temperatura
const byte FAN_CTRL = A2;  // Endendido de Fans
const byte ENC_A = 3;      // Encoder Pin A
const byte ENC_B = 2;      // Encoder Pin B
const byte ENC_BTN = 4;     // Encoder Boton
const byte CRR_SNSR = 1;    // Input A1 from ADC
const byte VLTG_SNSR = 3;    // Input A3 from ADC
const byte LOADONOFF = 15; // Input A1 used as a digital pin to set Load ON/OFF

//--------------- Variables para Encoder -------------------------------------------------

unsigned long lastButtonPress = 0;          // Use this to store if the encoder button was pressed or not
volatile float encoderPosition = 0;         // Antes era volatile float
volatile float factor = 0;                  // Factor de escala del Encoder
volatile unsigned long encoderMax = 999000; // sets maximum Rotary Encoder value allowed CAN BE CHANGED AS REQUIRED (was 50000)

//--------------- Variables de operacion y Modos CC, CR y CP----------------------------------------

int16_t adc1, adc3;                     // ADCs usados, adc0 y adc2 a GND
unsigned long controlVoltage = 0;       // Voltage de control para el DAC que controlara al MOSFET
float current = 0;                      // Corriente de Carga
float voltage = 0;                      // Voltage de Carga
const float MAX_VOLTAGE = 90;           // Por diseño puede medir hasta 200V, pero los MOSFET soportan solo hasta 100V
int CuPo = 8;                           // Posicion inicial del cursor
bool toggle = false;                    // Conmuta la carga On/Off
bool log_Load_data = false;                  // Flag de estado de la carga para loguear o cronometrar tiempos
float reading = 0;                      // Variable para Encoder dividido por 1000
float setCurrent = 0;                   // Variable para setear la corriente de carga
float setPower = 20;                    // Variable para setear la potencia de carga
float setResistance = 30;               // Variable para setear la resistencia de carga
float Set_Curr_Dsgn_Fact = 0.386894318; // Factor de diseño para el DAC, paso a convertir 5a1 (5V a 1V de Ref.) osea Corriente máxima 10A V1.63 = 0.4095
int CurrentCutOff;                      // Mantendra el valor de la corriente de corte seteado o cargado de la EEPROM
int PowerCutOff;                        // Mantendra el valor de la potencia de corte seteado o cargado de la EEPROM
int tempCutOff;                         // Mantendra el valor de la temperatura de corte seteado o cargado de la EEPROM
float ResistorCutOff = 999;             // Maximo valor de resistencia para el modo CR
enum ModeType { CC, CP, CR, BC, TC, TL, UNKNOWN };
ModeType Mode = CC; // Modo de operación, CC Default
const char* ModeNames[] = { "CC", "CP", "CR", "BC", "TC", "TL", "NA" };

//----------------------------------------Variables para el Keypad-------------------------------------------

const byte ROWS = 4; // Cuatro filas
const byte COLS = 4; // Cuatro columnas
// Definicion de las teclas del teclado
char hexaKeys[ROWS][COLS] = {
    {'1', '2', '3', 'M'},
    {'4', '5', '6', 'C'},
    {'7', '8', '9', 'S'},
    {'<', '0', '.', 'E'}};
int functionIndex = 1; // Es 1 para que el siguiente Modo sea Power() ya que se inicia en Current()

byte rowPins[ROWS] = {5, 6, 7, 8};    // Pineado de las filas del teclado
byte colPins[COLS] = {9, 10, 11, 12}; // Pineado de las columnas del teclado

Keypad customKeypad = Keypad(makeKeymap(hexaKeys), rowPins, colPins, ROWS, COLS);
char customKey;
char decimalPoint; // Variable para el punto decimal en la entrada de teclado
char numbers[10];  // Variable para la entrada de teclado
byte index = 0;    // poicion para el caranter de la variable numbers
float x = 0;       // Ver para que se usa

//-----------------------------------Variables de coordenadas para ubicar valores en LCD--------------------
int z = 1; // Posición en renglón (En CC,CP y CR dejo lugar para poner caracter ">"), aun no lo puse
int y = 0; // Posición provisoria
int r = 0; // Renglon

//---------------------------------------Inicializo Variables para Modo Baterias-----------------------------
float Seconds = 0;              // Variable para los segundio usada en Battery Capacity Mode (BC)
bool timerStarted = false;      // Variable para saber si el timer esta corriendo
DateTime startTime;             // Variable para el tiempo de inicio
float elapsedSeconds = 0.0;     // Variable para los segundos transcurridos
float BatteryLife = 0;          // Variable para la vida de la batería
float BatteryLifePrevious = 0;  // Variable para la vida de la batería anterior
float SecondsLog = 0;           // Variable usada para loguear el tiempo en segundos
float BatteryCutoffVolts;       // Variable usada para indicar a que Voltage se puede descargar la batería
float BatteryCurrent;           // Variable usada para setear la corriente de descargga de la bateria
float LoadCurrent;              // Almacena por un momento  a current
float LiPoCutOffVoltage = 3.5;  // Voltage mínimo de descarga para baterias LiPo
float LionCutOffVoltage = 2.8;  // Voltage mínimo de descarga para baterias Lion
float LiPoStoragVoltage = 3.8;  // Voltage mínimo de almacenamiento para baterias LiPo
float LionStoragVoltage = 3.7;  // Voltage mínimo de almacenamiento para baterias Liom
String BatteryType = "    ";    // Para definir el Tipo de Batería
bool exitMode = false;          // Para salir de la selección de tipo de batería

//---------------------------------------Variables para Control de Temperatura-----------------------------
const int FAN_ON_DRTN = 30000;  // Tiempo en miliseg. para mantener los fans encendidos (30 segundos)
const int TMP_CHK_TIME = 800;   // Perdíodo de control de temperatura (miliseg.)

int temp = 0;                   // Registra temperatura del disipador donde estan los MOSFET
unsigned long Last_tmpchk = 0;  // Tiempo desde el ùltimo chequeo de temperatura
unsigned long fan_on_time = 0;  // Tiempo que lleva encendido el Fan
bool fans_on = false;           // Flag de stado del Cooler
bool new_temp = true;           // flag si hay nuevo valor de temperatura

//---------------------------------------Variables para Modo Transient------------------------------------
float LowCurrent = 0;                  // Configuración de corriente baja para el modo transitorio
float HighCurrent = 0;                 // Configuración de corriente alta para el modo transitorio
unsigned long transientPeriod;         // Para almacenar el período de tiempo del pulso en el modo de pulso transitorio
unsigned long current_time;            // Para almacenar el tiempo actual en microsegundos
unsigned long last_time = 0;           // Para almacenar el tiempo del último cambio transitorio en microsegundos
bool transient_mode_status = false; // Para mantener el estado del modo transitorio (false = corriente baja, true = corriente alta)
float transientList[10][2];            // Array para almacenar los datos de la lista transitoria
int total_instructions;                // Utilizado en el modo de Transient List Mode
int current_instruction;               // Utilizado en el modo de Transient List Mode

//------------------------------ Posiciones reservadas en la EEMPROM --------------------------------------
const int ADD_CURRENT_CUT_OFF = 0x00;
const int ADD_POWER_CUT_OFF = 0x20;
const int ADD_TEMP_CUT_OFF = 0x40;

//---------------------------------------Variables para el Set Up-----------------------------------------
void setup() {

  //-------------------------------------Inicializa perifericos-------------------------------------------
  Serial.begin(9600); // Para Debugs y Logs
  lcd.begin(20, 4);   // initialize the lcd, default address 0x27
  rtc.begin();        // Inicializa el RTC en teoría en address 0x68
#ifndef WOKWI_SIMULATION
  ads.begin();                 // Inicializa el ADC con address 0x48
  ads.setGain(GAIN_TWOTHIRDS); // Setea la ganancia del ADC a 2/3x gain +/- 6.144V  1 bit = 0.1875mV
  dac.begin(0x60);             // Inicializa el DAC con address 0x60
#endif

  // Ver de agregar Heald Checks antes de inicializar e informar error de detectarse.

  //-------------------------------------Configuraciones iniciales de única vez-----------------------------
  // dac.setVoltage(0,true);                      // reset DAC to zero for no output current set at Switch On, Cambio a "True" para que guarde este valor en la Emprom
  // rtc.adjust(DateTime(F(__DATE__), F(__TIME__))); // Para ajsutar la hora al momento de compilar el código pero luego se debe comentar para que reloj siga corriendo

  //-------------------------------------Inicializa I/O---------------------------------------------------
  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);
  pinMode(ENC_BTN, INPUT_PULLUP);
  pinMode(LOADONOFF, INPUT_PULLUP);
  pinMode(TEMP_SNSR, INPUT);
  pinMode(FAN_CTRL, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(ENC_A), Read_Encoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B), Read_Encoder, CHANGE);

  //------------------------------------Pantalla Inicio--------------------------------------------------
  DateTime now = rtc.now();                                                                   // Obtiene la fecha y hora actual del RTC
  String date = String(now.day()) + "/" + String(now.month()) + "/" + String(now.year());     // Formatea la fecha
  String time = String(now.hour()) + ":" + String(now.minute()) + ":" + String(now.second()); // Formatea la hora
  lcd.clear();
  lcd.backlight(); // Turn on the LCD screen backlight
  printLCD(0, 0, F("*DC Electronic Load*"));
  //printLCD(0, 0, "*DC Electronic Load*");
  lcd.setCursor(0, 1); lcd.print(date + " - " + time);
  printLCD(0, 2, F("Guy Nardin"));
  printLCD(0, 3, F("v1.68"));
  /*
  Ahora:
    - Cambio de variables String Mode a Enum, costo el cambio, pero ahorro FLASH
    - Organización total de llamadas a Funciones, empiezo por CC
  Fixs: 

  Bugs:
  - En modo CR se puede poner una resistencia 0 (corriente alta, divide por cero)
  - Refresh de LCD al salir y volver al mismo modo o luego de un warning de limites excedidos.

  Mejoras ptes.:
  - Poder salir de un modo, dentro de los menues de selección. (Requiere reorganización del algorinmo de modos)
  - Poder salir del menu de configuración en cualquier momento. (Requiere reorganización del algorinmo de modos)
  - Descarga de baterias con corriente decreciente al llegar a batteryvoltagecutoff
  - Calibracíón incluir external voltage sense
  */  
  delay(2000);
  lcd.clear();
//---------------------------------------Chequea y Muestra los límites configurados----------------------
#ifndef WOKWI_SIMULATION
  CurrentCutOff = loadFromEEPROM(ADD_CURRENT_CUT_OFF); // Carga CurrentCutOff desde la EEPROM
  PowerCutOff = loadFromEEPROM(ADD_POWER_CUT_OFF);     // Carga PowerCutOff desde la EEPROM
  tempCutOff = loadFromEEPROM(ADD_TEMP_CUT_OFF);       // Carga tempCutOff desde la EEPROM
  if (CurrentCutOff < 1 || CurrentCutOff > 10 ||       // Chequea que los valores de los límites estén en el rango correcto
      PowerCutOff < 1 || PowerCutOff > 300 ||
      tempCutOff < 30 || tempCutOff > 99)
  {
    Config_Limits();
  }
  Show_Limits();
  delay(2000);
  lcd.clear();
#else
  // Simula que se cargan los valores de la EEPROM
  CurrentCutOff = 10;
  PowerCutOff = 300;
  tempCutOff = 80;
#endif

  //-------------------------------------- Modo por Default ------------------------------------------
  Constant_Current();
}

//------------------------------------- Bucle Principal-------------------------------------------------
void loop() {
  reading = encoderPosition / 1000;
  Temp_Control();
  Read_Keypad();
  Read_Load_Button();
  Cursor_Position();
  Read_Volts_Current();
  Check_Limits();
  DAC_Control();
  Battery_Capacity();
  Transient();
  Transient_Toggle_Timing();
  Update_LCD();
}

//------------------------------Load ON Status--------------------------------------
void Load_ON_status(bool loadonoff)
{
  toggle = loadonoff;
  #ifndef WOKWI_SIMULATION
  dac.setVoltage(loadonoff ? controlVoltage : 0, false); // Set DAC voltage based on load status
  #endif
  log_Load_data = loadonoff ? true : false; //
  printLCD(8, 0, loadonoff ? F("ON ") : F("OFF"));    // Fuerza la indicación del estado de la carga, por las dudas.
}

//-----------------------------Encoder Decoder--------------------------------------
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
}

//-----------------------------Read Keypad Input------------------------------------
void Read_Keypad(void) {
  customKey = customKeypad.getKey();              // Escanea el teclado   
  
  if (customKey == NO_KEY) return;                // Si no hay tecla presionada, sale de la función

  switch (customKey) {                            // Si hay una tecla presionada, la procesa
    case 'M':                                     // Cambio de Modo                    
      Load_ON_status(false);                      // Apaga la carga
      reading = 0; encoderPosition = 0;           // Resetea la posición del encoder y cualquier valor de reading
      z = 1;                                      // Resetea la posición en el renglón
      Reset_Input_Pointers();                     // Resetea el punto decimal y el indice
      switch (functionIndex) {                    // Cambia el modo de operación
        case 0: Constant_Current(); break;        // Si el modo actual es Current, cambia a Power
        case 1: Constant_Power(); break;          // Si el modo actual es Power, cambia a Resistance
        case 2: Constant_Resistance(); break;     // Si el modo actual es Resistance, cambia a Battery Capacity
        case 3: Battery_Type_Selec();             // Modo Materia
                if (exitMode) {
                  Transient_Mode_Selection(); }   // Si se sale del modo de selección de batería, inicia el modo transitorio
                if (exitMode) {                   // Si se selecciona el modo transitorio continuo
                  Constant_Current();             // Cambia a modo Constant Current
                  functionIndex = 0;              // Cambia el índice de función a 0
                } break;                              
      }
      functionIndex = (functionIndex + 1) % 4;    // Incrementa el índice de función
      break;                                      
      case 'C':                                       // Configuración de limites
          Config_Limits();                            // Configuración del usuario
          break;                                      

      case 'S':                                       // Uso futuro para Shift
          break;
  }

  // Solo permite entrada de valores en los modos CC, CP y CR
  if (Mode == BC || Mode == TC || Mode == TL) return;

  if (customKey >= '0' && customKey <= '9' && index < 5) { // Si la tecla presionada es un número, se permiten hasta 5 caracteres
      numbers[index++] = customKey;                        // Almacena el número en la variable numbers
      numbers[index] = '\0';                               // Agrega el caracter nulo al final de la cadena
      printLCD_S(z++, 3, String(customKey));                 // Muestra el número en el LCD
  }

  if (customKey == '.' && decimalPoint != '*') {           // Si la tecla presionada es un punto decimal y no se ha ingresado uno antes
      numbers[index++] = '.';                              // Almacena el punto decimal en la variable numbers
      numbers[index] = '\0';                               // Agrega el caracter nulo al final de la cadena
      printLCD(z++, 3, F("."));                               // Muestra el punto decimal en el LCD
      decimalPoint = '*';                                  // Marca que se ingresó un punto decimal
  }

  if (customKey == 'E') {                 // Confirmar entrada
      x = atof(numbers);                  // Convierte la cadena de caracteres en un número
      reading = x;                        // Asigna el valor a la variable reading  
      encoderPosition = reading * 1000;   // Asigna el valor a la variable encoderPosition
      numbers[index] = '\0';              // Resetea la cadena de caracteres
      z = 1;                              // Resetea la posición en el renglón
      printLCD(1, 3, F("     "));            // Borra el renglón del LCD
      Reset_Input_Pointers();                     // Resetea el punto decimal y el indice
  }

  // 🔹 **Manejo de borrado**
  if (customKey == '<' && index > 0) {  
    index--;  
    if (numbers[index] == '.') decimalPoint = ' '; // Si borramos un punto, permitimos otro  
    numbers[index] = '\0';  
    z--;  
    printLCD(z, 3, F(" ")); // Borra visualmente en LCD  
  }
}

//-----------------------------Toggle Current Load ON or OFF------------------------
void Read_Load_Button(void) {
  if (digitalRead(LOADONOFF) == LOW) {
      delay(200); // Anti-rebote
      toggle = !toggle;
      if (!toggle) {setCurrent = 0;} // Si la carga se apaga, resetear el valor de corriente
  }
}

//--------------------Calculate and Display Actual Voltage, Current, and Power-------------------
void Update_LCD(void) {
  static float lastCurrent = -1, lastVoltage = -1, lastPower = -1, lastReading = -1; 
  static ModeType lastMode = UNKNOWN;
  float power = 0;                  // Potencia de Carga
  power = voltage * current;

  // Evitar valores negativos por errores de medición
  if (power < 0) power = 0;
  if (voltage < 0.0) voltage = 0.0;
  if (current < 0.0) current = 0.0;

  lcd.noCursor();

  printLCD(8, 0, toggle? F("ON ") : F("OFF"));    // indica el estado de la carga.

    // Solo actualizar y formatear si el valor cambió
  if (current != lastCurrent || lastMode != Mode) {
    printLCDNumber(0, 1, current, 'A', (current < 10.0) ? 3 : 2);
    lastCurrent = current;
  }

  if (voltage != lastVoltage || lastMode != Mode) {
    printLCDNumber(7, 1, voltage, 'V', (voltage < 10.0) ? 3 : (voltage < 100.0) ? 2 : 1);
    lastVoltage = voltage;
  }

  if (power != lastPower || lastMode != Mode) {
    printLCDNumber(14, 1, power, 'W', (power < 100) ? 2 : 1);
    lastPower = power;
}

  // Actualizar modo solo si cambió
  if (Mode != lastMode) {
    printLCD_S(18, 3, ModeNames[Mode]);
  }

  // Solo actualizar LCD si la temperatura cambió
  if (new_temp) { printLCD_S(16, 0, String(temp) + String((char)0xDF) + "C");}

  // Mostrar lectura del encoder o cursor en la posición de la unidad
  if (Mode != TC && Mode != TL) {  // Evitar mostrar el encoder en modos transitorios
    if (reading != lastReading || lastMode != Mode) {
        lcd.setCursor(8, 2);
        if ((Mode == CP || Mode == CR) && reading < 100) lcd.print("0");
        if (reading < 10) lcd.print("0");
        lcd.print((Mode == CP || Mode == CR) ? String(reading, 2) : String(reading, 3));
        lastReading = reading;
    }
    lcd.setCursor(CuPo, 2); // Cursor en la unidad a modificar
    lcd.cursor();
  }
  lastMode = Mode;
}

//---------------------- Temperature Control --------------------------------------------
void Temp_Control(void) {
  unsigned long hldtmp_time = millis();
  
  if ((hldtmp_time - Last_tmpchk) >= TMP_CHK_TIME) {                           // Si pasaron TMP_CHK_TIME milisegundos, tomará la temperatura
    int act_temp = analogRead(TEMP_SNSR);                                      // Tomar temperatura del disipador
    #ifndef WOKWI_SIMULATION
    act_temp = act_temp * 0.48828125; // Convertir a Celsius
    #else
    act_temp = act_temp * 0.09765625; // Hasta 100°C con el pote de 0 a 5V que simula sensor de temperatura
    #endif

    if (temp != act_temp) {                                                   // Recalcular solo si cambió la temperatura
      temp = act_temp;
      new_temp = true;                                                        // Flag de que cambio la temperatura
    }
    else {new_temp = false;}                                                  // Si no hubo cambio deja en falso el flag

    Last_tmpchk = hldtmp_time;                                                // Actualiza el momento de chequeo de temperatura

    if (temp >= 40) {                                                         // Si la temperatura es igual o mayor a 40°C enciende el cooler
      digitalWrite(FAN_CTRL, HIGH);                                           // Encender el cooler si la temperatura es mayor o igual a 35°C
      fans_on = true;                                                         // Flag de cooler encendido
      fan_on_time = hldtmp_time;                                              // Actualizo el tiempo de encendido del cooler
    } else if (fans_on && (hldtmp_time - fan_on_time) >= FAN_ON_DRTN) {       // Si temp es < 40 va a dejar prendido el fan por FAN_ON_DRTN
        digitalWrite(FAN_CTRL, LOW);                                          // Apaga el cooler si paso el tiempo
        fans_on = false;                                                      // Flag de cooler encendido
      }
  }
}

//---------------------- Check and Enforce Limits -------------------------------------------
void Check_Limits() {
  char message[20] = "";
  float power = voltage * current;
  float maxpwrdis = constrain(140 - 0.80 * temp, 0, 120);
  float actpwrdis = max(0, power / 4);

  if (voltage > MAX_VOLTAGE) strcpy(message, "Voltage High");
  else if (current > CurrentCutOff * 1.05) strcpy(message, "Current High");
  else if (power > PowerCutOff) strcpy(message, "Power High");
  else if (temp >= tempCutOff) strcpy(message, "Over Temp");
  else if (actpwrdis >= maxpwrdis) strcpy(message, "Max PWR Dissip");
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
  Load_ON_status(false);
  reading = 0; encoderPosition = 0; setCurrent = 0;
  printLCD_S(1, 3, message);
  delay(3000);
  printLCD(0, 3, F("                "));
  }
}

//----------------------- Cursor Position (Optimized)-------------------------------------------------------
void Cursor_Position(void)
{
  // Definir la posición de la unidad en base al modo
  int unitPosition = (Mode == CP || Mode == CR) ? 10 : 9;
  // Detectar si el botón del encoder ha sido presionado
  if (digitalRead(ENC_BTN) == LOW) {
    delay(200); // Anti-rebote simple
    CuPo++;  // Incrementar la posición del cursor

    // Evitar que CP caiga en la posición del punto decimal
    if (CuPo == unitPosition + 1) {
      CuPo++;
    }
  }

  // Mantener CP dentro del rango permitido
  if (CuPo > 13) {
    CuPo = unitPosition;
  }

  // Asignar el factor de ajuste en base a la posición del cursor
  if (CuPo == unitPosition)       factor = 1000;  // Unidades
  else if (CuPo == unitPosition + 2) factor = 100;   // Decenas
  else if (CuPo == unitPosition + 3) factor = 10;    // Centenas
  else if (CuPo == unitPosition + 4) factor = 1;     // milesimas
}

//----------------------- Read Voltage and Current--------------------------------------------------------------
void Read_Volts_Current(void)
{

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
    voltage = 10;                                  // Simulo una lectura de tensión de 10V
    if (toggle) {
      if (Mode == TC || Mode == TL) { current = setCurrent;}
       else {current = (setCurrent / 1000);}
    } else {current = 0;}
  #endif
}

//---------------------- DAC Control Voltage for Mosfet---------------------------------------
void DAC_Control(void) {
  if (toggle) {
    if (Mode == CC) {
      setCurrent = reading * 1000;      // Al multiplicar por 1000 lo conviente en Amperes, no se bien porque, estudiar
      controlVoltage = setCurrent * Set_Curr_Dsgn_Fact;
    }

    if (Mode == CP) {
      setPower = reading * 1000;        // Asi queda en Watts 
      setCurrent = setPower / voltage;
      controlVoltage = setCurrent * Set_Curr_Dsgn_Fact;
    }

    if (Mode == CR) {
      setResistance = reading;                          // Asi queda em ohms, mínimo 1 ohms
      setCurrent = (voltage / setResistance) * 1000;    // para que setee en Amperes, ver bien porque
      controlVoltage = setCurrent * Set_Curr_Dsgn_Fact;
    }

    if (Mode == BC) {
      setCurrent = reading * 1000;
      controlVoltage = setCurrent * Set_Curr_Dsgn_Fact;
    }
      
    if (Mode == TC || Mode == TL) { // Transient Modes
      controlVoltage = setCurrent * 1000 * Set_Curr_Dsgn_Fact;   
    }
    #ifndef WOKWI_SIMULATION
    dac.setVoltage(controlVoltage, false); // set DAC output voltage for Range selected
    #endif
  } else {
    #ifndef WOKWI_SIMULATION
    dac.setVoltage(0, false); // set DAC output voltage to 0 if Load Off selected
    #endif
  }
}

//---------------------- Select Constant Current LCD set up--------------------------------
void Constant_Current(void) {
  Mode = CC;
  lcd.clear();
  printLCD(0, 0, F("DC LOAD"));          // Muestra el titulo del modo
  printLCD(0, 2, F("Set I = "));         // Muestra el mensaje
  printLCD(14, 2, F("A"));               // Muestra el mensaje
  printLCD(0, 3, F(">"));                // Indica la posibilidad de ingresar valores.
  CuPo = 9;                              // Pone el cursor en la posición de las unidades de Amperes
}

//--------------------- Select Constant Power LCD set up------------------------------------
void Constant_Power(void) {
  Mode = CP;
  lcd.clear();
  printLCD(0, 0, F("DC LOAD"));          // Muestra el titulo del modo
  printLCD_S(16, 0, String(temp) + String((char)0xDF) + "C");
  printLCD(0, 2, F("Set W = "));         // Muestra el mensaje
  printLCD(14, 2, F("W"));               // Muestra el mensaje
  printLCD(0, 3, F(">"));                 // Indica la posibilidad de ingresar valores.
  CuPo = 10;                            // Pone el cursor en la posición de las unidades de Potecia
}

//----------------------- Select Constant Resistance LCD set up---------------------------------------
void Constant_Resistance(void) {
  Mode = CR;
  lcd.clear();
  printLCD(0, 0, F("DC LOAD"));           // Muestra el titulo del modo
  printLCD(0, 2, F("Set R = "));          // Muestra el mensaje
  printLCD_S(14, 2, String((char)0xF4)); // Muestra el Símbolo de Ohms
  printLCD(0, 3, F(">"));                 // Indica la posibilidad de ingresar valores.
  CuPo = 10;                             // Pone el cursor en la posición de las unidades de Resistencia
}

//----------------------- Select Battery Capacity Testing LCD set up---------------------------------------
void Battery_Mode(void) {
  Mode = BC;
  lcd.clear();
  printLCD(0, 0, F("BATTERY"));          // Muestra el titulo del modo
  printLCD(0, 2, F("Set I = "));         // Muestra el mensaje
  printLCD(14, 2, F("A"));               // La unidad de corriente
  printLCDNumber(9, 3, BatteryLife, ' ', 0); // Mostrar sin decimales
  lcd.print(F("mAh"));
}

//---------------------- Battery Type Selection and Cutoff Setup -----------------------------
void Battery_Type_Selec() {
  exitMode = false;                       // Resetea EXIT mode
  lcd.noCursor();                         // Apaga el cursor para este menú
  lcd.clear();                            // Borra la pantalla del LCD
  printLCD(0, 0, F("Battery Type?"));        // Muestra el título
  printLCD(0, 1, F("Stor.: 1=LiPo 2=LiOn"));
  printLCD(0, 2, F("Disc.: 3=LiPo 4=LiOn"));
  printLCD(0, 3, F("5=Cutoff Voltage"));

  while (true) {  // 🔹 Bucle para evitar la salida accidental
    customKey = getKeyPressed(); 

    switch (customKey) {
        case '1': BatteryCutoffVolts = LiPoStoragVoltage; BatteryType = "LiPo"; break;
        case '2': BatteryCutoffVolts = LionStoragVoltage; BatteryType = "Lion"; break;
        case '3': BatteryCutoffVolts = LiPoCutOffVoltage; BatteryType = "LiPo"; break;
        case '4': BatteryCutoffVolts = LionCutOffVoltage; BatteryType = "Lion"; break;
        case '5': BatteryType = "Custom"; break;
        case 'M': case '<': exitMode = true; return; // 🔹 Salida segura del modo si el usuario decide salir
        default: continue;  // 🔹 Evita salir si la tecla es inválida
    }
    break;  // 🔹 Sale del bucle si se ingresó una tecla válida
  }

  if (BatteryType == "Custom" && !exitMode ) {
      // Pedir al usuario ingresar un voltaje de corte personalizado
      lcd.clear();
      printLCD(0, 0, F("Battery Voltage"));
      printLCD(0, 1, F("Cutoff?"));
      printLCD(0, 3, F("(0 - 25V)"));
      do {
        y = 1; z = 1; r = 2;
        printLCD(z - 1, r, F(">"));
        Value_Input(5);
      } while (x > 25 || x == 0);
      BatteryCutoffVolts = x;
  } 
  if (BatteryType != "Custom" && !exitMode) {
      // Si no es Custom, preguntar cantidad de celdas
      lcd.clear();
      printLCD_S(2, 0, "Battery "  + BatteryType);
      //printLCD(8, 1, BatteryType);
      printLCD(2, 2, F("Cells (1-6) ?"));
      
      customKey = getKeyPressed();
      if (customKey >= '1' && customKey <= '6') {
          BatteryCutoffVolts *= (customKey - '0');      // Multiplicar por la cantidad de celdas
      } else if (customKey == 'M') { exitMode = true;}   //Sale del modo
    }
  if (!exitMode){
      // Mostrar selección final
      lcd.clear();
      printLCD_S(2, 0, "Battery: " + BatteryType);
      //printLCD(8, 1, BatteryType);
      printLCD(2, 2, F("Cutoff Voltage"));
      printLCDNumber(6, 3, BatteryCutoffVolts, 'V');
      delay(3000); lcd.clear();
      printLCD_S(16, 2, BatteryType);   // Muestra el tipo de batería seleccionado
      timer_reset();                  // Resetea el timer
      BatteryLifePrevious = 0;        // Resetea la vida de la batería
      CuPo = 9;                         // Posiciona el cursor en la posición 9
      Battery_Mode();                 // Inicia el modo de capacidad de la batería
  }
}

//-------------------------------------Battery Capacity Discharge Routine--------------------------------------
void Battery_Capacity(void) {
  if (Mode != BC) return;  // Solo ejecuta si está en modo Battery Capacity

  static unsigned long lastUpdate = 0; // Guarda el tiempo de la última actualización del display
  unsigned long currentMillis = millis();
  
  if (toggle && voltage >= BatteryCutoffVolts && !timerStarted) { timer_start(); }    // Si la carga esta enable y el Timer == 2 (detenido), inicia el timer
  if (!toggle && voltage >= BatteryCutoffVolts && timerStarted) { timer_stop(); }    // Si la carga esta disable y el Timer == 1 (iniciado), detiene el timer

  
  if (currentMillis - lastUpdate >= 1000) { //  Solo actualizar cada 1 segundo para evitar flickering innecesario
    lastUpdate = currentMillis;             // Actualizar el tiempo de referencia
    
    lcd.noCursor();
    printLCD_S(0, 3, timer_getTime());  // Actualizar y mostrar tiempo transcurrido en el LCD

    Seconds = timer_getTotalSeconds();  // Obtiene el tiempo total en segundos
    LoadCurrent = (!timerStarted) ? BatteryCurrent : current; // Mantiene última corriente si se detiene el timer
    BatteryLife = round((LoadCurrent * 1000) * (Seconds / 3600));   // Calcula capacidad en mAh (sin decimales)

    if (BatteryLife > BatteryLifePrevious) {     // Solo actualizar LCD si el valor de mAh cambió
      printLCDNumber(9, 3, BatteryLife, ' ', 0); // Mostrar sin decimales
      lcd.print(F("mAh"));
      BatteryLifePrevious = BatteryLife;
    }
  }  
  
  // Si la batería llega al voltaje de corte, detener descarga
  if (voltage <= BatteryCutoffVolts) {
      BatteryCurrent = current;
      Load_ON_status(false);
      timer_stop();
  }

  // Registro de datos para logging cada segundo
  if (Mode == BC && log_Load_data && Seconds != SecondsLog) {
      SecondsLog = Seconds;
      Serial.print(SecondsLog);
      Serial.print(",");
      Serial.println(voltage);
  }
}

//------------------------Key input used for UserSetUp------------------------
void Value_Input(int maxDigits) {
  
  Reset_Input_Pointers();     // Resetea el punto decimal y el indice

  while (true) { 
      customKey = getKeyPressed(); // Leer entrada de teclado

      if (customKey >= '0' && customKey <= '9') { 
        if (index < maxDigits) { // Solo agregar si no superó el máximo
          numbers[index++] = customKey;
          numbers[index] = '\0'; // Mantener terminación de cadena
        }
      } 
      else if (customKey == '.' && decimalPoint != '*') { // Punto decimal único permitido
        if (index < maxDigits) {
          numbers[index++] = '.';
          numbers[index] = '\0';
          decimalPoint = '*'; // Marcar que se ha ingresado un punto
        }
      } 
      else if (customKey == '<' && index > 0) { // Borrar último carácter ingresado
          index--;
          
          // 🔹 Si el carácter eliminado fue un punto, permitir ingresarlo de nuevo
          if (numbers[index] == '.') {
              decimalPoint = ' '; // Restablecer permiso para ingresar punto
          }

          numbers[index] = '\0'; // Eliminar el último carácter
      } 
      else if (customKey == 'E') {        // Confirmar entrada
          if (index > 0) {                // Evitar que se confirme una entrada vacía
              x = atof(numbers);          // Convertir a número
              Reset_Input_Pointers();     // Resetea el punto decimal y el indice
              break;                      
          }
      }

      // 🛑 Borra exactamente `maxDigits` espacios antes de escribir la nueva entrada
      String clearStr = "";  
      for (int i = 0; i < maxDigits; i++) clearStr += " "; 
      printLCD_S(z, r, clearStr);  

      // ✅ Escribe la nueva entrada correctamente
      printLCD_S(z, r, String(numbers)); 
  }
}

//-------------------------------------------- Funciones para el Timer ---------------------------------------------------------

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

  int hours = totalSeconds / 3600;
  int minutes = (totalSeconds % 3600) / 60;
  int seconds = (totalSeconds % 3600) % 60;

  String formattedTime = "";
  if (hours < 10) { formattedTime += "0";}
  formattedTime += String(hours) + ":";

  if (minutes < 10) {formattedTime += "0";}

  formattedTime += String(minutes) + ":";

  if (seconds < 10) {formattedTime += "0";}
  formattedTime += String(seconds);

  return formattedTime;
}

//-------------------------------------User set up for limits-------------------------------------------------
void Config_Limits(void)
{
  Load_ON_status(false); // Apaga la carga
  Show_Limits();
  delay(2000);
  lcd.clear();           // Borra la pantalla del LCD

  printLCD(4, 0, F("Set Limits"));
  printLCD(0, 1, F("Current(A):"));
  y = 12; z = 12; r = 1;
  Value_Input(4);
  CurrentCutOff = min(x, 10);
  printLCDNumber(y, r, CurrentCutOff,' ');
  
  printLCD(0, 2, F("Power(W):"));
  r = 2; z = 12;
  Value_Input(3);
  PowerCutOff = min(x, 300);
  printLCDNumber(y, r, PowerCutOff, ' ',0);
 
  printLCD(0, 3, F("Temp.("));
  printLCD_S(6, 3, String((char)0xDF) + "C):");
  z = 12; r = 3;
  Value_Input(2);
  tempCutOff = min(x, 99);
  printLCD_S(y, r, String(tempCutOff));

  saveToEEPROM(ADD_CURRENT_CUT_OFF, CurrentCutOff);
  saveToEEPROM(ADD_POWER_CUT_OFF, PowerCutOff);
  saveToEEPROM(ADD_TEMP_CUT_OFF, tempCutOff);

  Show_Limits();
  delay(2000);
  lcd.clear();           // Borra la pantalla del LCD
  Constant_Current();    // Ir a modo Corriente Constante (default)
}

//-----------------------------Show limits Stored Data for Current, Power and Temp-----------------------------
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
  printLCD_S(9, 3, String(tempCutOff));
  printLCD_S(11, 3, String((char)0xDF) + "C");
}

//----------------------------------------Transient Type Selection--------------------------------------------
void Transient_Mode_Selection() {
  toggle = false;
  exitMode = false;
  lcd.noCursor(); lcd.clear();                    // Apaga el cursor y borra la pantalla del LCD

  printLCD(3, 0, F("Transient Mode"));
  printLCD(0, 1, F("1 = Continuous"));
  printLCD(0, 2, F("2 = List"));

  while (true) {                                  // Bucle para garantizar entrada válida
    customKey = getKeyPressed();

    if (customKey == '1' || customKey == '2') {   // Si la tecla es válida
      Mode = (customKey == '1') ? TC : TL;        // Asigna el modo de transitorio
      break;
    }
    else if (customKey == 'M') {                  // Si se presiona la tecla de Modos
      exitMode = true;                            // Salimos del modo
      break;
    }
    else if (!strchr("34567890<CSE.", customKey) && customKey != NO_KEY) {  // Si la tecla no es válida
      break;                                                                // Salimos del bucle si la tecla no es inválida
    }
  }
  if (!exitMode) {Transient_Mode_Setup(); }       // Si no se sale del modo, se llama a la función de modo transitorio
}

//----------------------------------------Transient Mode--------------------------------------------
void Transient_Mode_Setup(void) {
  if (Mode == TC) {

    lcd.clear(); // Apaga el cursor y borra la pantalla del LCD

    printLCD(3, 0, F("Transient Cont."));
    printLCD(0, 1, F("Low I (A):"));
     y = 11; z = 11; r = 1;                     // Setea las posiciones de la pantalla del LCD
    Value_Input(5);                             // Obtiene el valor ingresado por el usuario
    LowCurrent = min(x, CurrentCutOff);         // Limita la corriente baja al valor de corte de corriente
    printLCDNumber(11, r, LowCurrent, 'A', 3);  // Muestra el valor de la corriente baja

    printLCD(0, 2, F("High I (A):"));
    z = 11; r = 2;
    Value_Input(5);                             // Obtiene el valor ingresado por el usuario
    HighCurrent = min(x, CurrentCutOff);        // Limita la corriente alta al valor de corte de corriente
    printLCDNumber(11, r, HighCurrent, 'A', 3); // Muestra el valor de la corriente alta con tres decimales

    printLCD(0, 3, F("Time (mSec):"));
    z = 12; r = 3;                              // Setea las posiciones de la pantalla del LCD
    Value_Input(5);                             // Obtiene el valor ingresado por el usuario
    transientPeriod = x;                        // Guarda el valor del tiempo de transitorio
    lcd.clear();                                // Borra la pantalla del LCD
    Load_ON_status(false);                      // Apaga la carga
  }
  if (Mode == TL)  {
    Transient_List_Setup();                     // Si el modo es Transitorio de Lista, se llama a la función de configuración de la lista
    Load_ON_status(false);                      // Apaga la carga
  }
}

//-------------------------------------Transcient List Setup-------------------------------------------
void Transient_List_Setup()
{
  lcd.noCursor();
  lcd.clear(); // Apaga el cursor y borra la pantalla
  printLCD(0, 0, F("Transient List"));
  printLCD(0, 1, F("How many?"));
  printLCD(0, 2, F("(2 to 10)"));
  lcd.cursor();
  do { // Bucle para garantizar entrada válida
    y = 0; z = 1; r = 3;
    printLCD(0, r, F(">"));
    Value_Input(2);
  } while (x < 2 || x > 10); // Si el valor no está entre 2 y 10, se vuelve a pedir

  total_instructions = x - 1; // Guarda el número total de instrucciones
  lcd.clear();                // Borra la pantalla del LCD
  lcd.cursor();
  for (int i = 0; i <= total_instructions; i++) {   // Bucle para obtener los valores de la lista
    printLCD_S(0, 0, "Instruccion " + String(i + 1));
    printLCD_S(0, 1, F("Current (A):")); // Pide el valor de corriente en Amperes
    y = 12; z = 12; r = 1;
    Value_Input(5);                   // Permitir 5 digitos, ej.: 1.234
    x = min(x, CurrentCutOff);        // Limita a CutOff
    printLCDNumber(y , r, x, 'A', 3); // Muestra el valor de la corriente
    transientList[i][0] = x;          // Lo guarda en la lista

    printLCD(0, 2, F("Time (mSec):"));   // Pide el valor de tiempo en milisegundos
     y = 12; z = 12; r = 2;
    Value_Input(5);
    transientList[i][1] = x;                        // Guarda el valor del tiempo
    lcd.clear(); // Borra la pantalla
  }
  lcd.noCursor();
  current_instruction = 0; // Resetea el contador de instrucciones porque finalizo la configuración
}

//-------------------------------------Transcient Load Toggel-------------------------------------------
void Transient_Toggle_Timing() {

  if (!toggle) return;

  if (Mode == TC) {           // Si em modo es Transitorio Continuo
    current_time = micros();
    if (last_time == 0) { last_time = current_time; }
    else {
      switch (transient_mode_status) {
        case (false):
          if ((current_time - last_time) >= (transientPeriod * 1000.0)) { // Comprueba si el tiempo transcurrido es mayor o igual al tiempo de transitorio
            Transient_Toggle_Load(LowCurrent, true);                      // Cambia a la corriente baja
          }
          break;
        case (true):
          if ((current_time - last_time) >= (transientPeriod * 1000.0)) { // Comprueba si el tiempo transcurrido es mayor o igual al tiempo de transitorio
            Transient_Toggle_Load(HighCurrent, true);                     // Cambia a la corriente alta
          }
          break;
      }
    }
  }

  if (Mode == TL) {     // Si el modo es Transitorio de Lista
      current_time = micros();
      if (last_time == 0) {                                      // Si es la primera vez que se ejecuta
        last_time = current_time;                                // Inicializa el tiempo
        transientPeriod = transientList[current_instruction][1]; // Obtiene el tiempo de transitorio de la lista
        Transient_Toggle_Load(transientList[current_instruction][0], false);
      }
      if ((current_time - last_time) >= transientList[current_instruction][1] * 1000) { // Comprueba si el tiempo transcurrido es mayor o igual al tiempo de transitorio de la lista
        current_instruction++;                                                          // Incrementa el contador de instrucciones
        if (current_instruction > total_instructions)
        {                                                                               // Si el contador es mayor al total de instrucciones
          current_instruction = 0;                                                      // Resetea el contador de instrucciones
        }
        transientPeriod = transientList[current_instruction][1];                        // Obtiene el tiempo de transitorio de la lista
        Transient_Toggle_Load(transientList[current_instruction][0], false);            // Cambia a la corriente de la lista
      }
  }
}

//-------------------------------------Transcient Switch-------------------------------------------
void Transient_Toggle_Load(float current_setting, bool toggle_status)
{
  if (toggle_status) { transient_mode_status = !transient_mode_status;}
  setCurrent = current_setting;
  last_time = current_time;
}

//----------------------------------------Transient--------------------------------------------
void Transient(void) {

  static ModeType last_T_Mode = UNKNOWN;

  if (Mode == TC) {
    printLCD_S(3, 2, String(LowCurrent, 3));   // Muestra el valor de la corriente baja
    printLCD_S(14, 2, String(HighCurrent, 3)); // Muestra el valor de la corriente alta
    if (last_T_Mode != Mode) {
      lcd.noCursor();            // switch Cursor OFF for this menu
      printLCD(0, 0, F("DC LOAD")); // Muestra el titulo del modo
      printLCD(0, 2, F("Lo="));                   // Muestra el mensaje
      printLCD(8, 2, F("A"));                     // Muestra la unidad
      printLCD(11, 2, F("Hi="));                  // Muestra el mensaje
      printLCD(19, 2, F("A"));                    // Muestra la unidad
      printLCD(0, 3, F("Time = "));               // Muestra el mensaje
      printLCD_S(7, 3, String(transientPeriod)); // Muestra el valor del tiempo
      printLCD(12, 3, F("mSecs"));                // Muestra la unidad
      last_T_Mode = Mode;
      }
    }

  if (Mode == TL) {
    printLCD_S(13, 2, String(current_instruction + 1));   // Muestra la instrucción en curso
    printLCD_S(7, 3, String(transientPeriod));        // Muestra el valor del tiempo
    if (last_T_Mode != Mode) {
      lcd.noCursor();   
      printLCD(0, 0, F("DC LOAD"));                    // Muestra el titulo del modo
      printLCD(0, 2, F("Instruccion: "));              // Muestra el mensaje
      printLCD(0, 3, F("Time = "));                    // Muestra el mensaje
      printLCD(12, 3, F("mSecs"));                     // Muestra la unidad
      last_T_Mode = Mode;
    }
  }
}

//------------------------------------- Funciones para el LCD -------------------------------------------
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
 
// Función para imprimir un numero y su unidad en una posición específica
void printLCDNumber(int col, int row, float number, char unit, int decimals) {
  lcd.setCursor(col, row);

  // Buffer de 10 caracteres para el número formateado
  char buffer[10];

  // Formato del número con "X.YY" o "XX.Y" dependiendo del valor
  dtostrf(number, 6, decimals, buffer);

  // Elimina espacios al inicio generados por dtostrf
  for (int i = 0; i < 6; i++) {
    if (buffer[i] != ' ') {
      lcd.print(&buffer[i]);  // Imprime desde la primera posición válida
      break;
    }
  }
    lcd.print(unit);  // Imprime la unidad
}

//-------------------------------- Graba en EEPROM -----------------------------
void saveToEEPROM(int address, float value)
{
  float PreviousValue;
  EEPROM.get(address, PreviousValue);

  if (PreviousValue != value)
  { // Solo escribe si el valor ha cambiado
    EEPROM.put(address, value);
  }
}

//-------------------------------- Lee de EEPROM -----------------------------
float loadFromEEPROM(int address)
{
  float value;
  EEPROM.get(address, value);
  return value;
}

//------------------------------- Get Key Presed ---------------------------------
char getKeyPressed() {
  char key;
  do {
      key = customKeypad.getKey();
  } while (key == NO_KEY);  
  return key;
}

//------------------------------- Reset Input Pointers ----------------------------
void Reset_Input_Pointers (void){
  index = 0;
  decimalPoint = ' ';
}