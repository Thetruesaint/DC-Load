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
void load_ON_status(boolean loadonoff);
void read_encoder();
void readKeypadInput();
void Load_Switch_Check();
void UpdateLCD_IVP_values();
void displayEncoderReading(void);
void CursorPosition(void);
void readVoltageCurrent(void);
void dacControlVoltage(void);
void dacControl(void);
void Current(void);
void Power(void);
void Resistance(void);
void batteryCapacity(void);
void BatteryCapacityMode(void);
void batteryType(void);
void userSetUp(void);
void inputValue(int maxDigits);
int timer_status();
void timer_start();
void timer_stop();
void timer_reset();
float timer_getTotalSeconds();
String timer_getTime();
void setupLimits(void);
void transientMode(void);
void transientType(void);
void Transient(void);
void transientListSetup();
void transientLoadToggle();
void transientSwitch(float current_setting, boolean toggle_status);
void clearLCDLine(int row);
void printLCD(int col, int row, const String &message);
void saveToEEPROM(int address, float value);
float loadFromEEPROM(int address);
void checkLimits(void);
char getKeyPressed();

// Creamos los objetos que vamos a utilizar
#ifndef WOKWI_SIMULATION
Adafruit_MCP4725 dac; // Objeto dac para el MP4725
Adafruit_ADS1115 ads; // Objeto ads para el ADS115
#endif
LiquidCrystal_I2C lcd(0x27, 20, 4); // Objeto LCD 20x4 Address 0x27
RTC_DS1307 rtc;                     // Objeto RTC para el DS1307

//-------------------I/O Pins------------------------------------------------------------

const byte templm35 = A0;  // Sensado de Temperatura
const byte fansctrl = A2;  // Endendido de Fans
const byte ENC_A = 3;      // Encoder Pin A
const byte ENC_B = 2;      // Encoder Pin B
const byte encbtn = 4;     // Encoder Boton
const byte crrsnsr = 1;    // Input A1 from ADC
const byte vltgmtr = 3;    // Input A3 from ADC
const byte LoadOnOff = 15; // Input A1 used as a digital pin to set Load ON/OFF

//--------------- Variables para Encoder -------------------------------------------------

unsigned long lastButtonPress = 0;          // Use this to store if the encoder button was pressed or not
volatile float encoderPosition = 0;         // Antes era volatile float
volatile float factor = 0;                  // Factor de escala del Encoder
volatile unsigned long encoderMax = 999000; // sets maximum Rotary Encoder value allowed CAN BE CHANGED AS REQUIRED (was 50000)

//--------------- Variables de operacion y Modos CC, CR y CP----------------------------------------

int16_t adc0, adc1, adc2, adc3;
unsigned long controlVoltage = 0;       // Voltage de control para el DAC que controlara al MOSFET
float current = 0;                      // Corriente de Carga
float voltage = 0;                      // Voltage de Carga
float ActualVoltage = 0;                // Lectura de Voltage de Carga
float ActualCurrent = 0;                // Lectura de Corriente de Carga
float ActualPower = 0;                  // Potencia de Carga
int CP = 8;                             // Posicion inicial del cursor
boolean toggle = false;                 // Conmuta la carga On/Off
int Load = 0;                           // Load On/Off flag de estado de la carga
float reading = 0;                      // Variable para Encoder dividido por 1000
float setCurrent = 1.000;               // Variable para setear la corriente de carga
float setPower = 20;                    // Variable para setear la potencia de carga
float setResistance = 30;               // Variable para setear la resistencia de carga
float Set_Curr_Dsgn_Fact = 0.386894318; // Factor de diseño para el DAC, paso a convertir 5a1 (5V a 1V de Ref.) osea Corriente máxima 10A V1.63 = 0.4095
float setControlCurrent = 0;            // Variable temporaria para cargar el valor de corriente de control... ver si sirve
int setReading = 0;                     // Variable usada en la funcion dacControlVoltage() pero nada mas... ver si sirve
int CurrentCutOff;                      // Mantendra el valor de la corriente de corte seteado o cargado de la EEPROM
int PowerCutOff;                        // Mantendra el valor de la potencia de corte seteado o cargado de la EEPROM
int tempCutOff;                         // Mantendra el valor de la temperatura de corte seteado o cargado de la EEPROM
float ResistorCutOff = 999;             // Maximo valor de resistencia para el modo CR
String Mode = "  ";                     // Modo de operación

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
float LoadCurrent;              // Almacena por un momento el ActualCurrent
float MaxBatteryCurrent = 10.0; // Corriente máxima de descarga de la batería. Ver si no puede definirse como constante
float LiPoCutOffVoltage = 3.5;  // Voltage mínimo de descarga para baterias LiPo
float LionCutOffVoltage = 2.8;  // Voltage mínimo de descarga para baterias Lion
float LiPoStoragVoltage = 3.8;  // Voltage mínimo de almacenamiento para baterias LiPo
float LionStoragVoltage = 3.7;  // Voltage mínimo de almacenamiento para baterias Liom
String BatteryType = "    ";    // Para definir el Tipo de Batería
byte exitMode = 0;              // Para salir de la selección de tipo de batería

//---------------------------------------Variables para Control de Temperatura-----------------------------
int temp = 0;                      // Registra temperatura del disipador donde estan los MOSFET
const int tmpchk_time = 500;       // Perdíodo de control de temperatura (miliseg.)
unsigned long Last_tmpchk = 0;     // Tiempo desde el ùltimo chequeo de temperatura
const int fan_on_duration = 30000; // Tiempo en miliseg. para mantener los fans encendidos (30 segundos)
unsigned long fan_on_time = 0;     // Tiempo que lleva encendido el Fan
bool fans_on = false;              // Flag para saber si los fans estan encendidos

//---------------------------------------Variables para Modo Transient------------------------------------
float LowCurrent = 0;                  // Configuración de corriente baja para el modo transitorio
float HighCurrent = 0;                 // Configuración de corriente alta para el modo transitorio
unsigned long transientPeriod;         // Para almacenar el período de tiempo del pulso en el modo de pulso transitorio
unsigned long current_time;            // Para almacenar el tiempo actual en microsegundos
unsigned long last_time = 0;           // Para almacenar el tiempo del último cambio transitorio en microsegundos
boolean transient_mode_status = false; // Para mantener el estado del modo transitorio (false = corriente baja, true = corriente alta)
float transientList[10][2];            // Array para almacenar los datos de la lista transitoria
int total_instructions;                // Utilizado en el modo de Transient List Mode
int current_instruction;               // Utilizado en el modo de Transient List Mode

//------------------------------ Posiciones reservadas en la EEMPROM --------------------------------------
const int ADD_CURRENT_CUT_OFF = 0x00;
const int ADD_POWER_CUT_OFF = 0x20;
const int ADD_TEMP_CUT_OFF = 0x40;

//---------------------------------------Variables para el Set Up-----------------------------------------
void setup()
{

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
  pinMode(encbtn, INPUT_PULLUP);
  pinMode(LoadOnOff, INPUT_PULLUP);
  pinMode(templm35, INPUT);
  pinMode(fansctrl, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(ENC_A), read_encoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B), read_encoder, CHANGE);

  //------------------------------------Pantalla Inicio--------------------------------------------------
  DateTime now = rtc.now();                                                                   // Obtiene la fecha y hora actual del RTC
  String date = String(now.day()) + "/" + String(now.month()) + "/" + String(now.year());     // Formatea la fecha
  String time = String(now.hour()) + ":" + String(now.minute()) + ":" + String(now.second()); // Formatea la hora
  lcd.clear();
  lcd.backlight(); // Turn on the LCD screen backlight
  printLCD(1, 0, "DC Electronic Load");
  lcd.setCursor(1, 1);
  lcd.print(date + " - " + time);
  printLCD(1, 2, "Guy Nardin");
  printLCD(1, 3, "v1.65"); // Modos CC, CP y CR funcionando con Teclado. Ajustes para el Set con Load ON/Off
                           // Vuelvo a configurar que guarde Set Up en EEPROM (solo guarda enteros)
                           // coloco LM35
                           // Cambio de Ops Apms a LM324N alimentado a 9V
                           // 1.39 No podia superar los 4,9A el MCP4725 parece llegar al lìmite no sube mas de 4.987V y luego cae a 300mV cbio. Factor del DAC,
                           // paso a convertir 5a1 (5V a 1V de Ref.) osea Corriente máxima 10A
                           // 1.40 Agrego borrado de entrada de teclado con el boton "*" por si comento un error
                           // 1.41 Agrego RTC y las funciones de timer
                           // 1.42 Agrego Modo para prueba de Baterias,
                           // 1.43 Subo sensibilidad de lectura de corriente para Reading < 500,
                           // 1.44 Cambia el Modo con la tecla "MOD".
                           // 1.45 Remapeo de teclado, agrego el circuito autoLoadOff pero cambia el Vset del control de corriente. Vere luego si con calibraciòn se resuelve.
                           // 1.46 Agrego pin de control para encender los dos Cooler.
                           // 1.47 Volcado a PCBs con nueva RS, recalibro. Agrego promedio de temperatura y encendido de fans temporizado
                           // 1.48 Cambio el reset DAC a zero pero con "true" para que guarde este valor en la Emprom y cuando se encienda no active los MOSFET
                           // 1.49 Activo sensor de V diferencial con relacion 50 a 1 (0.02), adapto el divisor resistivo y escalo ganancia para sensibilidad a distintas escalas.
                           // 1.50 Cambio RShut (medi 0.99ohms), cbio. lim a 10A, controlo DAC y linealidad de control de corriente y agrego cbio de PGA para mas de 9,9A
                           // 1.51 Montaje en Gabinete y cambio de criterio de calibración de V e I, factor depende de ganancia del ADC.
                           // 1.52 Cambios en la función de Temperatura y corrección de Bugs
                           // 1.53 Agrego numero de celdas, unifico funcion de control de lìmites, limite de digitos para la funciòn ImputValue, mejoras en el SetupLimits y entrada de datos.
                           // 1.54 Creo funcion para display y estado del "Load_ON_Status"
                           // 1.60 Pruebo agregar Modo Trasient Continuo y Listado.. sin Trigger
                           // 1.61 Ajustes del còdigo para poner lìmites en modos Trasient
                           // 1.62 Cambio opciones de Descarga de Baterias
                           // 1.63 Bug en Trasient Mode que hace que no chequee los límites de corriente, temperatura o potencia.
                           // 1.64 Muchas mejoras en el código, en el manejo EEPROM, temp de Fans on a 40°C y ajusto factor de control de corriente.
                           // 1.65 Nuevas mejoras en el código, tal vez en refresh del display y agregar calibracíón o mejora en la descarga de baterias
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
    userSetUp();
  }
  setupLimits();
  delay(2000);
  lcd.clear();
#else
  // Simula que se cargan los valores de la EEPROM
  CurrentCutOff = 10;
  PowerCutOff = 300;
  tempCutOff = 80;
#endif

  //-------------------------------------- Pantalla por Default ------------------------------------------
  load_ON_status(false); // indicate that LOAD is off at start up
  Current();
}

//------------------------------------- Bucle Principal-------------------------------------------------
void loop()
{
  readKeypadInput();   // read Keypad entry
  Load_Switch_Check(); // Load on/off
  checkLimits();
  Transient();         // test for Transient Mode
  printLCD(18, 3, Mode);
  if (Mode != "TC" && Mode != "TL")
  {                                   // if NOT transient mode then Normal Operation
    reading = encoderPosition / 1000; // read input from rotary encoder
    displayEncoderReading();          // display rotary encoder input reading on LCD
    CursorPosition();                 // check and change the cursor position if cursor button pressed
  }
  else
  {
    transientLoadToggle();            // Start Transient Mode
  } 

  readVoltageCurrent();      // routine for ADC's to read actual Voltage and Current
  UpdateLCD_IVP_values();    // Display actual Voltage, Current readings and Actual Wattage
  dacControl();
  dacControlVoltage();       // sets the drive voltage to control the MOSFET
  batteryCapacity();         // test if Battery Capacity (BC) mode is selected - if so action
}

//------------------------------Load ON Status--------------------------------------
void load_ON_status(boolean loadonoff)
{
  printLCD(8, 0, loadonoff ? "ON " : "OFF");
#ifndef WOKWI_SIMULATION
  dac.setVoltage(loadonoff ? controlVoltage : 0, false); // Set DAC voltage based on load status
#endif
  Load = loadonoff ? 1 : 0;
}

//-----------------------------Encoder Decoder--------------------------------------
void read_encoder()
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
void readKeypadInput(void) {
  customKey = customKeypad.getKey();                  // Escanea el teclado   
  
  if (customKey == NO_KEY) return;                    // Si no hay tecla presionada, sale de la función

  switch (customKey) {                                // Si hay una tecla presionada, la procesa
      case 'M':  // Cambio de Modo                    
          toggle = false;                             // flag que indica que la carga esta apagada
          load_ON_status(false);                      // Apaga la carga
          switch (functionIndex) {                    // Cambia el modo de operación
              case 0: Current(); break;               // Si el modo actual es Current, cambia a Power
              case 1: Power(); break;                 // Si el modo actual es Power, cambia a Resistance
              case 2: Resistance(); break;            // Si el modo actual es Resistance, cambia a Battery Capacity
              case 3:                                 // Si el modo actual es Battery Capacity, cambia a Current  
                  batteryType();                      // Selecciona el tipo de batería
                  if (exitMode == 0) {                // Si no se sale del modo de selección de batería
                      printLCD(16, 2, BatteryType);   // Muestra el tipo de batería seleccionado
                      load_ON_status(false);          // Apaga la carga
                      timer_reset();                  // Resetea el timer
                      BatteryLifePrevious = 0;        // Resetea la vida de la batería
                      CP = 9;                         // Posiciona el cursor en la posición 9
                      BatteryCapacityMode();              // Inicia el modo de capacidad de la batería
                  } else {
                      transientType();                // Si se sale del modo de selección de batería, inicia el modo transitorio
                      if (exitMode == 1) {            // Si se selecciona el modo transitorio continuo
                          Current();                  // Cambia a Current
                          functionIndex = 0;          // Cambia el índice de función a 0
                      }
                  }
                  break;                              // Sale del switch
          }
          functionIndex = (functionIndex + 1) % 4;    // Incrementa el índice de función
          encoderPosition = 0;                        // Resetea la posición del encoder
          index = 0;                                  // Resetea el índice
          z = 1;                                      // Resetea la posición en el renglón
          decimalPoint = ' ';                         // Resetea el punto decimal
          break;                                      // Sale del switch

      case 'C':  // Configuración del usuario
          toggle = false;                             // flag que indica que la carga esta apagada
          userSetUp();                                // Configuración del usuario
          encoderPosition = 0;                        // Resetea la posición del encoder
          index = 0;                                  // Resetea el índice
          z = 1;                                      // Resetea la posición en el renglón
          decimalPoint = ' ';                         // Resetea el punto decimal
          break;                                      // Sale del switch

      case 'S':  // Uso futuro para Shift
          break;
  }

  if (Mode == "BC") return;                                // Si el modo es Battery Capacity, sale de la función

  if (customKey >= '0' && customKey <= '9' && index < 6) { // Si la tecla presionada es un número y el índice es menor a 6
      numbers[index++] = customKey;                        // Almacena el número en la variable numbers
      numbers[index] = '\0';                               // Agrega el caracter nulo al final de la cadena
      printLCD(z++, 3, String(customKey));                 // Muestra el número en el LCD
  }

  if (customKey == '.' && decimalPoint != '*') {           // Si la tecla presionada es un punto decimal y no se ha ingresado uno antes
      numbers[index++] = '.';                              // Almacena el punto decimal en la variable numbers
      numbers[index] = '\0';                               // Agrega el caracter nulo al final de la cadena
      printLCD(z++, 3, ".");                               // Muestra el punto decimal en el LCD
      decimalPoint = '*';                                  // Marca que se ingresó un punto decimal
  }

  if (customKey == 'E') {                 // Confirmar entrada
      x = atof(numbers);                  // Convierte la cadena de caracteres en un número
      reading = x;                        // Asigna el valor a la variable reading  
      encoderPosition = reading * 1000;   // Asigna el valor a la variable encoderPosition
      index = 0;                          // Resetea el índice
      numbers[index] = '\0';              // Resetea la cadena de caracteres
      z = 1;                              // Resetea la posición en el renglón
      printLCD(0, 3, "        ");         // Borra el renglón del LCD
      decimalPoint = ' ';                 // Resetea el punto decimal
  }

  if (customKey == '<') {         // Borrar entrada
      index = 0;                  // Resetea el índice
      z = 1;                      // Resetea la posición en el renglón
      printLCD(0, 3, "        "); // Borra el renglón del LCD
      numbers[index] = '\0';      // Resetea la cadena de caracteres
      decimalPoint = ' ';         // Resetea el punto decimal
  }
}

//-----------------------------Toggle Current Load ON or OFF------------------------
void Load_Switch_Check(void) {
  if (digitalRead(LoadOnOff) == LOW) {
      delay(200); // Anti-rebote
      toggle = !toggle;
      load_ON_status(toggle);
      
      if (toggle) {
          clearLCDLine(3); // Borra el 4to. Renlón del LCD si la carga está encendida
      } else {
          setCurrent = 0; // Resetea la corriente de carga si la carga está apagada
      }
  }
}

//--------------------Calculate and Display Actual Voltage, Current, and Power-------------------
void UpdateLCD_IVP_values(void) {
  static float lastCurrent = -1, lastVoltage = -1, lastPower = -1;
  static String lastMode = "-1";

  ActualCurrent = current;
  ActualVoltage = voltage;
  ActualPower = ActualVoltage * ActualCurrent;

  // Evitar valores negativos por errores de medición
  if (ActualPower < 0) ActualPower = 0;
  if (ActualVoltage < 0.0) ActualVoltage = 0.0;
  if (ActualCurrent < 0.0) ActualCurrent = 0.0;

  // Formateo con 2 decimales, asegurando siempre un ancho fijo
  String currentStr = (ActualCurrent < 10.0) ? String(ActualCurrent, 3) + "A" : String(ActualCurrent, 2) + "A";
  String voltageStr = (ActualVoltage < 10.0) ? String(ActualVoltage, 3) + "V" :
                    (ActualVoltage < 100.0) ? String(ActualVoltage, 2) + "V" :
                                              String(ActualVoltage, 1) + "V";
  String powerStr   = (ActualPower < 100) ? String(ActualPower, 2) + "W" : String(ActualPower, 1) + "W";

  // Forzar alineación con 7 caracteres fijos para corriente y voltaje, 5 para potencia
  while (currentStr.length() < 7) currentStr = currentStr + " ";
  while (voltageStr.length() < 7) voltageStr = voltageStr + " ";
  while (powerStr.length() < 7) powerStr = powerStr + " ";

  // Solo actualizar LCD si el valor cambió
  if (ActualCurrent != lastCurrent || lastMode != Mode) {
      printLCD(0, 1, currentStr);  // Ej: " 9.99A"
      lastCurrent = ActualCurrent;
  }

  if (ActualVoltage != lastVoltage || lastMode != Mode) {
      printLCD(7, 1, voltageStr);  // Ej: "29.73V"
      lastVoltage = ActualVoltage;
  }

  if (ActualPower != lastPower || lastMode != Mode) {
      printLCD(14, 1, powerStr);   // Ej: "300W"
      lastPower = ActualPower;
  }
  lastMode = Mode;
}

//---------------------- Check and Enforce Limits -------------------------------------------
void checkLimits(void) {
  bool limitExceeded = false;
  String message = "";

  // Verificar límites según el modo
  if ((Mode == "CC" && reading > CurrentCutOff) ||
      (Mode == "CP" && reading > PowerCutOff) ||
      (Mode == "CR" && reading > ResistorCutOff) ||
      (Mode == "BC" && reading > MaxBatteryCurrent)) {
      
      reading = (Mode == "CC") ? CurrentCutOff :
                (Mode == "CP") ? PowerCutOff :
                (Mode == "CR") ? ResistorCutOff : MaxBatteryCurrent;
      
      encoderPosition = reading * 1000;
      clearLCDLine(3);
  }

  // Verificar límite de potencia
  if (ActualPower > PowerCutOff) {
      message = "Exceeded Power";
      limitExceeded = true;
  }

  // Verificar límite de temperatura
  unsigned long currentTime = millis();
  if ((currentTime - Last_tmpchk) >= tmpchk_time) {
      Last_tmpchk = currentTime;
      temp = analogRead(templm35); // Tomar temperatura
      #ifndef WOKWI_SIMULATION
          temp = temp * 0.48828125; // Convertir a Celsius
      #else
          temp = temp * 0.09765625; // Hasta 100°C con el pote de 0 a 5V que simula sensor de temperatura
      #endif

      printLCD(16, 0, String(temp) + String((char)0xDF) + "C"); // Muestra la temperatura

      if (temp >= 40) { // Encender ventiladores si la temperatura es mayor o igual a 40°C
          digitalWrite(fansctrl, HIGH);
          fans_on = true;
          fan_on_time = currentTime;
      } else if (fans_on && (currentTime - fan_on_time) >= fan_on_duration) {
          digitalWrite(fansctrl, LOW); // Apagar los ventiladores después del tiempo especificado
          fans_on = false;
      }

      if (temp >= tempCutOff) { // Si se alcanza la temperatura máxima
          message = "Over Temperature";
          limitExceeded = true;
      }
  }

  // Si se superó algún límite de potencia o temperatura, se apaga la carga
  if (limitExceeded) {
      reading = 0;
      encoderPosition = 0;
      clearLCDLine(3);
      printLCD(0, 3, message);
      load_ON_status(false);
      toggle = false;
      delay(3000);
      clearLCDLine(3);
  }
}

//----------------------Display Rotary Encoder Input Reading on LCD---------------------------
void displayEncoderReading(void)
{

  lcd.setCursor(8, 2); // start position of setting entry

  if ((Mode == "CP" || Mode == "CR") && reading < 100)
  {
    lcd.print("0");
  }

  if (reading < 10)
  { // add a leading zero to display if reading less than 10
    lcd.print("0");
  }

  if (Mode == "CP" || Mode == "CR")
  {
    lcd.print(reading, 2); // show input reading from Rotary Encoder on LCD
  }
  else
  {
    lcd.print(reading, 3);
  }
  lcd.setCursor(CP, 2); // sets cursor position
  lcd.cursor();         // show cursor on LCD
}

//--------------------------Cursor Position-------------------------------------------------------
void CursorPosition(void)
{

  // Defaults for two digits before decimal and 3 after decimal point
  int unitPosition = 9;

  // Power and Resistance modes can be 3 digit before decimal but only 2 decimals
  if (Mode == "CP" || Mode == "CR")
  {
    unitPosition = 10;
  }

  if (digitalRead(encbtn) == LOW)
  {

    delay(100); // simple key bounce delay
    CP = CP + 1;
    if (CP == unitPosition + 1)
    {
      CP = CP + 1;
    }
  }

  if (CP > 13)
  {
    CP = unitPosition;
  } // No point in turning tens and hundreds
  if (CP == unitPosition + 4)
  {
    factor = 1;
  }
  if (CP == unitPosition + 3)
  {
    factor = 10;
  }
  if (CP == unitPosition + 2)
  {
    factor = 100;
  }
  if (CP == unitPosition)
  {
    factor = 1000;
  }
}

//---------------------------------------------Read Voltage and Current--------------------------------------------------------------
void readVoltageCurrent(void)
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
  adc3 = ads.readADC_SingleEnded(vltgmtr);                              // Lee el ADC
  voltage = ads.computeVolts(adc3) * voltageGains[3].calibrationFactor; // Calcula el voltaje

  for (const auto &setting : voltageGains)
  { // Itera sobre las configuraciones de ganancia
    if (voltage >= setting.minVoltage && voltage < setting.maxVoltage)
    {                                                               // Encuentra la configuración de ganancia correcta
      ads.setGain(setting.gain);                                    // Configura la ganancia correcta
      adc3 = ads.readADC_SingleEnded(vltgmtr);                      // Lee el ADC
      voltage = ads.computeVolts(adc3) * setting.calibrationFactor; // Calcula el voltaje
      break;                                                        // Sale del bucle
    }
  }

  // Lectura de corriente
  ads.setGain(GAIN_ONE);                                                // Puede ser mas de 10A, pongo la ganancia en x1 por protección
  adc1 = ads.readADC_SingleEnded(crrsnsr);                              // Lee el ADC
  current = ads.computeVolts(adc1) * currentGains[3].calibrationFactor; // Calcula la corriente

  for (const auto &setting : currentGains)
  { // Itera sobre las configuraciones de ganancia
    if (current >= setting.minVoltage && current < setting.maxVoltage)
    {                                                               // Encuentra la configuración de ganancia correcta
      ads.setGain(setting.gain);                                    // Configura la ganancia correcta
      adc1 = ads.readADC_SingleEnded(crrsnsr);                      // Lee el ADC
      current = ads.computeVolts(adc1) * setting.calibrationFactor; // Calcula la corriente
      break;                                                        // Sale del bucle
    }
  }

  ads.setGain(GAIN_TWOTHIRDS); // Restaurar configuración predeterminada
  #else
  voltage = 100;                                  // Simulo una lectura de tensión de 10V
  if (toggle) {current = setCurrent / 1000 * 0.99;} // Simulación de corriente con 1% menos
  else {current = 0;}
  #endif
}

//-----------------------DAC Control Voltage for Mosfet---------------------------------------
void dacControlVoltage(void)
{
  if (Mode == "CC")
  {
    setCurrent = reading * 1000; // set current is equal to input value in Amps
    setReading = setCurrent;     // show the set current reading being used
    setControlCurrent = setCurrent * Set_Curr_Dsgn_Fact;
    controlVoltage = setControlCurrent;
  }

  if (Mode == "CP")
  {
    setPower = reading * 1000; // in Watts
    setReading = setPower;
    setCurrent = setPower / ActualVoltage;
    setControlCurrent = setCurrent * Set_Curr_Dsgn_Fact;
    controlVoltage = setControlCurrent; //
  }

  if (Mode == "CR")
  {
    setResistance = reading; // in ohms
    setReading = setResistance;
    setCurrent = (ActualVoltage) / setResistance * 1000;
    setControlCurrent = setCurrent * Set_Curr_Dsgn_Fact;
    controlVoltage = setControlCurrent;
  }

  if (Mode == "TC" || Mode == "TL")
  { // Transient Modes
    setControlCurrent = (setCurrent * 1000) * Set_Curr_Dsgn_Fact;
    controlVoltage = setControlCurrent;
  }
}

//--------------------------Set DAC Voltage--------------------------------------------
void dacControl(void)
{
  if (!toggle)
  {
#ifndef WOKWI_SIMULATION
    dac.setVoltage(0, false); // set DAC output voltage to 0 if Load Off selected
#endif
    if (Mode == "BC" && ActualVoltage >= BatteryCutoffVolts && timer_status() == 1)
    {
      timer_stop();
    }
  }
  else
  {
#ifndef WOKWI_SIMULATION
    dac.setVoltage(controlVoltage, false); // set DAC output voltage for Range selected
#endif
    if (Mode == "BC" && ActualVoltage >= BatteryCutoffVolts && timer_status() != 1)
    {
      timer_start();
    }
  }
}

//-----------------------Select Constant Current LCD set up--------------------------------
void Current(void)
{
  Mode = ("CC");
  printLCD(0, 0, "DC LOAD");          // Muestra el titulo del modo
  printLCD(0, 2, "                "); // Borra el 3er. Renlón del LCD
  printLCD(0, 2, "Set I = ");         // Muestra el mensaje
  printLCD(16, 2, "    ");            // Borra cualquier valor anterior
  printLCD(14, 2, "A");               // Muestra el mensaje
  clearLCDLine(3);                    // Borra el 4to. Renlón del LCD
  CP = 9;                             // Pone el cursor en la posición de las unidades de Amperes
}

//----------------------Select Constant Power LCD set up------------------------------------
void Power(void)
{
  Mode = ("CP");
  printLCD(0, 0, "DC LOAD");          // Muestra el titulo del modo
  printLCD(0, 2, "                "); // Borra el 3er. Renglón del LCD
  printLCD(0, 2, "Set W = ");         // Muestra el mensaje
  printLCD(16, 2, "    ");            // Borra cualquier valor anterior
  printLCD(14, 2, "W");               // Muestra el mensaje
  clearLCDLine(3);                    // Borra el 4to. Renlón del LCD
  CP = 10;                            // Pone el cursor en la posición de las unidades de Potecia
}

//----------------------- Select Constant Resistance LCD set up---------------------------------------
void Resistance(void)
{
  Mode = ("CR");
  printLCD(0, 0, "DC LOAD");           // Muestra el titulo del modo
  printLCD(0, 2, "                ");  // Borra el 3er. Renlón del LCD
  printLCD(0, 2, "Set R = ");          // Muestra el mensaje
  printLCD(16, 2, "    ");             // Borra cualquier valor anterior
  printLCD(14, 2, String((char)0xF4)); // Muestra el Símbolo de Ohms
  clearLCDLine(3);                     // Borra el 4to. Renlón del LCD
  CP = 10;                             // Pone el cursor en la posición de las unidades de Resistencia
}

//-------------------------------------Battery Capacity Discharge Routine--------------------------------------
void batteryCapacity(void) {
  if (Mode != "BC") return;  // Solo ejecuta si está en modo Battery Capacity

  setCurrent = reading * 1000;
  setReading = setCurrent;
  setControlCurrent = setCurrent * Set_Curr_Dsgn_Fact;
  controlVoltage = setControlCurrent;

  printLCD(0, 3, timer_getTime()); // Muestra el tiempo transcurrido

  Seconds = timer_getTotalSeconds(); // Obtiene el tiempo total en segundos
  LoadCurrent = (timer_status() == 2) ? BatteryCurrent : ActualCurrent; // Mantiene última corriente si se detiene el timer

  // Calcula capacidad en mAh
  BatteryLife = round((LoadCurrent * 1000) * (Seconds / 3600));

  // Solo actualizar LCD si el valor cambia
  if (BatteryLife > BatteryLifePrevious) {
    String capacityStr = (BatteryLife < 1000) ? String(BatteryLife, 1) : String((int)BatteryLife); 
    capacityStr += "mAh";
    
    while (capacityStr.length() < 7) capacityStr = "0" + capacityStr; // Agrega ceros a la izquierda
    
    printLCD(9, 3, capacityStr);
    BatteryLifePrevious = BatteryLife;
  }

  // Si la batería llega al voltaje de corte, detener descarga
  if (ActualVoltage <= BatteryCutoffVolts) {
      BatteryCurrent = ActualCurrent;
      load_ON_status(false);
      toggle = false;
      timer_stop();
  }

  // Registro de datos para logging
  if (Mode == "BC" && Load == 1 && Seconds != SecondsLog) {
      SecondsLog = Seconds;
      Serial.print(SecondsLog);
      Serial.print(",");
      Serial.println(ActualVoltage);
  }
}

//----------------------- Select Battery Capacity Testing LCD set up---------------------------------------
void BatteryCapacityMode(void)
{
  Mode = ("BC");
  printLCD(0, 0, "BATTERY");          // Muestra el titulo del modo
  printLCD(0, 2, "                "); // Borra el 3er. Renlón del LCD
  printLCD(0, 2, "Set I = ");         // Muestra el mensaje
  printLCD(14, 2, "A");               // La unidad de corriente
  clearLCDLine(3);                    // Borra el 4to. Renlón del LCD
}

//---------------------- Battery Type Selection and Cutoff Setup -----------------------------
void batteryType() {
  exitMode = 0;                           // Resetea EXIT mode
  lcd.noCursor();                         // Apaga el cursor para este menú
  lcd.clear();                            // Borra la pantalla del LCD
  printLCD(0, 0, "Select Battery Type");  // Muestra el título
  printLCD(0, 1, "Stor.: 1=LiPo 2=LiOn");
  printLCD(0, 2, "Disc.: 3=LiPo 4=LiOn");
  printLCD(0, 3, "5=Custom Voltage");

  customKey = getKeyPressed(); // Espera entrada de teclado

  switch (customKey) {
      case '1': BatteryCutoffVolts = LiPoStoragVoltage; BatteryType = "LiPo"; break;
      case '2': BatteryCutoffVolts = LionStoragVoltage; BatteryType = "Lion"; break;
      case '3': BatteryCutoffVolts = LiPoCutOffVoltage; BatteryType = "LiPo"; break;
      case '4': BatteryCutoffVolts = LionCutOffVoltage; BatteryType = "Lion"; break;
      case '5': BatteryType = "Custom"; break;
      case 'M': case '<': exitMode = 1; return; // Salir del modo
      default: batteryType(); return;  // Si la tecla no es válida, repetir selección
  }

  if (BatteryType == "Custom" && exitMode != 1) {
      // Pedir al usuario ingresar un voltaje de corte personalizado
      lcd.clear();
      printLCD(0, 0, "   Enter Battery    ");
      printLCD(0, 1, "   Voltage Cutoff   ");
      printLCD(0, 3, "    (0 to 25V)      ");
      do {
        y = 8; z = 8; r = 2;
        printLCD(z - 1, r, ">");
        inputValue(5);
      } while (x > 25 || x == 0);
      BatteryCutoffVolts = x;
  } else {
      // Si no es Custom, preguntar cantidad de celdas
      lcd.clear();
      printLCD(2, 0, "Battery Selected");
      printLCD(8, 1, BatteryType);
      printLCD(2, 2, "Cells (1-6) ?");
      
      customKey = getKeyPressed();
      if (customKey >= '1' && customKey <= '6') {
          BatteryCutoffVolts *= (customKey - '0');  // Multiplicar por la cantidad de celdas
      }
  }

  // Mostrar selección final
  lcd.clear();
  printLCD(2, 0, "Battery Selected");
  printLCD(8, 1, BatteryType);
  printLCD(2, 2, "Cutoff Voltage");
  printLCD(6, 3, String(BatteryCutoffVolts) + "V");
  delay(3000); lcd.clear();
}

//-------------------------------------User set up for limits-------------------------------------------------
void userSetUp(void)
{
  load_ON_status(false); // Apaga la carga
  lcd.noCursor(); lcd.clear();
  y = 15;
  printLCD(4, 0, "User Set-Up");
  printLCD(0, 1, "Current Limit =");
  printLCD(19, 1, "A");
  r = 1; z = 15;
  inputValue(4);
  CurrentCutOff = min(x, 10);
  saveToEEPROM(ADD_CURRENT_CUT_OFF, CurrentCutOff);
  printLCD(y, r, String(CurrentCutOff));
  
  printLCD(0, 2, "Power Limit  =");
  printLCD(19, 2, "W");
  r = 2; z = 15;
  inputValue(3);
  PowerCutOff = min(x, 300);
  saveToEEPROM(ADD_POWER_CUT_OFF, PowerCutOff);
  printLCD(y, r, String(PowerCutOff));
  
  printLCD(0, 3, "Temp. Limit  =");
  printLCD(18, 3, String((char)0xDF) + "C");
  r = 3; z = 15;
  inputValue(2);
  tempCutOff = min(x, 99);
  saveToEEPROM(ADD_TEMP_CUT_OFF, tempCutOff);
  printLCD(y, r, String(tempCutOff));
  
  setupLimits();
  delay(1000);
  lcd.clear();           // Borra la pantalla del LCD
  Current();             // Ir a modo Corriente Constante (default)
  encoderPosition = 0;   // Resetear el encoder
  customKey = 'M';       // Vuelve a Modos, ver para que, creo que no es necesario
  load_ON_status(false); // Apaga la carga para que muestre el OFF
}

//------------------------Key input used for UserSetUp------------------------
void inputValue(int maxDigits) {
  index = 0;         // Reiniciar índice de entrada
  numbers[0] = '\0'; // Inicializar como cadena vacía
  decimalPoint = ' '; // Restablecer el estado del punto decimal

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
      else if (customKey == 'E') { // Confirmar entrada
          if (index > 0) { // Evitar que se confirme una entrada vacía
              x = atof(numbers); // Convertir a número
              break; // Salir del bucle
          }
      }

      // 🛑 Borra exactamente `maxDigits` espacios antes de escribir la nueva entrada
      String clearStr = "";  
      for (int i = 0; i < maxDigits; i++) clearStr += " "; 
      printLCD(z, r, clearStr);  

      // ✅ Escribe la nueva entrada correctamente
      printLCD(z, r, String(numbers)); 
  }

  // Resetear variables auxiliares después de la entrada
  index = 0;
  decimalPoint = ' ';
}

//-------------------------------------------- Funciones para el Timer ---------------------------------------------------------
int timer_status()
{
  if (timerStarted)
  {
    return 1; // Timer iniciado
  }
  else
  {
    return 2; // Timer detenido
  }
}

void timer_start()
{
  if (!timerStarted)
  {
    startTime = rtc.now();
    timerStarted = true;
  }
}

void timer_stop()
{
  if (timerStarted)
  {
    DateTime now = rtc.now();
    TimeSpan elapsedTime = now - startTime;
    elapsedSeconds += elapsedTime.totalseconds();
    timerStarted = false;
  }
}

void timer_reset()
{
  elapsedSeconds = 0.0;
  timerStarted = false;
}

float timer_getTotalSeconds()
{
  if (timerStarted)
  {
    DateTime now = rtc.now();
    TimeSpan elapsedTime = now - startTime;
    return elapsedSeconds + elapsedTime.totalseconds();
  }
  else
  {
    return elapsedSeconds;
  }
}

String timer_getTime()
{
  int totalSeconds = static_cast<int>(timer_getTotalSeconds());

  int hours = totalSeconds / 3600;
  int minutes = (totalSeconds % 3600) / 60;
  int seconds = (totalSeconds % 3600) % 60;

  String formattedTime = "";
  if (hours < 10)
  {
    formattedTime += "0";
  }
  formattedTime += String(hours) + ":";

  if (minutes < 10)
  {
    formattedTime += "0";
  }
  formattedTime += String(minutes) + ":";

  if (seconds < 10)
  {
    formattedTime += "0";
  }
  formattedTime += String(seconds);

  return formattedTime;
}

//-----------------------------Show limits Stored Data for Current, Power and Temp-----------------------------
void setupLimits(void)
{
  lcd.clear();
  printLCD(1, 0, "Maximum Limits Set"); // Muestra el titulo
  printLCD(0, 1, "Current Limit=");
  printLCD(18, 1, "A");
  CurrentCutOff = loadFromEEPROM(ADD_CURRENT_CUT_OFF); // Lee el valor de la corriente de corte desde la EEPROM
  printLCD(15, 1, String(CurrentCutOff));              // Muestra el valor de la corriente de corte

  printLCD(0, 2, "Power Limit  =");
  printLCD(19, 2, "W");
  PowerCutOff = loadFromEEPROM(ADD_POWER_CUT_OFF); // Lee el valor de la potencia de corte desde la EEPROM
  printLCD(15, 2, String(PowerCutOff));            // Muestra el valor de la potencia de corte

  printLCD(0, 3, "Temp. Limit  =");
  printLCD(17, 3, String((char)0xDF) + "C");
  tempCutOff = loadFromEEPROM(ADD_TEMP_CUT_OFF); // Lee el valor de la temperatura de corte desde la EEPROM
  printLCD(15, 3, String(tempCutOff));           // Muestra el valor de la temperatura de corte
}

//----------------------------------------Transient Mode--------------------------------------------
void transientMode(void)
{

  if (Mode != "TL")
  {

    lcd.noCursor();
    lcd.clear(); // Apaga el cursor y borra la pantalla del LCD

    printLCD(3, 0, "Transient Mode");
    printLCD(0, 1, "Set Low  I=");
    printLCD(19, 1, "A");
    y = 11; z = 11; r = 1;                  // Setea las posiciones de la pantalla del LCD
    inputValue(5);                          // Obtiene el valor ingresado por el usuario
    LowCurrent = min(x, CurrentCutOff);     // Limita la corriente baja al valor de corte de corriente
    printLCD(11, r, String(LowCurrent, 3)); // Muestra el valor de la corriente baja

    printLCD(0, 2, "Set High I=");
    printLCD(19, 2, "A");
    z = 11; r = 2;
    inputValue(5);                           // Obtiene el valor ingresado por el usuario
    HighCurrent = min(x, CurrentCutOff);     // Limita la corriente alta al valor de corte de corriente
    printLCD(11, r, String(HighCurrent, 3)); // Muestra el valor de la corriente alta con tres decimales

    if (Mode == "TC")
    { // Si el modo es Transitorio Continuo
      printLCD(0, 3, "Set Time  = ");
      printLCD(16, 3, "mSec");
      z = 11;
      r = 3;         // Setea las posiciones de la pantalla del LCD
      inputValue(5); // Obtiene el valor ingresado por el usuario

      transientPeriod = x;                      // Guarda el valor del tiempo de transitorio
      printLCD(11, r, String(transientPeriod)); // Muestra el valor de la duración del transitorio
    }
    else
    {
      clearLCDLine(3);
    } // Borra el 4to. Renlón del LCD

    lcd.clear();           // Borra la pantalla del LCD
    toggle = false;        // Flag para apagar la carga
    load_ON_status(false); // Apaga la carga
  }
  else
  {
    transientListSetup();  // Si el modo es Transitorio de Lista, se llama a la función de configuración de la lista
    lcd.clear();           // Borra la pantalla del LCD
    toggle = false;        // Flag para apagar la carga
    load_ON_status(false); // Apaga la carga
  }
}

//----------------------------------------Transient Type Selection--------------------------------------------
void transientType()
{
  toggle = false;
  exitMode = 0;
  lcd.noCursor();
  lcd.clear(); // Apaga el cursor y borra la pantalla del LCD

  printLCD(3, 0, "Transient Mode");
  printLCD(0, 1, "1 = Continuous");
  printLCD(0, 2, "2 = List");

  while (true)
  { // Bucle para garantizar entrada válida
    customKey = getKeyPressed();

    if (customKey == '1' || customKey == '2')
    {                                          // Si la tecla es válida
      Mode = (customKey == '1') ? "TC" : "TL"; // Asigna el modo de transitorio
      break;                                   // Salimos del bucle si la tecla es válida
    }
    else if (customKey == 'M')
    {               // Si se presiona la tecla de Modos
      exitMode = 1; // Salimos del modo
      break;        // También salimos si se cambia de modo
    }
    else if (!strchr("34567890<CSE.", customKey) && customKey != NO_KEY)
    {        // Si la tecla no es válida
      break; // Salimos del bucle si la tecla no es inválida
    }
  }

  lcd.clear();
  if (exitMode == 0)
  {
    transientMode();
  } // Si no se sale del modo, se llama a la función de modo transitorio
}

//----------------------------------------Transient--------------------------------------------
void Transient(void)
{
  if (Mode == "TC" || Mode == "TL")
  {
    lcd.noCursor();            // switch Cursor OFF for this menu
    printLCD(0, 0, "DC LOAD"); // Muestra el titulo del modo

    if (Mode == "TC")
    {
      printLCD(0, 2, "Lo=");                   // Muestra el mensaje
      printLCD(3, 2, String(LowCurrent, 3));   // Muestra el valor de la corriente baja
      printLCD(8, 2, "A");                     // Muestra la unidad
      printLCD(11, 2, "Hi=");                  // Muestra el mensaje
      printLCD(14, 2, String(HighCurrent, 3)); // Muestra el valor de la corriente alta
      printLCD(19, 2, "A");                    // Muestra la unidad
      printLCD(0, 3, "Time = ");               // Muestra el mensaje
      printLCD(7, 3, String(transientPeriod)); // Muestra el valor del tiempo
      printLCD(12, 3, "mSecs");                // Muestra la unidad
    }
    else
    {
      printLCD(0, 3, "  "); // Borra el 4to. Renlón del LCD
    }
  }
}

//-------------------------------------Transcient List Setup-------------------------------------------
void transientListSetup()
{
  lcd.noCursor();
  lcd.clear(); // Apaga el cursor y borra la pantalla
  printLCD(0, 0, "Setup Transient List");
  printLCD(0, 1, "Enter Number in List");
  printLCD(0, 2, "(between 2 to 10)");
  lcd.cursor();
  do { // Bucle para garantizar entrada válida
    y = 0; z = 1; r = 3;
    printLCD(0, r, ">");
    inputValue(2);
  } while (x < 2 || x > 10); // Si el valor no está entre 2 y 10, se vuelve a pedir

  total_instructions = x - 1; // Guarda el número total de instrucciones
  lcd.clear();                // Borra la pantalla del LCD

  for (int i = 0; i <= total_instructions; i++)
  {                                                 // Bucle para obtener los valores de la lista
    printLCD(0, 0, "Set Current " + String(i + 1)); // Pide el valor de corriente en Amperes
    printLCD(7, 1, "A");                            // Muestra la unidad
    do {                                            // Asegura que el valor de corriente no supere 10A
      y = 0; z = 1; r = 1;
      printLCD(0, r, ">");
      inputValue(5);                                // Permitir 5 digitos, ej.: 1.234A
    } while (x > 10 || x <= 0);                     // Si el valor es mayor a 10A o menor/igual a 0, repeti
    transientList[i][0] = x;                        // Guarda el valor de la corriente

    printLCD(0, 2, "Set Time " + String(i + 1));    // Pide el valor de tiempo en milisegundos
    printLCD(7, 3, "mSec");
    y = 0; z = 1; r = 3;
    printLCD(0, r, ">");
    inputValue(5);
    transientList[i][1] = x; // Guarda el valor del tiempo

    lcd.clear(); // Borra la pantalla
  }
  current_instruction = 0; // Resetea el contador de instrucciones porque finalizo la configuración
}

//-------------------------------------Transcient Load Toggel-------------------------------------------
void transientLoadToggle()
{
  if (Mode == "TC")
  { // Si em modo es Transitorio Continuo
    current_time = micros();
    if (last_time == 0)
    {
      last_time = current_time;
    }
    else
    {
      switch (transient_mode_status)
      {
      case (false):
        if ((current_time - last_time) >= (transientPeriod * 1000.0))
        {                                    // Comprueba si el tiempo transcurrido es mayor o igual al tiempo de transitorio
          transientSwitch(LowCurrent, true); // Cambia a la corriente baja
        }
        break;
      case (true):
        if ((current_time - last_time) >= (transientPeriod * 1000.0))
        {                                     // Comprueba si el tiempo transcurrido es mayor o igual al tiempo de transitorio
          transientSwitch(HighCurrent, true); // Cambia a la corriente alta
        }
        break;
      }
    }
  }

  if (Mode == "TL")
  { // Si el modo es Transitorio de Lista
    if (Load == 1)
    { // Solo lo hara si la carga está encendida
      current_time = micros();
      if (last_time == 0)
      {                                                          // Si es la primera vez que se ejecuta
        last_time = current_time;                                // Inicializa el tiempo
        transientPeriod = transientList[current_instruction][1]; // Obtiene el tiempo de transitorio de la lista
        transientSwitch(transientList[current_instruction][0], false);
      }
      if ((current_time - last_time) >= transientList[current_instruction][1] * 1000)
      {                        // Comprueba si el tiempo transcurrido es mayor o igual al tiempo de transitorio de la lista
        current_instruction++; // Incrementa el contador de instrucciones
        if (current_instruction > total_instructions)
        {                          // Si el contador es mayor al total de instrucciones
          current_instruction = 0; // Resetea el contador de instrucciones
        }
        transientPeriod = transientList[current_instruction][1];       // Obtiene el tiempo de transitorio de la lista
        transientSwitch(transientList[current_instruction][0], false); // Cambia a la corriente de la lista
      }
    }
  }
}

//-------------------------------------Transcient Switch-------------------------------------------
void transientSwitch(float current_setting, boolean toggle_status)
{
  if (toggle_status)
  {
    transient_mode_status = !transient_mode_status;
  }
  setCurrent = current_setting;
  last_time = current_time;
}

//------------------------------------- Funciones para el LCD -------------------------------------------
// Función para limpiar una línea específica
void clearLCDLine(int row)
{
  printLCD(0, row, "                    "); // 20 espacios para borrar la línea
}

// Función para imprimir un mensaje en una posición específica
void printLCD(int col, int row, const String &message)
{
  lcd.setCursor(col, row);
  lcd.print(message);
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