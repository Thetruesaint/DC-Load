//#define WOKWI_SIMULATION

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
void LimitsChecks ();
void ActualReading();
void currentDisplayCal();
void powerLevelCutOff();
void temperatureCutOff (void);
void displayEncoderReading (void);
void CursorPosition(void);
void readVoltageCurrent (void);
void dacControlVoltage (void);
void dacControl (void) ;
void batteryCapacity (void);
void tempcheck(void);
void Current(void);
void Power(void);
void Resistance(void);
void BatteryCapacity(void) ;
void batteryType (void);
void batteryTypeSelected (void);
void setBatteryCutOff (void);
void userSetUp (void);
void inputValue (int maxDigits);
int timer_status();
void timer_start();
void timer_stop();
void timer_reset();
float timer_getTotalSeconds();
String timer_getTime();
void setupLimits (void);
void transientMode (void);
void transientType (void);
void Transient (void);
void transientListSetup();
void transientLoadToggle();
void transientSwitch(float current_setting, boolean toggle_status);
void clearLCDLine(int row);
void printLCD(int col, int row, const String &message);
void saveToEEPROM(int address, float value);
float loadFromEEPROM(int address);


// Creamos los objetos que vamos a utilizar
#ifndef WOKWI_SIMULATION
Adafruit_MCP4725 dac;                         // Objeto dac para el MP4725
Adafruit_ADS1115 ads;                         // Objeto ads para el ADS115
#endif
LiquidCrystal_I2C lcd(0x27, 20, 4);           // Objeto LCD 20x4 Address 0x27
RTC_DS1307 rtc;                               // Objeto RTC para el DS1307

//-------------------I/O Pins------------------------------------------------------------

const byte templm35 = A0;                     // Sensado de Temperatura
const byte fansctrl = A2;                     // Endendido de Fans 
const byte ENC_A = 3;                         // Encoder Pin A
const byte ENC_B = 2;                         // Encoder Pin B 
const byte encbtn = 4;                        // Encoder Boton
const byte crrsnsr = 1;                       // Input A1 from ADC
const byte vltgmtr = 3;                       // Input A3 from ADC
const byte LoadOnOff = 15;                    // Input A1 used as a digital pin to set Load ON/OFF

//--------------- Variables para Encoder -------------------------------------------------

unsigned long lastButtonPress = 0;            // Use this to store if the encoder button was pressed or not
volatile float encoderPosition = 0;           // Antes era volatile float
volatile float factor = 0;                    // Factor de escala del Encoder
volatile unsigned long encoderMax = 999000;   // sets maximum Rotary Encoder value allowed CAN BE CHANGED AS REQUIRED (was 50000)

//--------------- Variables de operacion y Modos CC, CR y CP----------------------------------------

int16_t adc0, adc1, adc2, adc3;
unsigned long controlVoltage = 0;             // Voltage de control para el DAC que controlara al MOSFET
float current = 0;                            // Corriente de Carga
float voltage = 0;                            // Voltage de Carga
float ActualVoltage = 0;                      // Lectura de Voltage de Carga
float ActualCurrent = 0;                      // Lectura de Corriente de Carga
float ActualPower = 0;                        // Potencia de Carga
int CP = 8;                                   // Posicion inicial del cursor 
boolean toggle = false;                       // Conmuta la carga On/Off
int Load = 0;                                 // Load On/Off flag de estado de la carga
float reading = 0;                            // Variable para Encoder dividido por 1000
float setCurrent = 1.000;                     // Variable para setear la corriente de carga
float setPower = 20;                          // Variable para setear la potencia de carga
float setResistance = 30;                     // Variable para setear la resistencia de carga
float Set_Curr_Dsgn_Fact = 0.386894318;       // Factor de diseño para el DAC, paso a convertir 5a1 (5V a 1V de Ref.) osea Corriente máxima 10A V1.63 = 0.4095
float setControlCurrent = 0;                  // Variable temporaria para cargar el valor de corriente de control... ver si sirve
int setReading = 0;                           // Variable usada en la funcion dacControlVoltage() pero nada mas... ver si sirve
int CurrentCutOff;                            // Mantendra el valor de la corriente de corte seteado o cargado de la EEPROM
int PowerCutOff;                              // Mantendra el valor de la potencia de corte seteado o cargado de la EEPROM
int tempCutOff;                               // Mantendra el valor de la temperatura de corte seteado o cargado de la EEPROM
float ResistorCutOff = 999;                   // Maximo valor de resistencia para el modo CR
String Mode ="  ";                            // Modo de operación

//----------------------------------------Variables para el Keypad-------------------------------------------

const byte ROWS = 4;                          // Cuatro filas
const byte COLS = 4;                          // Cuatro columnas
//Definicion de las teclas del teclado
char hexaKeys[ROWS][COLS] = {
  {'1','2','3','M'},
  {'4','5','6','C'},
  {'7','8','9','S'},
  {'<','0','.','E'}
};
int functionIndex = 1;                        // Es 1 para que el siguiente Modo sea Power() ya que se inicia en Current()

byte rowPins[ROWS] = {5, 6, 7, 8};            // Pineado de las filas del teclado
byte colPins[COLS] = {9, 10, 11, 12};         // Pineado de las columnas del teclado

Keypad customKeypad = Keypad(makeKeymap(hexaKeys), rowPins, colPins, ROWS, COLS);
char customKey;
char decimalPoint;                            // Variable para el punto decimal en la entrada de teclado
char numbers[10];                             // Variable para la entrada de teclado
byte index = 0;                               // poicion para el caranter de la variable numbers
float x = 0;                                  // Ver para que se usa

//-----------------------------------Variables de coordenadas para ubicar valores en LCD--------------------
int z = 1;                                    // Posición en renglón (En CC,CP y CR dejo lugar para poner caracter ">"), aun no lo puse
int y = 0;                                    // Posición provisoria
int r = 0;                                    // Renglon  

//---------------------------------------Inicializo Variables para Modo Baterias-----------------------------
float Seconds = 0;                            // Variable para los segundio usada en Battery Capacity Mode (BC)
bool timerStarted = false;                    // Variable para saber si el timer esta corriendo
DateTime startTime;                           // Variable para el tiempo de inicio
float elapsedSeconds = 0.0;                   // Variable para los segundos transcurridos
float BatteryLife = 0;                        // Variable para la vida de la batería
float BatteryLifePrevious = 0;                // Variable para la vida de la batería anterior
float SecondsLog = 0;                         // Variable usada para loguear el tiempo en segundos
float BatteryCutoffVolts;                     // Variable usada para indicar a que Voltage se puede descargar la batería
float BatteryCurrent;                         // Variable usada para setear la corriente de descargga de la bateria
float LoadCurrent;                            // Almacena por un momento el ActualCurrent
float MaxBatteryCurrent = 10.0;               // Corriente máxima de descarga de la batería. Ver si no puede definirse como constante
float LiPoCutOffVoltage = 3.5;                // Voltage mínimo de descarga para baterias LiPo
float LionCutOffVoltage = 2.8;                // Voltage mínimo de descarga para baterias Lion
float LiPoStoragVoltage = 3.8;                // Voltage mínimo de almacenamiento para baterias LiPo
float LionStoragVoltage = 3.7;                // Voltage mínimo de almacenamiento para baterias Liom
String BatteryType ="    ";                   // Para definir el Tipo de Batería
byte exitMode = 0;                            // Para salir de la selección de tipo de batería

//---------------------------------------Variables para Control de Temperatura-----------------------------
int temp = 0;                                 // Registra temperatura del disipador donde estan los MOSFET
const int tmpchk_time = 500;                  // Perdíodo de control de temperatura (miliseg.)
unsigned long Last_tmpchk = 0;                // Tiempo desde el ùltimo chequeo de temperatura
const int fan_on_duration = 30000;            // Tiempo en miliseg. para mantener los fans encendidos (30 segundos)
unsigned long fan_on_time = 0;                // Tiempo que lleva encendido el Fan
bool fans_on = false;                         // Flag para saber si los fans estan encendidos

//---------------------------------------Variables para Modo Transient------------------------------------
float LowCurrent = 0;                         // Configuración de corriente baja para el modo transitorio
float HighCurrent = 0;                        // Configuración de corriente alta para el modo transitorio
unsigned long transientPeriod;                // Para almacenar el período de tiempo del pulso en el modo de pulso transitorio
unsigned long current_time;                   // Para almacenar el tiempo actual en microsegundos
unsigned long last_time = 0;                  // Para almacenar el tiempo del último cambio transitorio en microsegundos
boolean transient_mode_status = false;        // Para mantener el estado del modo transitorio (false = corriente baja, true = corriente alta)
float transientList [10][2];                  // Array para almacenar los datos de la lista transitoria
int total_instructions;                       // Utilizado en el modo de Transient List Mode
int current_instruction;                      // Utilizado en el modo de Transient List Mode

//------------------------------ Posiciones reservadas en la EEMPROM --------------------------------------
const int ADD_CURRENT_CUT_OFF = 0x00;
const int ADD_POWER_CUT_OFF = 0x20;
const int ADD_TEMP_CUT_OFF = 0x40;

//---------------------------------------Variables para el Set Up-----------------------------------------
void setup() {

  //-------------------------------------Inicializa perifericos-------------------------------------------
  Serial.begin(9600);                          // Para Debugs y Logs
  lcd.begin(20,4);                             // initialize the lcd, default address 0x27
  rtc.begin();                                 // Inicializa el RTC en teoría en address 0x68
  #ifndef WOKWI_SIMULATION
  ads.begin();                                 // Inicializa el ADC con address 0x48
  ads.setGain(GAIN_TWOTHIRDS);                 // Setea la ganancia del ADC a 2/3x gain +/- 6.144V  1 bit = 0.1875mV
  dac.begin(0x60);                             // Inicializa el DAC con address 0x60
  #endif

  // Ver de agregar Heald Checks antes de inicializar e informar error de detectarse.

  //-------------------------------------Configuraciones iniciales de única vez-----------------------------
  //dac.setVoltage(0,true);                      // reset DAC to zero for no output current set at Switch On, Cambio a "True" para que guarde este valor en la Emprom 
  //rtc.adjust(DateTime(F(__DATE__), F(__TIME__))); // Para ajsutar la hora al momento de compilar el código pero luego se debe comentar para que reloj siga corriendo

  //-------------------------------------Inicializa I/O---------------------------------------------------
  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);
  pinMode(encbtn, INPUT_PULLUP);
  pinMode (LoadOnOff, INPUT_PULLUP);
  pinMode (templm35, INPUT);
  pinMode (fansctrl, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(ENC_A), read_encoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B), read_encoder, CHANGE);
  
  //------------------------------------Pantalla Inicio--------------------------------------------------
  DateTime now = rtc.now();  // Obtiene la fecha y hora actual del RTC
  String date = String(now.day()) + "/" + String(now.month()) + "/" + String(now.year());     // Formatea la fecha
  String time = String(now.hour()) + ":" + String(now.minute()) + ":" + String(now.second()); // Formatea la hora
  lcd.clear();
  lcd.backlight();              // Turn on the LCD screen backlight                    
  printLCD(1, 0, "DC Electronic Load");
  lcd.setCursor(1,1);
  lcd.print(date +" - "+ time);
  printLCD(1, 2, "Guy Nardin");
  printLCD(1, 3, "v1.64");              // Modos CC, CP y CR funcionando con Teclado. Ajustes para el Set con Load ON/Off
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
                                        // 1.64 Mejoras varias en el código, Guardado en EEPROM, temp de Fans on a 40°C.                                
  delay(2000);
  lcd.clear();
  //---------------------------------------Chequea y Muestra los límites configurados----------------------
  #ifndef WOKWI_SIMULATION
  CurrentCutOff = loadFromEEPROM(ADD_CURRENT_CUT_OFF);   // Carga CurrentCutOff desde la EEPROM
  PowerCutOff = loadFromEEPROM(ADD_POWER_CUT_OFF);       // Carga PowerCutOff desde la EEPROM
  tempCutOff = loadFromEEPROM(ADD_TEMP_CUT_OFF);         // Carga tempCutOff desde la EEPROM
  if (CurrentCutOff < 1 || CurrentCutOff > 10 ||         // Chequea que los valores de los límites estén en el rango correcto
    PowerCutOff < 1 || PowerCutOff > 300 || 
    tempCutOff < 30 || tempCutOff > 99) {
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
  load_ON_status(false);               //indicate that LOAD is off at start up
  Current();
  }

//------------------------------------- Bucle Principal-------------------------------------------------
void loop() {
  readKeypadInput();                                     //read Keypad entry
  Load_Switch_Check();                                        //Load on/off
  Transient();                                           //test for Transient Mode
  printLCD(18, 3, Mode);
  if(Mode != "TC" && Mode != "TL"){                      //if NOT transient mode then Normal Operation
    reading = encoderPosition/1000;                        //read input from rotary encoder
    LimitsChecks();                                        //Chequea Limites de todos los Modos y resetea a máximo de ser necesario
    displayEncoderReading();                               //display rotary encoder input reading on LCD
    CursorPosition();                                      //check and change the cursor position if cursor button pressed
    }
    else{transientLoadToggle();}                           //Start Transient Mode
  
  readVoltageCurrent();                                  //routine for ADC's to read actual Voltage and Current
  powerLevelCutOff();                                    //Check if Power Limit has been exceeded
  temperatureCutOff();                                   //check if Maximum Temperature is exceeded
  ActualReading();                                       //Display actual Voltage, Current readings and Actual Wattage
  dacControl();
  dacControlVoltage();                                   //sets the drive voltage to control the MOSFET
  batteryCapacity();                                     //test if Battery Capacity (BC) mode is selected - if so action
  tempcheck();                                           //Chequea Temperatura y acciona Coolers en consecuencia
}

//------------------------------Load ON Status--------------------------------------
void load_ON_status(boolean loadonoff) {
    if(!loadonoff) {
      printLCD(8, 0, "OFF");
      #ifndef WOKWI_SIMULATION
      dac.setVoltage(0,false);                          //Ensures Load is OFF - sets DAC output voltage to 0
      #endif
      Load = 0;                                         //Setea Flag para el Log
    }
    else if (loadonoff) {
      printLCD(8, 0, "ON ");
      lcd.setCursor(0,3);
      Load = 1;                                         //Setea Flag para el Log
    }
  }

//-----------------------------Encoder Decoder---------------------------------------
void read_encoder() {
  // Encoder interrupt routine for both pins. Updates encoderPosition
  // if they are valid and have rotated a full indent
 
  static uint8_t old_AB = 3;  // Lookup table index
  static int8_t encval = 0;   // Encoder value  
  static const int8_t enc_states[]  = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0}; // Lookup table

  old_AB <<=2;  // Remember previous state

  if (digitalRead(ENC_A)) old_AB |= 0x02; // Add current state of pin A
  if (digitalRead(ENC_B)) old_AB |= 0x01; // Add current state of pin B
  
  encval += enc_states[( old_AB & 0x0f )];

  // Update encoderPosition if encoder has rotated a full indent, that is at least 4 steps
  if( encval > 3 ) {        // Four steps forward
    encoderPosition = encoderPosition + factor;              // Update encoderPosition Up
    encval = 0;
  }
  else if( encval < -3 ) {        // Four steps backward
    encoderPosition = encoderPosition - factor;              // Update encoderPosition Down
    encval = 0;
  }
  encoderPosition = min(encoderMax, max(0, encoderPosition));
  } 

//-----------------------------Read Keypad Input---------------------------------------
void readKeypadInput (void) {
  customKey = customKeypad.getKey();
   
  // if (customKey != NO_KEY){                              //only used for testing keypad
  // Serial.print("customKey = ");                          //only used for testing keypad
  // Serial.println(customKey);                             //only used for testing keypad
  // }                                                      //only used for testing keypad
  if (customKey != NO_KEY){            
    if(customKey == 'M'){                                   // Cambio de Modo
      toggle = false;                                       // Apago la carga si estaba activa.
      load_ON_status(false);                                //Si hay cambio de Modo, me aseguro de que se desconecte la carga.
      if (functionIndex == 0) {Current();}
      else if (functionIndex == 1) {Power();}
      else if (functionIndex == 2) {Resistance();}
      else if (functionIndex == 3) {
        batteryType();
        if (exitMode == 0){                           // Si se selecciona un tipo de baterìa, continua con la rutina                                   
          printLCD(16, 2, BatteryType);               // Muestra el tipo de batería 
          load_ON_status(false);
          timer_reset();                              //reset timer
          BatteryLifePrevious = 0;
          CP = 9;                                     //set cursor position
          BatteryCapacity();                          //go to Battery Capacity Routine
         }
        else {
          transientType();
          if (exitMode == 1){
            Current();
            functionIndex = 0;                        // Para que continue con la funcion siguiente, que será Power ()
            }
        }
      }  
      functionIndex = (functionIndex + 1) % 4;                // Incrementar el índice de función y asegurarse de que esté en el rango correcto
      encoderPosition = 0;                                    //reset encoder reading to zero
      index = 0;
      z = 1;                                                  //sets column position for LCD displayed character
      decimalPoint = (' ');                                   //clear decimal point test character reset
    }
            
    if(customKey == 'C'){                                   //check if CNF button pressed
      toggle = false;                                         //switch Load OFF
      userSetUp();
      encoderPosition = 0;                                    //reset encoder reading to zero
      index = 0;
      z = 1;                                                  //sets column position for LCD displayed character
      decimalPoint = (' ');                                   //clear decimal point text character reset
    }
          
    if(customKey == 'S'){                                     // Uso futuro para el boton "Shift"
    }

    if (Mode != "BC"){
      if(customKey >= '0' && customKey <= '9' && index < 6){  // check for keypad number input, no mas de 5 digitos, no es necesario
        numbers[index++] = customKey;
        numbers[index] = '\0';
        printLCD(z, 3, String(customKey));                    // Muestra el número ingresado 
        z++;
      }
    
      if(customKey == '.'){                                   // check if decimal button key pressed
        if (decimalPoint != ('*')){                           // test if decimal point entered twice - if so skip 
          numbers[index++] = '.';
          numbers[index] = '\0';
          printLCD(z, 3, ".");                                // Muestra el punto decimal 
          z++;
          decimalPoint = ('*');                               // used to indicate decimal point has been input
          }
        }

      if(customKey == 'E') {                                  // Tecla "Enter" carga el valor en el Set
        x = atof(numbers);     
        reading = x;
        encoderPosition = reading*1000;
        index = 0;
        numbers[index] = '\0';
        z = 1;                                                // sets column position for LCD displayed character
        printLCD(0, 3, "        ");                           // Borra el valor ingresado en el 4to. Renglón del LCD
        decimalPoint = (' ');                                 // clear decimal point test character reset
      }

      if(customKey == '<'){                                   // clear entry
      index = 0;
      z = 1;
      printLCD(0, 3, "        ");                             // Borra el valor ingresado en el 4to. Renglón del LCD
      numbers[index] = '\0';                              
      decimalPoint = (' ');                                 //clear decimal point test character reset
      } 
    }
  }
}

//-----------------------------Toggle Current Load ON or OFF------------------------------
void Load_Switch_Check(void) {
  if (digitalRead(LoadOnOff) == LOW) {
    delay(200);                                          //simple key bounce delay 
    if(toggle) {
      load_ON_status(false);
      setCurrent = 0;                                    //reset setCurrent to zero
      toggle = !toggle;
      }
    else {
      load_ON_status(true);
      clearLCDLine(3);                                // Borra el 4to. Renlón del LCD
      toggle = !toggle;
      }
  }
}

//-----------------------------Limit Maximum Current Setting-------------------------------
void LimitsChecks (void) {


  if (Mode == "CC" && reading > CurrentCutOff){          //Limit maximum Current Setting
      reading = CurrentCutOff;                           // Sino lo asigno a la variable primero, aparecen Bugs en la variable encoderPosition
      encoderPosition = reading * 1000;                  //keep encoder position value at maximum Current Limit
      clearLCDLine(3);                                // Borra el 4to. Renlón del LCD
  }

  if (Mode == "CP" && reading > PowerCutOff) {           //Limit maximum Current Setting
      reading = PowerCutOff;                             // Sino lo asigno a la variable primero, aparecen Bugs en la variable encoderPosition
      encoderPosition = reading * 1000;                  //keep encoder position value at maximum Current Limit
      clearLCDLine(3);                                // Borra el 4to. Renlón del LCD 
  }

  if (Mode == "CR" && reading > ResistorCutOff ) {       //Limit maximum Current Setting
      reading = ResistorCutOff;                          // Sino lo asigno a la variable primero, aparecen Bugs en la variable encoderPosition
      encoderPosition = reading * 1000;                  //keep encoder position value at maximum Current Limit
      clearLCDLine(3);                                // Borra el 4to. Renlón del LCD
  }

  if (Mode == "BC" && reading > MaxBatteryCurrent){
      reading = MaxBatteryCurrent;
      encoderPosition = (MaxBatteryCurrent*1000);            //keep encoder position value at "MaxBatteryCurrent"
  }
  }

//--------------------Calculate Actual Voltage and Current and display on LCD-------------------
void ActualReading(void) {

  ActualCurrent = current;
  currentDisplayCal();                           //LCD display current calibration correction

  ActualVoltage = voltage;                         //calculate load voltage upto 30v 
  ActualPower = ActualVoltage*ActualCurrent;

  if (ActualPower <=0){
    ActualPower = 0;
  }

  if (ActualVoltage <=0.0){                              //added to prevent negative readings on LCD due to error
    ActualVoltage = 0.0;
  }
  if (ActualCurrent <= 0.0){                             //added to prevent negative readings on LCD due to error
    ActualCurrent = 0.0;
  }
 
 lcd.setCursor(0,1);
    
  if ( ActualCurrent < 10.0 ) {
    lcd.print(ActualCurrent,3);
    }
    else {
        lcd.print(ActualCurrent,2);
    }
    
    lcd.print("A");
    lcd.print(" ");
    
    if (ActualVoltage < 10.0) {
        lcd.print(ActualVoltage, 3);
    } else {
        lcd.print(ActualVoltage, 2);
    }    

    lcd.print("V");
    lcd.print(" ");
     
    if (ActualPower < 100 ) {
        lcd.print(ActualPower,2);
    } else {
        lcd.print(ActualPower,1);
    }
    lcd.print("W");
    lcd.print(" ");
  }

//-----------------------------Current Read Calibration for LCD Display -----------------------
void currentDisplayCal (void) {                   // Ojo que esto enmascara lecturas de corriente de Offset de los Op. Amp. solo cuando el LOAD esta en ON, se ve.

  if(ActualCurrent <= 0){
    ActualCurrent = 0;}
    else if(Load == 0){       
    ActualCurrent = 0;}
  }

//----------------------Power Level Cutoff Routine-------------------------------------------
void powerLevelCutOff (void) {
  if (ActualPower  > PowerCutOff){                //Check if Power Limit has been exceed
  reading = 0;
  encoderPosition = 0; 
  clearLCDLine(3);                                // Borra el 4to. Renlón del LCD
  printLCD(0, 3, "Exceeded Power");               // Muestra el mensaje 
  load_ON_status(false);
  toggle = false;                                 // Switch Load Off
  delay(3000);                                    // Mostramos el mensaje por 3 segundos.
  clearLCDLine(3);                                // Borra el 4to. Renlón del LCD
  }
}

//------------------------------------------High Temperature Cut-Off--------------------------------------------------------------
void temperatureCutOff (void) {
  if (temp >= tempCutOff){                        // Si se alcanza la temperatura maxima, setea la carga a Cero
  reading = 0;
  encoderPosition = 0; 
  clearLCDLine(3);                                // Borra el 4to. Renlón del LCD
  printLCD(0, 3, "Over Temperature");             // Muestra el mensaje 
  load_ON_status(false);                          // Apaga la carga
  toggle = false;                                 // Switch Load Off
  delay(3000);                                    // Mostramos el mensaje por 3 segundos.
  clearLCDLine(3);                                // Borra el 4to. Renlón del LCD
  }
}

//----------------------Display Rotary Encoder Input Reading on LCD---------------------------
void displayEncoderReading (void) {

    lcd.setCursor(8,2);                                      //start position of setting entry
    
    if ( ( Mode == "CP" || Mode == "CR" ) && reading < 100 ) {
        lcd.print("0");
    }
    
    if (reading < 10) {                                      //add a leading zero to display if reading less than 10
        lcd.print("0"); 
    }

    if ( Mode == "CP" || Mode == "CR" ) {
        lcd.print (reading, 2);                              //show input reading from Rotary Encoder on LCD
    } else {
        lcd.print (reading, 3);
    }
    lcd.setCursor (CP, 2);                                   //sets cursor position
    lcd.cursor();                                            //show cursor on LCD
  }

//--------------------------Cursor Position-------------------------------------------------------
void CursorPosition(void) {

    // Defaults for two digits before decimal and 3 after decimal point
    int unitPosition = 9;

    //Power and Resistance modes can be 3 digit before decimal but only 2 decimals
    if ( Mode == "CP" || Mode == "CR" ) {
        unitPosition = 10;        
    }

    if (digitalRead(encbtn) == LOW) {
    
        delay(100);                                          //simple key bounce delay  
        CP = CP + 1;
        if (CP == unitPosition + 1 ) {
            CP = CP + 1;
        }
    }
    
    if (CP > 13)  { CP = unitPosition; }                     //No point in turning tens and hundreds
    if (CP == unitPosition +4 ) { factor = 1; }
    if (CP == unitPosition +3 ) { factor = 10; }
    if (CP == unitPosition +2 ) { factor = 100; }
    if (CP == unitPosition )    { factor = 1000; }
  }

//---------------------------------------------Read Voltage and Current--------------------------------------------------------------
void readVoltageCurrent (void) {
       
                                                                    //static float multiplier = 0.1875F; /* ADS1115  @ +/- 6.144V gain (16-bit results) */
                                                                    //ads.setGain(GAIN_TWOTHIRDS);  // 2/3x gain +/- 6.144V  1 bit = 0.1875mV
                                                                    //ads.setGain(GAIN_ONE);        // 1x gain   +/- 4.096V  1 bit = 0.125mV
                                                                    //ads.setGain(GAIN_FOUR);       // 4x gain   +/- 1.024V  1 bit = 0.03125mV
                                                                    //ads.setGain(GAIN_EIGHT);      // 8x gain   +/- 0.512V  1 bit = 0.015625mV
                                                                    //ads.setGain(GAIN_SIXTEEN);    // 16x gain  +/- 0.256V  1 bit = 0.0078125mV
  #ifndef WOKWI_SIMULATION
  ads.setGain(GAIN_ONE);        // 1x gain   +/- 4.096V  1 bit = 0.125mV
  adc3 = ads.readADC_SingleEnded(vltgmtr);
  voltage = ads.computeVolts(adc3)*50.5589;                         // Por 50 por el divisor resistivo y ampl. dif. para sensado remoto de 50 a 1 (Max. 200V). Calibración promedio

  if (0 <= voltage && voltage < 12) {                             // Si es entre 0 y 12V, pongo la ganancia en x16 y vuelvo a leer para mejorar la presición.
    ads.setGain(GAIN_SIXTEEN);    // 16x gain  +/- 0.256V  1 bit = 0.0078125mV    
    adc3 = ads.readADC_SingleEnded(vltgmtr);
    voltage = ads.computeVolts(adc3)*50.8346;                     // Calibración para PGA de 16x
    }
  else if (12 <= voltage && voltage < 25){                        // Si es entre 12 y 25V, pongo la ganancia en x8 y vuelvo a leer para mejorar la presición.
    ads.setGain(GAIN_EIGHT);      // 8x gain   +/- 0.512V  1 bit = 0.015625mV
    adc3 = ads.readADC_SingleEnded(vltgmtr);
    voltage = ads.computeVolts(adc3)*49.6135;                     // Calibración para PGA de 8x
    }

  else if (25 <= voltage && voltage < 50){                        // Si es entre 12 y 50V, pongo la ganancia en x8 y vuelvo a leer para mejorar la presición.
    ads.setGain(GAIN_FOUR);       // 4x gain   +/- 1.024V  1 bit = 0.03125mV
    adc3 = ads.readADC_SingleEnded(vltgmtr);
    voltage = ads.computeVolts(adc3)*49.6857;                     // Calibración para PGA de 4x
    }

  ads.setGain(GAIN_ONE);                                            // 1x gain   +/- 4.096V  1 bit = 0.125mV
  adc1 = ads.readADC_SingleEnded(crrsnsr);                          // Puede ser mas de 10A, pongo la ganancia en x1 por protección
  current = ads.computeVolts(adc1)*9.6774;                          // Calibración

  if (0.000 <= current && current < 1.900) {                      // x16 y vuelvo a leer para mejorar la presición.
    ads.setGain(GAIN_SIXTEEN);                                    // 16x gain  +/- 0.256V  1 bit = 0.0078125mV    
    adc1 = ads.readADC_SingleEnded(crrsnsr);
    current = ads.computeVolts(adc1)*10.000;                      // Calibración para PGA de 16x
    }
  else if (1.900 <= current && current < 4.900){                  // x8 y vuelvo a leer para mejorar la presición.
    ads.setGain(GAIN_EIGHT);                                      // 8x gain   +/- 0.512V  1 bit = 0.015625mV
    adc1 = ads.readADC_SingleEnded(crrsnsr);  
    current = ads.computeVolts(adc1)*9.9941;                      // Calibración para PGA de 8x
    }
  else if (4.900 <= current && current < 9.800){                  // x4 y vuelvo a leer para mejorar la presición.
        ads.setGain(GAIN_FOUR);                                       // 4x gain   +/- 1.024V  1 bit = 0.03125mV
        adc1 = ads.readADC_SingleEnded(crrsnsr);
        current = ads.computeVolts(adc1)*9.7621;                      // Calibración para PGA de 4x
    }
  ads.setGain(GAIN_TWOTHIRDS);                                      // 2/3x gain +/- 6.144V  1 bit = 3mV      0.1875mV
  #else
  voltage = 10.01;                                                 // Simulo una lectura de tensión de 10V
  current = setCurrent / 1000 * 0.99;                              // Simulo una lectura de corriente con un 1% menos de la seteada
  #endif
  }  

//-----------------------DAC Control Voltage for Mosfet---------------------------------------
void dacControlVoltage (void) {
  if (Mode == "CC"){
  setCurrent = reading*1000;                                //set current is equal to input value in Amps
  setReading = setCurrent;                                  //show the set current reading being used
  setControlCurrent = setCurrent * Set_Curr_Dsgn_Fact;
  controlVoltage = setControlCurrent; 
  }

  if (Mode == "CP"){
  setPower = reading*1000;                                  //in Watts
  setReading = setPower;
  setCurrent = setPower/ActualVoltage;
  setControlCurrent = setCurrent * Set_Curr_Dsgn_Fact;
  controlVoltage = setControlCurrent;                       //
  }

  if (Mode == "CR"){
  setResistance = reading;                                  //in ohms
  setReading = setResistance;
  setCurrent = (ActualVoltage)/setResistance*1000;
  setControlCurrent = setCurrent * Set_Curr_Dsgn_Fact;
  controlVoltage = setControlCurrent; 
  }

  if (Mode == "TC" || Mode == "TL"){                            //Transient Modes
  setControlCurrent = (setCurrent * 1000) * Set_Curr_Dsgn_Fact;
  controlVoltage = setControlCurrent; 
  }
  }

//--------------------------Set DAC Voltage--------------------------------------------
void dacControl (void) {
  if (!toggle){
    #ifndef WOKWI_SIMULATION
    dac.setVoltage(0,false);                                 //set DAC output voltage to 0 if Load Off selected
    #endif
    if(Mode == "BC" && ActualVoltage >= BatteryCutoffVolts && timer_status() == 1){
    timer_stop();
    }
  
  }else{
    #ifndef WOKWI_SIMULATION
    dac.setVoltage(controlVoltage,false);                   //set DAC output voltage for Range selected
    #endif
    if(Mode == "BC" && ActualVoltage >= BatteryCutoffVolts && timer_status() != 1){
    timer_start();
    }
  }
  }

//-------------------------------------Battery Capacity Discharge Routine----------------------------------------------------
void batteryCapacity (void) {
  if (Mode == "BC"){
    
    setCurrent = reading*1000;                             //set current is equal to input value in Amps
    setReading = setCurrent;                               //show the set current reading being used
    setControlCurrent = setCurrent * Set_Curr_Dsgn_Fact;
    controlVoltage = setControlCurrent;

    printLCD(0, 3, timer_getTime());                      // Muestra el tiempo 

    Seconds = timer_getTotalSeconds();                     //get totals seconds
  
    LoadCurrent = ActualCurrent;                           //if timer still running use present Actual Current reading
   
    if (timer_status() == 2){                              //if timer is halted then use last Actual Current reading before timer stopped
      LoadCurrent = BatteryCurrent;
      }
 
    BatteryLife = (LoadCurrent*1000)*(Seconds/3600);       //calculate battery capacity in mAh
    lcd.setCursor(9,3);
    BatteryLife = round(BatteryLife);
    
    if(BatteryLife >= BatteryLifePrevious){                //only update LCD (mAh) if BatteryLife has increased
  
      if (BatteryLife < 10) {                              //add a 3 leading zero to display if reading less than 10
      lcd.print("000");
      }

      if (BatteryLife >= 10 && BatteryLife <100){          //add a 2 leading zero to display
      lcd.print("00");  
      }

      if (BatteryLife >= 100 && BatteryLife <1000){        //add a 1 leading zero to display
      lcd.print("0"); 
      }
  
    lcd.print(BatteryLife,0);                               //No actualiza si hubo un CuffOff por exceder algùn lìmite
    printLCD(13, 3, "mAh");                                 // Muestra el la unidad de consumo de corriente 
    BatteryLifePrevious = BatteryLife;                      //update displayed battery capacity on LCD
    } 
  }


  if (Mode == "BC" && ActualVoltage <= BatteryCutoffVolts){ //stops clock if battery reached cutoff level and switch load off

  BatteryCurrent = ActualCurrent;
  load_ON_status(false);                                       
  toggle = false;                                           //Load is toggled OFF
  timer_stop();
  }
      if (Mode == "BC" && Load == 1){                       //Routine used for data logging in Battery Capacity Mode
          if (Seconds != SecondsLog){                       //only send serial data if time has changed
            SecondsLog = Seconds;
            Serial.print (SecondsLog);                      //sends serial data of time in seconds
            Serial.print (",");                             //sends a comma as delimiter for logged data
            Serial.println (ActualVoltage);                 //sends serial data of Voltage reading         
              }
          }
  
  }

//--------------------------------------------------Temperature Check----------------------------------------------------------
void tempcheck(void) {
  unsigned long hldtmp_time = millis();
  
  if ((hldtmp_time - Last_tmpchk) >= tmpchk_time) {
    temp = analogRead(templm35);                            // Tomar temperatura
    #ifndef WOKWI_SIMULATION
    temp = temp * 0.48828125;                               // Convertir a Celsius
    #else
    temp = temp * 0.09765625;                               // Hasta 100°C con el pote de 0 a 5V que simula sensosr de temperatura
    #endif
    Last_tmpchk = hldtmp_time;

    if (temp >= 40) {                                       // Controlar el encendido de los fans y el temporizador
      digitalWrite(fansctrl, HIGH);                         // Encender los fans si la temperatura es mayor o igual a 40°C
      fans_on = true;
      fan_on_time = hldtmp_time;
    } else if (fans_on && (hldtmp_time - fan_on_time) >= fan_on_duration) {
        digitalWrite(fansctrl, LOW);                        // Apagar los fans después del tiempo especificado si estaban encendidos
        fans_on = false;
      }
    printLCD(16, 0, String(temp) + String((char)0xDF) + "C");  // Muestra la temperatura con el símbolo de grados Celsius
  }
}

//-----------------------Select Constant Current LCD set up--------------------------------
void Current(void) {
  Mode = ("CC");
  printLCD(0, 0, "DC LOAD");              // Muestra el titulo del modo   
  printLCD(0, 2, "                ");     // Borra el 3er. Renlón del LCD
  printLCD(0, 2, "Set I = ");             // Muestra el mensaje 
  printLCD(16, 2, "    ");                // Borra cualquier valor anterior
  printLCD(14, 2, "A");                   // Muestra el mensaje 
  clearLCDLine(3);                        // Borra el 4to. Renlón del LCD
  CP = 9;                                 // Pone el cursor en la posición de las unidades de Amperes
  }

//----------------------Select Constant Power LCD set up------------------------------------
void Power(void) {
  Mode = ("CP");
  printLCD(0, 0, "DC LOAD");              // Muestra el titulo del modo 
  printLCD(0, 2, "                ");     // Borra el 3er. Renglón del LCD
  printLCD(0,2, "Set W = ");              // Muestra el mensaje 
  printLCD(16,2, "    ");                 // Borra cualquier valor anterior
  printLCD(14,2, "W");                    // Muestra el mensaje 
  clearLCDLine(3);                        // Borra el 4to. Renlón del LCD
  CP = 10;                                // Pone el cursor en la posición de las unidades de Potecia
  }

//----------------------- Select Constant Resistance LCD set up---------------------------------------
void Resistance(void) {
  Mode = ("CR");
  printLCD(0, 0, "DC LOAD");              // Muestra el titulo del modo   
  printLCD(0, 2, "                ");     // Borra el 3er. Renlón del LCD
  printLCD(0,2, "Set R = ");              // Muestra el mensaje 
  printLCD(16,2, "    ");                 // Borra cualquier valor anterior
  printLCD(14,2, String((char)0xF4));     // Muestra el Símbolo de Ohms
  clearLCDLine(3);                        // Borra el 4to. Renlón del LCD
  CP = 10;                                // Pone el cursor en la posición de las unidades de Resistencia
  }

//----------------------- Select Battery Capacity Testing LCD set up---------------------------------------
void BatteryCapacity(void) {
  Mode = ("BC");
  printLCD(0, 0, "BATTERY");              // Muestra el titulo del modo 
  printLCD(0, 2, "                ");     // Borra el 3er. Renlón del LCD
  printLCD(0,2, "Set I = ");              // Muestra el mensaje 
  printLCD(14,2, "A");                    // La unidad de corriente 
  clearLCDLine(3);                        // Borra el 4to. Renlón del LCD
  }

//----------------------Battery Action Selection Routine------------------------------------------------
void batteryType (void) {
  exitMode = 0;                           // Resetea EXIT mode
  lcd.noCursor();                         // Apaga el cursor para este menu               
  lcd.clear();                            // Borra la pantalla del LCD
  printLCD(0, 0, "Select:             "); // Muestra Menu de selección 
  printLCD(0, 1, "Stor.: 1=LiPo 2=LiOn"); 
  printLCD(0, 2, "Disc.: 3=LiPo 4=LiOn");
  printLCD(0, 3, "5=Set Voltage");

  customKey = customKeypad.waitForKey();  // Espera entrada de teclado

  switch (customKey) {
    case '1': case '3': BatteryCutoffVolts = LiPoStoragVoltage; BatteryType = "LiPo"; break;
    case '2': case '4': BatteryCutoffVolts = LionStoragVoltage; BatteryType = "Lion"; break;
    case '5': BatteryType = "Custom"; break;
    case 'M': case '<': exitMode = 1; return;           // Salir del modo y de la función
    default: batteryType(); return;                     // Si la tecla no es válida, se vuelve a llamar a sí misma para repetir la selección
  }

  if(BatteryType == "Custom" && exitMode != 1){         // Si se selecciona Custom, se pide el voltaje de corte
  setBatteryCutOff();
    }
  
  if (BatteryType != "Custom" && exitMode != 1){
    lcd.clear();
    printLCD(2, 0, "Battery Selected");                 // Avisa que se seleccionó la batería
    printLCD(8, 1, BatteryType);                        // Indica el tipo de batería seleccionado
    printLCD(2, 2, "Discharge Cutoff");                 // Indica que se seleccionará el voltaje de corte

    customKey = customKeypad.waitForKey();              // Espera entrada de teclado
    
    if (customKey >= '1' && customKey <= '6') {         // Si la tecla es válida, multiplica el valor por el voltaje de corte
      BatteryCutoffVolts *= (customKey - '0');          // Convierte el char en int y multiplica
    } else {
      batteryTypeSelected();                            // Si la tecla no es válida, llama a la función de selección
    }
  }

  batteryTypeSelected();                                // Muestra la batería seleccionada y las opciones de corte
  lcd.clear();                                          // Borra la pantalla del LCD
  }

//--------------------------Battery Selected Information--------------------------------------------
void batteryTypeSelected (void) {
  if (exitMode !=1){                                      // si no se sale del modo, muestra la información de la batería seleccionada
  lcd.clear();                                            // Borra la pantalla del LCD
  printLCD(2, 0, "Battery Selected");                     // Muestra la batería seleccionada
  printLCD(8, 1, BatteryType);                            // Muestra el tipo de batería seleccionado
  printLCD(2, 2, "Discharge Cutoff");                     // Muestra el voltaje de corte
  printLCD(6, 3, String(BatteryCutoffVolts) + " Volts");  // Muestra el voltaje de corte
  delay(3000);
  }
}

//--------------------------Set Battery Cut-Off Voltage--------------------------------------------
void setBatteryCutOff (void) {

  lcd.clear();
  printLCD(3, 0, "Enter Battery");    // Solicitud para ingresar el voltage de corte
  printLCD(2, 1, "Cut-Off Voltage");
  printLCD(5, 3, "(0 to 25V)");
  y = 8; z = 8; r = 2;                // Setea las posiciones de la pantalla del LCD
  printLCD(z-1, r, ">");              // Muestra el cursor en la posición inicial
  inputValue(4);
    while (x > 25 || x == 0) {
      y = 8; z = 8; r = 2;
      index = 0;
      printLCD(z, r, "     ");        // Borra el valor ingresado en el 4to. Renglón del LCD
      inputValue(4);
    }
  BatteryCutoffVolts = x;
  lcd.clear();                        // Borra la pantalla del LCD
  }

//-------------------------------------User set up for limits-------------------------------------------------
void userSetUp (void) {
  load_ON_status(false);                    // Apaga la carga
  y = 14;
  z = 14;
  
  lcd.noCursor();                           // Apaga el cursor para este menu               
  lcd.clear();                              // Borra la pantalla
  printLCD(4, 0, "User Set-Up");            // Muestra el titulo del modo
  printLCD(0, 1, "Current Limit =");        // Muestra el mensaje
  printLCD(19, 1, "A");                     // Muestra la unidad 
  r = 1;
  inputValue(2);
  CurrentCutOff = x;
  CurrentCutOff = min(CurrentCutOff, 10);   // Asegura que el valor no sea mayor a 10A
  saveToEEPROM(ADD_CURRENT_CUT_OFF, CurrentCutOff);  // Guarda el valor CurrentCutOff en la EEPROM
  printLCD(14, r, String(CurrentCutOff));   // Muestra de corriente de corte 
  z = 14;
  printLCD(0, 2, "Power Limit  =");         // Muestra el mensaje
  printLCD(19, 2, "W");                     // Muestra la unidad 
  r = 2;
  inputValue(3);
  PowerCutOff = x;
  PowerCutOff = min(PowerCutOff, 300);      // Asegura que el valor no sea mayor a 300W
  saveToEEPROM(ADD_POWER_CUT_OFF, PowerCutOff); // Guarda el valor PowerCutOff en la EEPROM
  printLCD(14, r, String (PowerCutOff));    // Muestra de potencia de corte
  z = 14;

  printLCD(0, 3, "Temp. Limit  =");         // Muestra la temperatura máxima
  printLCD(18, 3, String ((char)0xDF)+"C"); // Muestra el símbolo de grados Celsius
  r = 3;
  inputValue(2);
  tempCutOff = x;
  tempCutOff = min(tempCutOff, 99);         // Asegura que el valor no sea mayor a 99°C
  saveToEEPROM(ADD_TEMP_CUT_OFF, tempCutOff); // Guarda el valor tempCutOff en la EEPROM
  printLCD(14, r, String (tempCutOff));     // Muestra la temperatura de corte 
  setupLimits();                            // Mostrar los limites configurados
  delay(1000);

  lcd.clear();                              // Borra la pantalla del LCD
  Current();                                // Ir a modo Corriente Constante (default)
  encoderPosition = 0;                      // Resetear el encoder
  customKey = 'M';                          // Vuelve a Modos, ver para que, creo que no es necesario
  load_ON_status(false);                    // Apaga la carga para que muestre el OFF
}

//------------------------Key input used for UserSetUp------------------------
void inputValue (int maxDigits) {

  while (customKey != 'E' && index <= maxDigits) {    // Verifica si se presionó la tecla de Enter o si se llegó al máximo de dígitos
  
    customKey = customKeypad.getKey();                // Escanea el teclado
    
    if (customKey >= '0' && customKey <= '9'){       
      numbers[index++] = customKey;                   // Almacena el valor ingresado
      numbers[index] = '\0';                          // Null termina el string
      printLCD(z,r,String(customKey));                // Muestra el valor ingresado
      z++;                                            // Incrementa la posición del cursor  
    }
    
    if (customKey == '.'){                            // Testea si se ingresó el punto decimal
      if (decimalPoint != ('*')){                     // Testea si el punto decimal ya fue ingresado
        numbers[index++] = '.';                       // Almacena el punto decimal
        numbers[index] = '\0';                        // Null termina el string
        printLCD(z,r,".");                            // Muestra el punto decimal
        z++;                                          // Incrementa la posición del cursor
        decimalPoint = ('*');                         // Marca que el punto decimal fue ingresado
      }
    }

    if(customKey == '<'){                              // Borrar todo lo ingresado
      index = 0;
      z = y;
      printLCD(y,r,"     ");                          // Borra el valor ingresado 
      numbers[index] = '\0';                          // Null termina el string
      decimalPoint = (' ');                           // Resetea el punto decimal
    }
  }
 
  if(customKey == 'E'|| index > maxDigits) {          // Si se presionó la tecla de Enter o se llegó al máximo de dígitos
    x = atof(numbers);                                // Convierte el string a un float
    index = 0;                                        // Resetea el índice
    numbers[index] = '\0';                            // Null termina el string
    decimalPoint = (' ');                             // Resetea el punto decimal
    customKey = 'Z';                                  // Por si vuelvo a llamar a la función
  }
}

//-------------------------------------------- Funciones para el Timer ---------------------------------------------------------
int timer_status() {
  if (timerStarted) {
    return 1; // Timer iniciado
  } else {
    return 2; // Timer detenido
  }
}

void timer_start() {
  if (!timerStarted) {
    startTime = rtc.now();
    timerStarted = true;
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
  } else {
    return elapsedSeconds;
  }
}

String timer_getTime() {
  int totalSeconds = static_cast<int>(timer_getTotalSeconds());

  int hours = totalSeconds / 3600;
  int minutes = (totalSeconds % 3600) / 60;
  int seconds = (totalSeconds % 3600) % 60;

  String formattedTime = "";
  if (hours < 10) {
    formattedTime += "0";
  }
  formattedTime += String(hours) + ":";

  if (minutes < 10) {
    formattedTime += "0";
  }
  formattedTime += String(minutes) + ":";

  if (seconds < 10) {
    formattedTime += "0";
  }
  formattedTime += String(seconds);

  return formattedTime;
}

//-----------------------------Show limits Stored Data for Current, Power and Temp-----------------------------
void setupLimits (void) {
  lcd.clear();
  printLCD(1, 0, "Maximum Limits Set");                 // Muestra el titulo 
  printLCD(0, 1, "Current Limit=");       
  printLCD(18, 1, "A");
  CurrentCutOff = loadFromEEPROM(ADD_CURRENT_CUT_OFF);  // Lee el valor de la corriente de corte desde la EEPROM
  printLCD(15, 1, String(CurrentCutOff));               // Muestra el valor de la corriente de corte 

  printLCD(0, 2, "Power Limit  =");
  printLCD(19, 2, "W");
  PowerCutOff = loadFromEEPROM(ADD_POWER_CUT_OFF);      // Lee el valor de la potencia de corte desde la EEPROM
  printLCD(15, 2, String(PowerCutOff));                 // Muestra el valor de la potencia de corte 

  printLCD(0, 3, "Temp. Limit  =");
  printLCD(17, 3, String((char)0xDF) + "C");
  tempCutOff = loadFromEEPROM(ADD_TEMP_CUT_OFF);        // Lee el valor de la temperatura de corte desde la EEPROM
  printLCD(15, 3, String(tempCutOff));                  // Muestra el valor de la temperatura de corte 
}

//----------------------------------------Transient Mode--------------------------------------------
void transientMode (void) {

  if(Mode != "TL"){

    lcd.noCursor(); lcd.clear();                  // Apaga el cursor y borra la pantalla del LCD

    printLCD(3, 0, "Transient Mode");     
    printLCD(0, 1, "Set Low  I=");
    printLCD(19, 1, "A");
    y = 11; z = 11; r = 1;                        // Setea las posiciones de la pantalla del LCD
    inputValue(5);                                // Obtiene el valor ingresado por el usuario
    LowCurrent = min(x, CurrentCutOff);           // Limita la corriente baja al valor de corte de corriente
    printLCD(11, r, String(LowCurrent,3));       // Muestra el valor de la corriente baja

    printLCD(0, 2, "Set High I=");
    printLCD(19, 2, "A");
    z = 11; r = 2;
    inputValue(5);                                // Obtiene el valor ingresado por el usuario 
    HighCurrent = min(x, CurrentCutOff);          // Limita la corriente alta al valor de corte de corriente
    printLCD(11, r, String(HighCurrent,3));       // Muestra el valor de la corriente alta con tres decimales

    if(Mode == "TC"){                             // Si el modo es Transitorio Continuo
      printLCD(0, 3, "Set Time  = ");
      printLCD(16, 3, "mSec");
      z = 11; r = 3;                              // Setea las posiciones de la pantalla del LCD
      inputValue(5);                              // Obtiene el valor ingresado por el usuario  

      transientPeriod = x;                        // Guarda el valor del tiempo de transitorio
      printLCD(11, r, String(transientPeriod));   // Muestra el valor de la duración del transitorio 
    } else{ clearLCDLine(3); }                    // Borra el 4to. Renlón del LCD

    lcd.clear();                                  // Borra la pantalla del LCD
    toggle = false;                               // Flag para apagar la carga
    load_ON_status(false);                        // Apaga la carga
    } else{  
      transientListSetup();                       // Si el modo es Transitorio de Lista, se llama a la función de configuración de la lista
      lcd.clear();                                // Borra la pantalla del LCD
      toggle = false;                             // Flag para apagar la carga
      load_ON_status(false);                      // Apaga la carga
    }
}

//----------------------------------------Transient Type Selection--------------------------------------------
void transientType() {
  toggle = false;
  exitMode = 0;
  lcd.noCursor(); lcd.clear();                                          // Apaga el cursor y borra la pantalla del LCD
  
  printLCD(3, 0, "Transient Mode");
  printLCD(0, 1, "1 = Continuous");
  printLCD(0, 2, "2 = List");

  while (true) {                                                          // Bucle para garantizar entrada válida
    customKey = customKeypad.waitForKey();
    
    if (customKey == '1' || customKey == '2') {                           // Si la tecla es válida
      Mode = (customKey == '1') ? "TC" : "TL";                            // Asigna el modo de transitorio
      break;                                                              // Salimos del bucle si la tecla es válida
    } 
    else if (customKey == 'M') {                                          // Si se presiona la tecla de Modos
      exitMode = 1;                                                       // Salimos del modo
      break;                                                              // También salimos si se cambia de modo
    } 
    else if (!strchr("34567890<CSE.", customKey) && customKey != NO_KEY) { // Si la tecla no es válida
      break;                                                              // Salimos del bucle si la tecla no es inválida
    }
  }

  lcd.clear();
  if (exitMode == 0) { transientMode(); }                                 // Si no se sale del modo, se llama a la función de modo transitorio
}

//----------------------------------------Transient--------------------------------------------
void Transient (void) {
  if (Mode == "TC" || Mode == "TL"){
    lcd.noCursor();                                         //switch Cursor OFF for this menu 
    printLCD(0, 0, "DC LOAD");              // Muestra el titulo del modo 
    
    if (Mode == "TC"){
    printLCD(0, 2, "Lo=");                   // Muestra el mensaje 
    printLCD(3, 2, String(LowCurrent,3));    // Muestra el valor de la corriente baja 
    printLCD(8, 2, "A");                     // Muestra la unidad 
    printLCD(11, 2, "Hi=");                  // Muestra el mensaje 
    printLCD(14,2, String(HighCurrent,3));   // Muestra el valor de la corriente alta 
    printLCD(19,2, "A");                     // Muestra la unidad 
    printLCD(0, 3, "Time = ");               // Muestra el mensaje 
    printLCD(7, 3, String(transientPeriod)); // Muestra el valor del tiempo 
    printLCD(12, 3, "mSecs");                // Muestra la unidad 
    } else{
      printLCD(0, 3, "  ");                  // Borra el 4to. Renlón del LCD
    }
  }
}

//-------------------------------------Transcient List Setup-------------------------------------------
void transientListSetup() {
  lcd.noCursor(); lcd.clear();                          // Apaga el cursor y borra la pantalla 
  printLCD(0, 0, "Setup Transient List");
  printLCD(0, 1, "Enter Number in List");
  printLCD(0, 2, "(between 2 to 10)");

  do {                                                  // Bucle para garantizar entrada válida
    y = 0; z = 0; r = 3;
    inputValue(2);
  } while (x < 2 || x > 10);                            // Si el valor no está entre 2 y 10, se vuelve a pedir 

  total_instructions = x - 1;                           // Guarda el número total de instrucciones
  lcd.clear();                                          // Borra la pantalla del LCD

  for (int i = 0; i <= total_instructions; i++) {       // Bucle para obtener los valores de la lista    
    printLCD(0, 0, "Set Current " + String(i + 1));     // Pide el valor de corriente en Amperes
    printLCD(16, 1, "A");                               // Muestra la unidad
    y = 0; z = 0; r = 1;
    inputValue(2);
    transientList[i][0] = x;                            // Guarda el valor de la corriente

    printLCD(0, 2, "Set Time " + String(i + 1));        // Pide el valor de tiempo en milisegundos
    printLCD(16, 2, "mSec");
    y = 0; z = 0; r = 3;
    inputValue(5);
    transientList[i][1] = x;                            // Guarda el valor del tiempo

    lcd.clear();                                        // Borra la pantalla 
  }
  current_instruction = 0;                              // Resetea el contador de instrucciones porque finalizo la configuración
}

//-------------------------------------Transcient Load Toggel-------------------------------------------
void transientLoadToggle() {
  if(Mode == "TC"){                                                         // Si em modo es Transitorio Continuo
  current_time = micros();
  if (last_time == 0){
    last_time = current_time;
  } else {
      switch (transient_mode_status){
        case (false):
          if ((current_time - last_time) >= (transientPeriod * 1000.0)){    // Comprueba si el tiempo transcurrido es mayor o igual al tiempo de transitorio
             transientSwitch(LowCurrent, true);                             // Cambia a la corriente baja       
          }
        break;
        case (true):
          if ((current_time - last_time) >= (transientPeriod * 1000.0)){    // Comprueba si el tiempo transcurrido es mayor o igual al tiempo de transitorio
            transientSwitch(HighCurrent, true);                             // Cambia a la corriente alta
          }
        break; 
      } 
    }
  }

  if(Mode == "TL"){                                                         // Si el modo es Transitorio de Lista
    if (Load == 1){                                                         // Solo lo hara si la carga está encendida
      current_time = micros();
      if (last_time == 0){                                                  // Si es la primera vez que se ejecuta
        last_time = current_time;                                           // Inicializa el tiempo
        transientPeriod = transientList[current_instruction][1];            // Obtiene el tiempo de transitorio de la lista
        transientSwitch(transientList[current_instruction][0], false);
      }
      if((current_time - last_time) >= transientList[current_instruction][1] * 1000){   // Comprueba si el tiempo transcurrido es mayor o igual al tiempo de transitorio de la lista
        current_instruction++;                                                          // Incrementa el contador de instrucciones
        if(current_instruction > total_instructions){                                   // Si el contador es mayor al total de instrucciones
          current_instruction = 0;                                                      // Resetea el contador de instrucciones
        }
        transientPeriod = transientList[current_instruction][1];                        // Obtiene el tiempo de transitorio de la lista
        transientSwitch(transientList[current_instruction][0], false);                  // Cambia a la corriente de la lista
      }
    }
  }
}

//-------------------------------------Transcient Switch-------------------------------------------
void transientSwitch(float current_setting, boolean toggle_status) {
  if (toggle_status){
    transient_mode_status = !transient_mode_status;
  }
  setCurrent = current_setting;
  last_time = current_time;
}

//------------------------------------- Funciones para el LCD -------------------------------------------
// Función para limpiar una línea específica
void clearLCDLine(int row) {
  printLCD(0, row, "                    "); // 20 espacios para borrar la línea
}

// Función para imprimir un mensaje en una posición específica
void printLCD(int col, int row, const String &message) {
  lcd.setCursor(col, row);
  lcd.print(message);
}

//-------------------------------- Graba en EEPROM -----------------------------
void saveToEEPROM(int address, float value) {
  float PreviousValue;
  EEPROM.get(address, PreviousValue);
  
  if (PreviousValue != value) {  // Solo escribe si el valor ha cambiado
      EEPROM.put(address, value);
  }
}
//-------------------------------- Lee de EEPROM -----------------------------
float loadFromEEPROM(int address) {
  float value;
  EEPROM.get(address, value);
  return value;
}
