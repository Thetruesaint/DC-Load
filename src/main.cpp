#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_ADS1X15.h>
#include <Adafruit_MCP4725.h>
#include <Keypad.h>                           //http://playground.arduino.cc/Code/Keypad
#include <EEPROM.h>                           //include EEPROM library used for storing setup data
#include <math.h>                             
#include <RTClib.h>


// put function declarations here:
void setup();
void loop();
void load_ON_status(boolean loadonoff);
void read_encoder();
void readKeypadInput();
void LoadSwitch();
void LimitsChecks ();
void ActualReading();
void currentDisplayCal();
void powerLevelCutOff();
void temperatureCutOff (void);
void displayEncoderReading (void);
void CursorPosition(void) ;
void readVoltageCurrent (void);
void dacControlVoltage (void);
void dacControl (void) ;
void batteryCapacity (void);
void tempcheck(void);
void Current(void);
void Power(void) ;
void Resistance(void) ;
void BatteryCapacity(void) ;
void batteryType (void);
void batteryTypeSelected (void);
void setBatteryCutOff (void);
void userSetUp (void);
void inputValue (int maxDigits);
int timer_status() ;
void timer_start() ;
void timer_stop() ;
void timer_reset() ;
float timer_getTotalSeconds() ;
String timer_getTime() ;
void setupLimits (void) ;
void transientMode (void) ;
void transientType (void) ;
void Transient (void) ;
void transientListSetup() ;
void transientLoadToggle() ;
void transientSwitch(float current_setting, boolean toggle_status) ;

Adafruit_MCP4725 dac;                         // Objeto dac para el MP4725
Adafruit_ADS1115 ads;                         // Objeto ads para el ADS115
LiquidCrystal_I2C lcd(0x27, 20, 4);           // Objeto lcd.Sddress to 0x27 for a 20 chars and 4 line display
RTC_DS1307 rtc;                               // Objeto rtc para el DS1307

//-------------------I/O Pins------------------------------------------------------------

const byte templm35 = A0;                     // Sensado de Temperatura
const byte fansctrl = A2;                     // Endendido de Fans 
const byte ENC_A = 3;                         // Encoder Pin A
const byte ENC_B = 2;                         // Encoder Pin B 
const byte encbtn = 4;                        // Encoder button
const byte crrsnsr = 1;                       // Input A1 from ADC
const byte vltgmtr = 3;                       // Input A3 from ADC
const byte LoadOnOff = 15;                    // Input A1 used as a digital pin to set Load ON/OFF

//--------------- Variables para Encoder -------------------------------------------------

unsigned long lastButtonPress = 0;            //Use this to store if the encoder button was pressed or not
unsigned long _lastIncReadTime = micros(); 
unsigned long _lastDecReadTime = micros(); 
int _pauseLength = 25000;
int _fastIncrement = 10;
volatile float encoderPosition = 0;           // Antes era volatile float
volatile float factor = 0;                    // Factor de escala del Encoder
volatile unsigned long encoderMax = 999000;   // sets maximum Rotary Encoder value allowed CAN BE CHANGED AS REQUIRED (was 50000)

//--------------- Variables de operacion y Modos CC, CR y CP----------------------------------------

int16_t adc0, adc1, adc2, adc3;
unsigned long controlVoltage = 0;             //used for DAC to control MOSFET
float current = 0;                            // Corriente de Load
float voltage = 0;                            // voltage de Load
float ActualVoltage = 0;                      //variable used for Actual Voltage reading of Load
float ActualCurrent = 0;                      //variable used for Actual Current reading of Load
float ActualPower = 0;                        //variable used for Actual Power reading of Load
int CP = 8;                                   //cursor start position
boolean toggle = false;                       //used for toggle of Load On/Off button
int Load = 0;                                 //Load On/Off flag para Logueo
float reading = 0;                            //variable for Rotary Encoder value divided by 1000
float setCurrent = 1.000;                     //variable used for the set current of the load
float setPower = 20;                          //variable used for the set power of the load
float setResistance = 30;                     //variable used for the set resistance of the load
float setCurrentCalibrationFactor = 0.4095;   //Factor del DAC, paso a convertir 5a1 (5V a 1V de Ref.) osea Corriente máxima 10A
float setControlCurrent = 0;                  //variable used to set the temporary store for control current required
int setReading = 0;                           //variable usada en la funcion dacControlVoltage() pero nada mas... ver si sirve
int CurrentCutOff = EEPROM.read(0x00);        //Colocar valores enteros, ya que después del punto decimal no lo guartda.
int PowerCutOff = EEPROM.read(0x20);          //Colocar valores enteros, ya que después del punto decimal no lo guartda.
int tempCutOff = EEPROM.read(0x40);           //Colocar valores enteros, ya que después del punto decimal no lo guartda.
float ResistorCutOff = 999;                   //maximum Resistor we want to deal with in software
String Mode ="  ";                            //used to identify which mode

//----------------------------------------Variables para el Keypad-------------------------------------------

const byte ROWS = 4;                          //four rows
const byte COLS = 4;                          //four columns
//define the symbols on the buttons of the keypads
char hexaKeys[ROWS][COLS] = {
  {'1','2','3','M'},
  {'4','5','6','C'},
  {'7','8','9','S'},
  {'<','0','.','E'}
};
int functionIndex = 1;                        // Es 1 para que el siguiente Modo sea Power() ya que se inicia en Current()

byte rowPins[ROWS] = {5, 6, 7, 8}; //connect to the row pin outs of the keypad
byte colPins[COLS] = {9, 10, 11, 12}; //connect to the column pin outs of the keypad

Keypad customKeypad = Keypad(makeKeymap(hexaKeys), rowPins, colPins, ROWS, COLS);
char customKey;
char decimalPoint;                           //used to test for more than one press of * key (decimal point)
char numbers[10];                            //keypad number entry - Plenty to store a representation of a float
byte index = 0;
float x = 0;

//-----------------------------------Variables de coordenadas para ubicar valores en LCD--------------------
int z = 1;                                  //Posición en renglón (En CC,CP y CR dejo lugar para poner caracter ">")
int y = 0;                                  //Posición provisoria
int r = 0;                                  //Renglon  

//---------------------------------------Inicializo Variables para Modo Baterias-----------------------------
float Seconds = 0;                            //time variable used in Battery Capacity Mode (BC)
bool timerStarted = false;
DateTime startTime;
float elapsedSeconds = 0.0;
float BatteryLife = 0;                        //
float BatteryLifePrevious = 0;                //
float SecondsLog = 0;                         //variable used for data logging of the time in seconds
float BatteryCutoffVolts;                     //used to set battery discharge cut-off voltage
float BatteryCurrent;                         //Variable dedicada para la corriente de bateria
float LoadCurrent;                            //Almacena por un momento el ActualCurrent
float MaxBatteryCurrent = 10.0;               //maximum battery current allowed for Battery Capacity Testing
float LiPoCutOffVoltage = 3.5;                //set cutoff voltage for LiPo Discharge
float LionCutOffVoltage = 2.8;                //set cutoff voltage for Lion Discharge
float LiPoStoragVoltage = 3.8;                //set cutoff voltage for LiPo Storage
float LionStoragVoltage = 3.7;                //set cutoff voltage for Lion Storage
String BatteryType ="    ";
byte exitMode = 0;                            //used to exit battery selection menu and return to CC Mode

//---------------------------------------Variables para Control de Temperatura-----------------------------
int temp = 0;                                 // Registra temperatura
const int tmpchk_time = 500;                  // Perdiodo de control de temperatura (miliseg.)
unsigned long Last_tmpchk = 0;                // Tiempo desde el ùltimo chequeo de temperatura
const int fan_on_duration = 30000;            // Tiempo en miliseg. para mantener los fans encendidos (30 segundos)
unsigned long fan_on_time = 0;                // Tiempo que lleva encendido el Fan
bool fans_on = false;                         // Estado de los Coolers

//---------------------------------------Variables para Modo Transient------------------------------------
float LowCurrent = 0;                         //the low current setting for transcient mode
float HighCurrent = 0;                        //the high current setting for transcient mode
unsigned long transientPeriod;                //used to store pulse time period in transcient pulse mode
unsigned long current_time;                   //used to store the current time in microseconds
unsigned long last_time = 0;                  //used to store the time of the last transient switch in micro seconds
boolean transient_mode_status = false;        //used to maintain the state of the trascient mode (false = low current, true = high current)
float transientList [10][2];                  //array to store Transient List data
int total_instructions;                       //used in Transient List Mode
int current_instruction;                      //used in Transient List Mode

void setup() {

  //-------------------------------------Inicializa perifericos-------------------------------------------
  lcd.begin(20,4);                              // initialize the lcd, default address 0x27
  ads.begin();                                  // initialize the ads, default address 0x48
  ads.setGain(GAIN_TWOTHIRDS);                  // 2/3x gain +/- 6.144V  1 bit = 0.1875mV
  dac.begin(0x60);                              // initialize the ads, default address 0x60
  rtc.begin();                                  // Inicializa el RTC en teoría en address 0x68
  Serial.begin(9600);                           // Para Debugs y Logs
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
  lcd.setCursor(1,0);
  lcd.print("DC Electronic Load");
  lcd.setCursor(1,1);
  lcd.print(date +" - "+ time);
  lcd.setCursor(0,2);             
  lcd.print("Guy Nardin");
  lcd.setCursor(0,3);
  lcd.print("v1.63");                   // Modos CC, CP y CR funcionando con Teclado. Ajustes para el Set con Load ON/Off
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
                                       
                                            
  delay(2000);
  lcd.clear();
  //---------------------------------------Chequea y Muestra los límites configurados----------------------
 if(CurrentCutOff > 10){                // Si es mayor puede que sea un Nano nuevo, llamar a reconfigurar lìmites (EPPROM inicializa con todos 255)
    userSetUp();
     }
  setupLimits();
  delay(2000);
  lcd.clear();
  //-------------------------------------- Pantalla por Default ------------------------------------------
  load_ON_status(false);               //indicate that LOAD is off at start up
  Current();
  }

//------------------------------------- Bucle Principal-------------------------------------------------
void loop() {
            readKeypadInput();                                     //read Keypad entry

            if (digitalRead(LoadOnOff) == LOW) {
              LoadSwitch();                                        //Load on/off
              delay(200);                                          //simple key bounce delay 
            }

            Transient();                                           //test for Transient Mode

            lcd.setCursor(18,3);                                   //sets display of Mode indicator at bottom right of LCD
            lcd.print(Mode);                                       //display mode selected on LCD (CC, CP, CR or BC)

            if(Mode != "TC" && Mode != "TL"){                      //if NOT transient mode then Normal Operation

            reading = encoderPosition/1000;                        //read input from rotary encoder
            LimitsChecks();                                        //Chequea Limites de todos los Modos y resetea a máximo de ser necesario
            displayEncoderReading();                               //display rotary encoder input reading on LCD
            CursorPosition();                                      //check and change the cursor position if cursor button pressed

            }else{
              transientLoadToggle();                               //Start Transient Mode
            }
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
      lcd.setCursor(8,0);
      lcd.print("OFF");
      dac.setVoltage(0,false);                          //Ensures Load is OFF - sets DAC output voltage to 0
      Load = 0;                                         //Setea Flag para el Log
    }
    else if (loadonoff) {
      lcd.setCursor(8,0);
      lcd.print("ON ");
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
    
    if((micros() - _lastIncReadTime) < _pauseLength && factor == 1) {
      encoderPosition = encoderPosition + _fastIncrement;
    }
    _lastIncReadTime = micros();
    encoderPosition = encoderPosition + factor;              // Update encoderPosition Up
    encval = 0;
  }
  else if( encval < -3 ) {        // Four steps backward
    
    if((micros() - _lastDecReadTime) < _pauseLength) {
      encoderPosition = encoderPosition - _fastIncrement;
    }
    _lastDecReadTime = micros();
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
             
  if(customKey == 'M'){                                     //check if Constant Current button pressed
    toggle = false;                                         //switch Load OFF
    load_ON_status(false);                                  //Si hay cambio de Modo, me aseguro de que se desconecte la carga.
      if (functionIndex == 0) {
        Current();
      } else if (functionIndex == 1) {
        Power();
      } else if (functionIndex == 2) {
        Resistance();
      } else if (functionIndex == 3) {
            batteryType();
            if (exitMode == 0){                           // Si se selecciona un tipo de baterìa, continua con la rutina                                   
              lcd.setCursor(16,2);
              lcd.print(BatteryType);                     //print battery type on LCD 
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
         
  if(customKey == 'S'){                                   //Uso futuro para el boton "Shift"
  }

  if (Mode != "BC"){
    if(customKey >= '0' && customKey <= '9' && index < 6){  //check for keypad number input, no mas de 5 digitos, no es necesario
       numbers[index++] = customKey;
       numbers[index] = '\0';
       lcd.setCursor(z,3);                              
       lcd.print(customKey);                              //show number input on LCD
       z = z+1;
     }
  
    if(customKey == '.'){                                   //check if decimal button key pressed
      if (decimalPoint != ('*')){                         //test if decimal point entered twice - if so skip 
        numbers[index++] = '.';
        numbers[index] = '\0';
        lcd.setCursor(z,3);
        lcd.print(".");
        z = z+1;
        decimalPoint = ('*');                               //used to indicate decimal point has been input
        }
      }

    if(customKey == 'E') {                                  //Tecla "Enter" carga el valor en el Set
      x = atof(numbers);     
      reading = x;
      encoderPosition = reading*1000;
      index = 0;
      numbers[index] = '\0';
      z = 1;                                           //sets column position for LCD displayed character
      lcd.setCursor(0,3);
      lcd.print("        ");
      decimalPoint = (' ');                            //clear decimal point test character reset
    }

  if(customKey == '<'){                                   //clear entry
    index = 0;
    z = 1;
    lcd.setCursor(z,3);
    lcd.print("        ");
    numbers[index] = '\0';                              
    decimalPoint = (' ');                                 //clear decimal point test character reset
    } 
  }
  }
//-----------------------------Toggle Current Load ON or OFF------------------------------
void LoadSwitch(void) {
    if(toggle) {
      load_ON_status(false);
      setCurrent = 0;                                     //reset setCurrent to zero
      toggle = !toggle;
      }
    else {
      load_ON_status(true);
      lcd.setCursor(0,3);
      lcd.print("                    ");                 //clear bottom line of LCD
      toggle = !toggle;
    }
  }

//-----------------------------Limit Maximum Current Setting-------------------------------
void LimitsChecks (void) {


  if (Mode == "CC" && reading > CurrentCutOff){          //Limit maximum Current Setting
      reading = CurrentCutOff;                           // Sino lo asigno a la variable primero, aparecen Bugs en la variable encoderPosition
      encoderPosition = reading * 1000;                  //keep encoder position value at maximum Current Limit
      lcd.setCursor(0,3);
      lcd.print("                    ");                 //20 spaces to clear last line of LCD 
  }

  if (Mode == "CP" && reading > PowerCutOff) {           //Limit maximum Current Setting
      reading = PowerCutOff;                             // Sino lo asigno a la variable primero, aparecen Bugs en la variable encoderPosition
      encoderPosition = reading * 1000;                  //keep encoder position value at maximum Current Limit
      lcd.setCursor(0,3);
      lcd.print("                    ");                 //20 spaces to clear last line of LCD 
  }

  if (Mode == "CR" && reading > ResistorCutOff ) {       //Limit maximum Current Setting
      reading = ResistorCutOff;                          // Sino lo asigno a la variable primero, aparecen Bugs en la variable encoderPosition
      encoderPosition = reading * 1000;                  //keep encoder position value at maximum Current Limit
      lcd.setCursor(0,3);
      lcd.print("                    ");                 //20 spaces to clear last line of LCD 
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
void currentDisplayCal (void) {                           // Ojo que esto enmascara lecturas de corriente de Offset de los Op. Amp. solo cuando el LOAD esta en ON, se ve.

  if(ActualCurrent <= 0){
    ActualCurrent = 0;}
    else if(Load == 0){       
    ActualCurrent = 0;}
  }

//----------------------Power Level Cutoff Routine-------------------------------------------
void powerLevelCutOff (void) {
  if (ActualPower  > PowerCutOff){                        //Check if Power Limit has been exceed
  reading = 0;
  encoderPosition = 0; 
  lcd.setCursor(0,3);
  lcd.print("                    ");
  lcd.setCursor(0,3);
  lcd.print("Exceeded Power");
  load_ON_status(false);
  toggle = false;                                         //switch Load Off
  delay(3000);                                            // Mostramos el mensaje por 3 segundos.
  lcd.setCursor(0,3);
  lcd.print("                    ");                      //Limpiamos el último renglón para que no se pise con entradas de teclado.
  }
  }
//------------------------------------------High Temperature Cut-Off--------------------------------------------------------------
void temperatureCutOff (void) {
  if (temp >= tempCutOff){                                 //if Maximum temperature is exceeded
  reading = 0;
  encoderPosition = 0; 
  lcd.setCursor(0,3);
  lcd.print("                    ");
  lcd.setCursor(0,3);
  lcd.print("Over Temperature");
  load_ON_status(false);
  toggle = false;                                         //switch Load Off
  delay(3000);                                            // Mostramos el mensaje por 3 segundos.
  lcd.setCursor(0,3);
  lcd.print("                    ");                      //Limpiamos el último renglón para que no se pise con entradas de teclado.
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

  }  
//-----------------------DAC Control Voltage for Mosfet---------------------------------------
void dacControlVoltage (void) {
  if (Mode == "CC"){
  setCurrent = reading*1000;                                //set current is equal to input value in Amps
  setReading = setCurrent;                                  //show the set current reading being used
  setControlCurrent = setCurrent * setCurrentCalibrationFactor;
  controlVoltage = setControlCurrent; 
  }

  if (Mode == "CP"){
  setPower = reading*1000;                                  //in Watts
  setReading = setPower;
  setCurrent = setPower/ActualVoltage;
  setControlCurrent = setCurrent * setCurrentCalibrationFactor;
  controlVoltage = setControlCurrent;                       //
  }

  if (Mode == "CR"){
  setResistance = reading;                                  //in ohms
  setReading = setResistance;
  setCurrent = (ActualVoltage)/setResistance*1000;
  setControlCurrent = setCurrent * setCurrentCalibrationFactor;
  controlVoltage = setControlCurrent; 
  }

  if (Mode == "TC" || Mode == "TL"){                            //Transient Modes
  setControlCurrent = (setCurrent * 1000) * setCurrentCalibrationFactor;
  controlVoltage = setControlCurrent; 
  }
  }
//--------------------------Set DAC Voltage--------------------------------------------
void dacControl (void) {
  if (!toggle){
    dac.setVoltage(0,false);                                 //set DAC output voltage to 0 if Load Off selected
    if(Mode == "BC" && ActualVoltage >= BatteryCutoffVolts && timer_status() == 1){
    timer_stop();
    }
  
  }else{
    dac.setVoltage(controlVoltage,false);                   //set DAC output voltage for Range selected
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
    setControlCurrent = setCurrent * setCurrentCalibrationFactor;
    controlVoltage = setControlCurrent;

    lcd.setCursor(0,3);
    lcd.print (timer_getTime());                           //start clock and print clock time

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
    lcd.setCursor(13,3);
    lcd.print("mAh");
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
    temp = analogRead(templm35);                                 // Tomar temperatura
    temp = temp * 0.48828125;                                    // Convertir a Celsius

    Last_tmpchk = hldtmp_time;

    if (temp >= 35) {                                           // Controlar el encendido de los fans y el temporizador
      digitalWrite(fansctrl, HIGH);                             // Encender los fans si la temperatura es mayor o igual a 35°C
      fans_on = true;
      fan_on_time = hldtmp_time;
    } else if (fans_on && (hldtmp_time - fan_on_time) >= fan_on_duration) {
        digitalWrite(fansctrl, LOW);                            // Apagar los fans después del tiempo especificado si estaban encendidos
        fans_on = false;
      }
    
    lcd.setCursor(16, 0);
    lcd.print(temp);                                            // Actualizar temperatura en el LCD
    lcd.print((char)0xDF);
    lcd.print("C");

  }
  }
//-----------------------Select Constant Current LCD set up--------------------------------
void Current(void) {
  Mode = ("CC");
  lcd.setCursor(0,0);
  lcd.print("DC LOAD");  
  lcd.setCursor(0,2);
  lcd.print("                ");
  lcd.setCursor(0,2);
  lcd.print("Set I = ");
  lcd.setCursor(16,2);
  lcd.print("    ");
  lcd.setCursor(14,2);
  lcd.print("A");
  lcd.setCursor(0,3);                                   //clear last line of time info
  lcd.print("                    ");                    //20 spaces so as to allow for Load ON/OFF to still show
  CP = 9;                                               //sets cursor starting position to units.
  }

//----------------------Select Constant Power LCD set up------------------------------------
void Power(void) {
  Mode = ("CP");
  lcd.setCursor(0,0);
  lcd.print("DC LOAD");
  lcd.setCursor(0,2);
  lcd.print("                ");
  lcd.setCursor(0,2);
  lcd.print("Set W = ");
  lcd.setCursor(16,2);
  lcd.print("    ");
  lcd.setCursor(14,2);
  lcd.print("W");
  lcd.setCursor(0,3);                                   //clear last line of time info
  lcd.print("                    ");                    //20 spaces so as to allow for Load ON/OFF to still show
  CP = 10;                                               //sets cursor starting position to units.
  }

//----------------------- Select Constant Resistance LCD set up---------------------------------------
void Resistance(void) {
  Mode = ("CR");
  lcd.setCursor(0,0);
  lcd.print("DC LOAD");  
  lcd.setCursor(0,2);
  lcd.print("                ");
  lcd.setCursor(0,2);
  lcd.print("Set R = ");
  lcd.setCursor(16,2);
  lcd.print("    ");
  lcd.setCursor(14,2);
  lcd.print((char)0xF4);                                // Símbolo de Ohms
  lcd.setCursor(0,3);                                   //clear last line of time info
  lcd.print("                    ");                    //20 spaces so as to allow for Load ON/OFF to still show
  CP = 10;                                               //sets cursor starting position to units.
  }
//----------------------- Select Battery Capacity Testing LCD set up---------------------------------------
void BatteryCapacity(void) {
  Mode = ("BC");
  lcd.setCursor(0,0);
  lcd.print("BATTERY");
  lcd.setCursor(0,2);
  lcd.print("                ");
  lcd.setCursor(0,2);
  lcd.print("Set I = ");
  lcd.setCursor(14,2);
  lcd.print("A");
  lcd.setCursor(0,3);                                   //clear last line of time info
  lcd.print("                    ");                    //20 spaces so as to allow for Load ON/OFF to still show
  }

//----------------------Battery Action Selection Routine------------------------------------------------
void batteryType (void) {
  exitMode = 0;                                         //reset EXIT mode
  lcd.noCursor();                                       //switch Cursor OFF for this menu               
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Select:             ");  
  lcd.setCursor(0,1);
  lcd.print("Stor.: 1=LiPo 2=LiOn");
  lcd.setCursor(0,2);
  lcd.print("Disc.: 3=LiPo 4=LiOn");
  lcd.setCursor(0,3);                                   //clear last line of time info
  lcd.print("5=Set Voltage");                           //20 spaces so as to allow for Load ON/OFF to still show

  customKey = customKeypad.waitForKey();                //stop everything till the user press a key.

  if (customKey == '1'){
  BatteryCutoffVolts = LiPoStoragVoltage;
  BatteryType = ("LiPo");
    }

  if (customKey == '2'){
  BatteryCutoffVolts = LionStoragVoltage;
  BatteryType = ("Lion");
    }

  if (customKey == '3'){
  BatteryCutoffVolts = LiPoCutOffVoltage;
  BatteryType = ("LiPo");  
    }

  if (customKey == '4'){
  BatteryCutoffVolts = LionCutOffVoltage;
  BatteryType = ("Lion"); 
    }

  if (customKey == '5'){ 
  BatteryType = ("Custom");
    }

  if (customKey == 'M'){                                  //Exit selection screen
  customKey = 'NO_KEY';
  exitMode = 1;
    }

  if (customKey == '7' || customKey == '8' || customKey == '9' || customKey == '0' || customKey == '6' || customKey == 'C' || customKey == 'S' || customKey == 'E' || customKey == '<' || customKey == '.'){
  batteryType();                                                        //ignore other keys
    }

  if(BatteryType == "Custom" && exitMode != 1){
  setBatteryCutOff();
    }
  
  if (BatteryType != "Custom" && exitMode != 1){
    lcd.clear();
    lcd.setCursor(2,0);
    lcd.print("Battery Selected");
    lcd.setCursor(8,1);
    lcd.print(BatteryType);                                 //display battery type selected
    lcd.setCursor(2,2);
    lcd.print("Number of Cells?");                          //Preguntar por cantidad de celdas.

    customKey = customKeypad.waitForKey();                  //stop everything till the user press a key.

    if (customKey == '1'){
    BatteryCutoffVolts = BatteryCutoffVolts * 1;
      }

    if (customKey == '2'){
    BatteryCutoffVolts = BatteryCutoffVolts * 2;
      }

    if (customKey == '3'){
    BatteryCutoffVolts = BatteryCutoffVolts * 3;  
      }

    if (customKey == '4'){
    BatteryCutoffVolts = BatteryCutoffVolts * 4;
      }

    if (customKey == '5'){ 
    BatteryCutoffVolts = BatteryCutoffVolts * 5;
      }
    
    if (customKey == '6'){ 
    BatteryCutoffVolts = BatteryCutoffVolts * 6;
      }


    if (customKey == '7' || customKey == '8' || customKey == '9' || customKey == '0' || customKey == 'M' || customKey == 'C' || customKey == 'S' || customKey == 'E' || customKey == '<' || customKey == '.'){
    batteryTypeSelected();                                                        //ignore other keys
      }
  }

  batteryTypeSelected();                                    //briefly display battery type selected and discharge cut off voltage
  lcd.clear();

  }
//--------------------------Battery Selected Information--------------------------------------------
void batteryTypeSelected (void) {
  if (exitMode !=1){                                      //if battery selection was EXIT then skip this routine
  lcd.clear();
  lcd.setCursor(2,0);
  lcd.print("Battery Selected");
  lcd.setCursor(8,1);
  lcd.print(BatteryType);                                 //display battery type selected
  lcd.setCursor(2,2);
  lcd.print("Discharge Cutoff");
  lcd.setCursor(6,3);
  lcd.print(BatteryCutoffVolts);                          //display battery discharge cut off voltage
  lcd.print(" volts");                                    
  delay(3000);
  }
  }

//--------------------------Set Battery Cut-Off Voltage--------------------------------------------
void setBatteryCutOff (void) {

  lcd.clear();
  lcd.setCursor(3,0);
  lcd.print("Enter Battery");
  lcd.setCursor(2,1);
  lcd.print("Cut-Off Voltage");
  y = 8;
  z = 8;
  r = 2;
  lcd.setCursor(z-1,r);
  lcd.print(">");
  lcd.setCursor(5,3);
  lcd.print("(0 to 25V)");

  inputValue(4);
    while (x > 25 || x == 0) {
      y = 8;
      z = 8;
      r = 2;
      index = 0;
      lcd.setCursor(z,r);
      lcd.print("     ");
      inputValue(4);
  }
  BatteryCutoffVolts = x;
  lcd.clear();
  }

//-------------------------------------User set up for limits-------------------------------------------------
void userSetUp (void) {
  load_ON_status(false);
  y = 14;
  z = 14;
  
  lcd.noCursor();                                       //switch Cursor OFF for this menu               
  lcd.clear();
  lcd.setCursor(4,0);
  lcd.print("User Set-Up");
  lcd.setCursor(0,1);
  lcd.print("Current Limit=");
  lcd.setCursor(19,1);
  lcd.print("A");
  r = 1;
  inputValue(2);
  CurrentCutOff = x;
    if(CurrentCutOff > 10){                     //Test and go to user set limits if required
    CurrentCutOff = 10;
     }
  EEPROM.write(0x00, CurrentCutOff);          //Guarda el valor entero solamente, no ingresar con decimales.
  lcd.setCursor(14,r);
  lcd.print(CurrentCutOff);
  z = 14;

  lcd.setCursor(0,2);
  lcd.print("Power Limit  =");
  lcd.setCursor(19,2);
  lcd.print("W");
  r = 2;
  inputValue(3);
  PowerCutOff = x;
    if(PowerCutOff > 300){                     //Test and go to user set limits if required
    PowerCutOff = 300;
     }
  EEPROM.write(0x20, PowerCutOff);              //Guarda el valor entero solamente, no ingresar con decimales.
  lcd.setCursor(14,r);
  lcd.print(PowerCutOff);
  z = 14;

  lcd.setCursor(0,3);
  lcd.print("Temp. Limit  =");
  lcd.setCursor(18,3);
  lcd.print((char)0xDF);
  lcd.print("C");
  r = 3;
  inputValue(2);
  tempCutOff = x;
    if(tempCutOff > 99){                     //Test and go to user set limits if required
    tempCutOff = 99;
     }
  EEPROM.write(0x40, tempCutOff);             //Guarda el valor entero solamente, no ingresar con decimales.
  lcd.setCursor(14,r);
  lcd.print(tempCutOff);

  setupLimits();                                        //Mostrar los limites configurados
  delay(1000);

  lcd.clear();
  load_ON_status(false);
  Current();                                            //Go to Constant Current routine (default)
  encoderPosition = 0;                                  //reset encoder reading to zero
  customKey = 'M';                                      //Vuelve a Modos
  }

//------------------------Key input used for UserSetUp------------------------
void inputValue (int maxDigits) {

 while(customKey != 'E' && index <= maxDigits){           //check if enter pressed
  
  customKey = customKeypad.getKey();
  if(customKey >= '0' && customKey <= '9'){               //check for keypad number input
       numbers[index++] = customKey;
       numbers[index] = '\0';
       lcd.setCursor(z,r);                              
       lcd.print(customKey);                              //show number input on LCD
       z = z+1;
     }
  
  if(customKey == '.'){                                   //Decimal point
      if (decimalPoint != ('*')){                         //test if decimal point entered twice - if so skip
      numbers[index++] = '.';
      numbers[index] = '\0';
      lcd.setCursor(z,r);
      lcd.print(".");
      z = z+1;
      decimalPoint = ('*');                               //used to indicate decimal point has been input
        }
     }

 if(customKey == '<'){                                    //clear entry
    index = 0;
    z = y;
    lcd.setCursor(y,r);
    lcd.print("     ");
    numbers[index] = '\0';                                //
    decimalPoint = (' ');                                 //clear decimal point test character reset
     }
  }
 
  if(customKey == 'E'|| index > maxDigits) {                                  //enter value 
    x = atof(numbers);     
    index = 0;
    numbers[index] = '\0';
    decimalPoint = (' ');                                 //clear decimal point test character reset
    customKey = 'Z';                                      // Por si vuelvo a llamar a la función
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
  lcd.setCursor(1,0);
  lcd.print("Maximum Limits Set");
  lcd.setCursor(0,1);
  lcd.print("Current Limit=");
  lcd.setCursor(18,1);
  lcd.print("A");
  lcd.setCursor(15,1);
  CurrentCutOff = EEPROM.read(0x00);
  lcd.print(CurrentCutOff);

  lcd.setCursor(0,2);
  lcd.print("Power Limit  =");
  lcd.setCursor(19,2);
  lcd.print("W");
  lcd.setCursor(15,2);
  PowerCutOff = EEPROM.read(0x20);
  lcd.print(PowerCutOff);

  lcd.setCursor(0,3);
  lcd.print("Temp. Limit  =");
  lcd.setCursor(17,3);
  lcd.print((char)0xDF);
  lcd.print("C");
  tempCutOff = EEPROM.read(0x40);
  lcd.setCursor(15,3);
  lcd.print(tempCutOff);
  }
  


//----------------------------------------Transient Mode--------------------------------------------
void transientMode (void) {

  if(Mode != "TL"){

    y = 11;
    z = 11;
    
    lcd.noCursor();                                       //switch Cursor OFF for this menu               
    lcd.clear();
    lcd.setCursor(3,0);
    lcd.print("Transient Mode");  
    lcd.setCursor(0,1);
    lcd.print("Set Low  I=");
    lcd.setCursor(19,1);
    lcd.print("A");
    r = 1;
    inputValue(5);
    
    if(x >= CurrentCutOff){
      LowCurrent = CurrentCutOff;
    }else{
      LowCurrent = x;
    }
    lcd.setCursor(11,r);
    lcd.print(LowCurrent,3);
    z = 11;

    lcd.setCursor(0,2);
    lcd.print("Set High I=");
    lcd.setCursor(19,2);
    lcd.print("A");
    r = 2;
    inputValue(5);
    if(x >= CurrentCutOff){
      HighCurrent = CurrentCutOff;
    }else{
      HighCurrent = x;
    }
    lcd.setCursor(11,r);
    lcd.print(HighCurrent,3);

  if(Mode == "TC"){
    z = 11;

    lcd.setCursor(0,3);
    lcd.print("Set Time  = ");
    lcd.setCursor(16,3);
    lcd.print("mSec");
    r = 3;
    inputValue(5);
    transientPeriod = x;
    lcd.setCursor(11,r);
    lcd.print(transientPeriod);
    } else{
    lcd.setCursor(0,3);
    lcd.print("                    ");
    }

  lcd.clear();
  toggle = false;                                           //switch Load OFF
  load_ON_status(false);
    }else{  
  transientListSetup();
  lcd.clear();
  toggle = false;                                           //switch Load OFF
  load_ON_status(false);
  }
  }

//----------------------------------------Transient Type Selection--------------------------------------------
void transientType (void) {
  toggle = false;                                         //switch Load OFF
  exitMode = 0;                                           //reset EXIT mode
  lcd.noCursor();                                         //switch Cursor OFF for this menu               
  lcd.clear();
  lcd.setCursor(3,0);
  lcd.print("Transient Mode");  
  lcd.setCursor(0,1);
  lcd.print("1 = Continuous");
  lcd.setCursor(0,2);
  lcd.print("2 = List");

  customKey = customKeypad.waitForKey();                  //stop everything till the user press a key.

  if (customKey == '1'){
  Mode = ("TC"); 
    }

  if (customKey == '2'){
   Mode = ("TL");
    }

  if (customKey == 'M'){                                  //Exit selection screen
  exitMode = 1;
    }

  if (customKey == '3' || customKey == '4' || customKey == '5' || customKey == '6' || customKey == '7' || customKey == '8' || customKey == '9' || customKey == '0' || customKey == '<' || customKey == '.' || customKey == 'C' || customKey == 'S' || customKey == 'E'){
  transientType();                                                      //ignore other keys
      }
  lcd.clear();

  if (exitMode == 0){                                       //if Transient Mode type selected, go to Transient Mode
     transientMode();
      }
 }

//----------------------------------------Transient--------------------------------------------
void Transient (void) {
  
  if(Mode == "TC" || Mode == "TL"){
  lcd.noCursor();                                         //switch Cursor OFF for this menu 
  lcd.setCursor(0,0);
  lcd.print("DC LOAD");
  
  if(Mode == "TC"){
  lcd.setCursor(0,2);
  lcd.print("Lo=");
  lcd.setCursor(3,2);
  lcd.print(LowCurrent,3);
  lcd.setCursor(8,2);
  lcd.print("A");
  lcd.setCursor(11,2);
  lcd.print("Hi=");
  lcd.setCursor(14,2);                                    
  lcd.print(HighCurrent,3);
  lcd.setCursor(19,2);
  lcd.print("A");
  lcd.setCursor(0,3);                                     
  lcd.print("Time = ");
  lcd.setCursor(7,3);
  lcd.print(transientPeriod);
  lcd.setCursor(12,3);                                    
  lcd.print("mSecs");
    }else{
     lcd.setCursor(0,3);
     lcd.print("  ");
    }
  }
  }

//-------------------------------------Transcient List Setup-------------------------------------------
void transientListSetup() {
  lcd.noCursor();                                                 
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Setup Transient List");
  lcd.setCursor(0,1);
  lcd.print("Enter Number in List");
  lcd.setCursor(0,2);
  lcd.print("(between 2 to 10 max"); 
  y = 0;
  z = 0;
  r = 3;
  inputValue(2);
  if (x > 10 || x < 2) {
    transientListSetup();
  }
  total_instructions = int(x-1);
  lcd.clear();
  
    for(int i=0; i<=(total_instructions); i++){
      lcd.setCursor(0,0);
      lcd.print("Set Current ");
      lcd.print(i+1);
      lcd.setCursor(16,1);
      lcd.print("A");
      y = 0;
      z = 0;
      r = 1;
      inputValue(2);            //get the users input value
      transientList[i][0] = x; //store the users entered value in the transient list 
      lcd.setCursor(0,2);
      lcd.print("Set Time ");
      lcd.print(i+1);
      lcd.setCursor(16,3);
      lcd.print("mSec");
      y = 0;
      z = 0;
      r = 3;
      inputValue(5);            //get the users input value
      transientList[i][1] = x; //store the users entered value in the transient list
      lcd.clear();   
  }
  current_instruction = 0;      //start at first instrution
  }

//-------------------------------------Transcient Load Toggel-------------------------------------------
void transientLoadToggle() {

  if(Mode == "TC"){
  current_time = micros();                              //get the current time in micro seconds()
  if (last_time == 0){
    last_time = current_time;
  } else {
      switch (transient_mode_status){
        case (false):
          // we are in the low current setting
          if ((current_time - last_time) >= (transientPeriod * 1000.0)){
             transientSwitch(LowCurrent, true);  
          }
        break;
        case (true):
          // we are in the high current setting 
          if ((current_time - last_time) >= (transientPeriod * 1000.0)){
            transientSwitch(HighCurrent, true);            
          }
        break; 
      } 
    }
  }

  if(Mode == "TL"){
    if (Load == 1){                                        // Only perform Transient List if Load is ON
      current_time = micros();                              //get the current time in micro seconds()
      if (last_time == 0){
      last_time = current_time;
      transientPeriod = transientList[current_instruction][1];   //Time data for LCD display
      transientSwitch(transientList[current_instruction][0], false);
      }
      if((current_time - last_time) >= transientList[current_instruction][1] * 1000){     //move to next list instruction
        current_instruction++;
        if(current_instruction > total_instructions){
          current_instruction = 0;
        }
        transientPeriod = transientList[current_instruction][1];   //Time data for LCD display
        transientSwitch(transientList[current_instruction][0], false);
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















