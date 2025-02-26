#include "variables.h"
#include "funciones.h"

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

//-------------------------------------Configuraciones iniciales de única vez-----------------------------
// dac.setVoltage(0,true);                      // reset DAC to zero for no output current set at Switch On, Cambio a "True" para que guarde este valor en la Emprom

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
lcd.setCursor(0, 1); lcd.print(date + " - " + time);
#ifndef WOKWI_SIMULATION
printLCD(0, 2, F("By Guy Nardin"));
#else
printLCD(0, 2, F("Para SIMULACION"));
#endif
printLCD(0, 3, F("v1.70b")); // Version Beta para testear en equipo Real

#ifndef WOKWI_SIMULATION
delay (2000);
#else
delay(500);     //Para probar mas rapido
#endif

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
}

//------------------------------------- Bucle Principal-------------------------------------------------
void loop() {
  Temp_Control();
  Read_Keypad();
  Read_Load_Button();
  Read_Volts_Current();
  Check_Limits();
  DAC_Control();
  switch (Mode) {
    case CC:  Const_Current_Mode(); Cursor_Position(); break;
    case CP:  Const_Power_Mode(); Cursor_Position(); break;
    case CR:  Const_Resistance_Mode(); Cursor_Position(); break;
    case BC:  Battery_Mode(); Battery_Capacity(); Cursor_Position(); break;
    case TC:  Transient_Cont_Mode(); Transcient_Cont_Timing(); break;
    case TL:  Transient_List_Mode(); Transient_List_Timing(); break;
    case UNKNOWN: 
    default: break;
  }
  Update_LCD();
}
