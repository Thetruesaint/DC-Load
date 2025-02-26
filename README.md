## v1.70 25/02/2025 ## MAJOR RELEASE ##

**Mejoras:**
- En BC reemplace W por voltageCuttOff, es mas útil y elimino el pantallazo de Cuttoff Voltage
- En BC dejo min y seg. Agrego tipo de bateria
- Optimizo Temp Control y el disp. de TEmperatura, Cbio. a "Set X>", realineando el input de I, W y R.
- Shift +1 a 6 salto directo al modo. Shift + < o tecla no valida, resetea el modo. 
- En menus de Setup o Config, Value_Input, shift + tecla no valida, reinicia el mismo modo también. 
- Value_Imput ahora es default con 5 digitos.
- elimino exitMode global, ya no se necesita.
- En TC y TL: usar instrucciones del 0 al 9, mas simple
- Paso a const variable de cell Voltage de las baterias y el factor de controlcoltage.
- const float MAX para limites, asig. a cuttoffs que son int (ojo!). Constrain en Config_Limits con mínimos y esos MAX.
- CC, CR, CP y BC Limite a reading/encoder por variable global. Check_limit quedo solo para los de Hardware.
- CR y CP con ajustes de decenas, centenas y solo decimas. CC y BC se alinean con las unidades (unitPosition = 8), ajusto Update_LCD
- Optimización de Cursor_Position. 
- En BC Setup ahora se puede llamar a Config_Limits.
- Cambia TL con "Step" es mas tradicional y claro, reoganizo plantillas LCD
- A, V y W en minusculas para valores dinámicos, y A con un caracer especial mas petiso que se carga en el LCD por Setup.
- En TC y TL se deshabilita el encoder. Nueva función Encoder_Status() que inicializa tambien limites de reading y Encoder
- Blink de unidad de limite excedido
- Optimización de espacio con funcion Print_Spaces.

**Fixes:**
- ##CRITICO## DAC Control con !toggle no ponia setCurrent = 0; al cambiar de modo y darle ON seteba la corriente anterior por un momento.
- Cambie limite del Read_Encoder, encoderMax = 10000 solo para CC y BC, antes afectaba a CP y CR
- Calculo mAh era erroneo, asumia que la corriente era cte. Pase a hacer una integración con la I en curso
- Movi el display de I, R y W y tuve que rediseñar Cursor_Position ojo con esto.
- Read_Keypad, con E, carga el valor 0, se ignora si index = 0.
- Variables cutoff de int a float, redireccionamiento EEPROM.
- CR no limitaba el 0 ingresado por teclado.
- CuPo no inicializaba al cambiar de modo.
- En BC quedaba en 100mA cuando termina. Se retea los input cuando "Done"
- No se veia bien el cursor, cambio método a blinking manual LCD_RFSH_TIME
- Al salir de TL Setup, quedaba el blink.
- En BC, Setup solo salir con M, con Shit no respondia
- Value Imput permite deshabilitar numeros decimales (mSec de TC y TL o BC selec. de n°S)
- En CC y BC, en la unidad del Set quedaba tapada
- En TC y TL se podia modificar Config_Limits mal el condicional de Read_Keypad. 
- Read_Keypad permitia 5 digitos mas el . que luego no se borraba. Faltaba condicional en el "."

**Bugs:**
- No detectados por ahora
  
**Trabajando:**
- Relese Terminado!

**A Trabajar:**
- No mas para v1.70.

**En Cola: (next Release)**
- No convertir ADC por rango de voltage, complicado al pedo... y peligroso. NOs ahorramos FLASH
- Uso para Shift+C para ir al un nuevo modo: CALIBRACION (MILSTONE) para V1.71
- Modo Calibracíón incluir external voltage sense?
- Revisar la lógica del encoder, ya no se requiere asceleración y usa dos identaciones para avanzar.
- Unificar Read_Keypad e Value_Input? para que tome valores instantaneos.

**Posibles Mejoras SW:**

- En CC, CP, CR y BC, con BTN encoder habilito cambiar de valor o de unidad para --CuPo, reqiuere reingenieriaa Cursor_Position
- Colocar un indicador de Shift? salvo BC, hay lugar en 20,3
- En TL poner "-" por cada step y marcar en cual se esta ej.: ---3----- -> ----4----
- En TC y TL: mostrar mSec decrecientes?
- En TC, ajustar timing con encoder?
- En CP y CR: Recalcular los limites de W y R en funcion de la DC presente?..
- Heald Checks en setup e informar errores con periféricos I2C, temp., etc. llamar a Check_Limits?
- Anuncio de limite excedido, hacer "blimk" del valor excedido.
- Activar el LOAD ON OFF por interrupción?
-  Setear hora y fecha del RTC y poder mirarla boton Shift?.
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__))); //
- EL RTC es un DS1307 de MAXIM y cuenta con una EEPROM AT24C32 de ATMEL. Ver de aprovechar esta memoria.

**Posibles Mejoras de HW:**
- Cooler mas grande contra el Disipador.
- Buzzer
- Medir frecuencia máxima TC y TL con el osciloscopio a ver hasta donde llega, usar Lipo.
- Mosfets para coolers
- E Control para apagar los MOSFET si falla el DAC
- habiliar control externo de MOSFETs?
