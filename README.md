## v1.70 24/02/2025

 Mejoras:
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
  
  Fixes:
    - ##CRITICO## DAC Control con !toggle no ponia setCurrent = 0; al cambiar de modo y darle ON seteba la corriente anterior por un momento.
    - Cambie limite del Read_Encoder, encoderMax = 10000 solo para CC y BC, antes afectaba a CP y CR
    - Calculo mAh era erroneo, asumia que la corriente era cte. Pase a hacer una integración con la I en curso
    - Movi el display de I, R y W y tuve que rediseñar Cursor_Position ojo con esto.
 
  Bugs:
    - Check_limit no sirve para limitar entradas de encoder que setea por interrupciones.
    - Variables cutoff declaradas como int, y mal direccionadas en la EEPROM. Se pasan a float y se cambia el direccionamiento
    - Dentro de TL ejecutando se, no puedo salir con Shift+M, solo con M.
    - En BC, menu inicial solo se pude salir con M, con Shit no funciona
    - Se pueden ingresar tiempos mSec con punto decimal, ver efecto.
       
  A Trabajar:
     - Limitar a reading/encoder por variable global en cada set mode. Check_limit que quede solo para los de Hardware
   
  En Cola:
    - Uso para Shift+C para ir al un nuevo modo: CALIBRACION (MILSTONE)
    - En modos TC y TL Deshabilitar interrupción de encoder? afectara tiempos cortos..
  
  Posibles Mejoras:
    - En TC y TL: mostrar mSec decrecientes?, usar instrucciones del 0 al 9, mas simple
    - En TC, ajustar timing con encoder?
    - En CP y CR: Recalcular los limites de W y R en funcion de la DC presente?..
    - Heald Checks en setup e informar errores con periféricos I2C, temp., etc. llamar a Check_Limits?
    - Ponerle un Buzzer?
    - Modo Calibracíón incluir external voltage sense?
    - Anuncio de limite excedido, hacer "blimk" del valor excedido.
    - Activar el LOAD ON OFF por interrupción?, desactivar el mosfet con el procesador directamente?
    - Ver la frecuencia máxima de conmutación de los Trasient con el osciloscopio a ver hasta donde llega.
    - Setear hora y fecha del RTC y poder mirarla boton Shift?.
        rtc.adjust(DateTime(F(__DATE__), F(__TIME__))); //
    - EL RTC es un DS1307 de MAXIM y cuenta con una EEPROM AT24C32 de ATMEL. Ver de aprovechar esta memoria.
