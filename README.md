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

  Fixes:
    - ##CRITICO## DAC Control con !toggle no ponia setCurrent = 0; al cambiar de modo y darle ON seteba la corriente anterior por un momento.
    - Cambie limite del Read_Encoder, encoderMax = 10000 solo para CC y BC, antes afectaba a CP y CR
    - Calculo mAh era erroneo, asumia que la corriente era cte. Pase a hacer una integración con la I en curso
 
  Bugs:
    - Se pueden ingresar tiempos mSec con punto decimal, ver efecto.
       
  Trabajando:
    - Optim: Limitar a reading/encoder por modo por variable global en cada set mode.
  
  En Cola:
    - Uso para Shift+C para ir al un nuevo modo: CALIBRACION (MILSTONE)
  
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
