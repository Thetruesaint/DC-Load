## v1.70 23/02/2025

 Mejoras:
  - En BC reemplace W por voltageCuttOff, es mas útil y elimino el pantallazo de Cuttoff Voltage
  - En BC dejo min y seg. Agrego tipo de bateria
  - Optimizo Temp Control y el disp. de TEmperatura, Cbio. a "Set X>", realineando el input de I, W y R.

  Fixes:
  - Cambie limite del Read_Encoder, encoderMax = 10000 solo para CC y BC, antes afectaba a CP y CR
  - Calculo mAh era erroneo, asumia que la corriente era cte. Pase a hacer una integración con la I en curso
 
  Bugs detectados:
  - Ningunos por el momento
       
  Trabajando:
  - Optim: Limitar a reading/encoder por modo por variable global en cada set mode?
  - Shift + Modo, resetea el modo? o shift + < va para atras en la config?
  - Probe descarga de BC y medir mAh: Descargo 1126mAh cargo luego 1180mAh 
  
  En Cola:

  - En modos TC y TS mostrar mSec decrecientes?

  Posibles Mejoras:
  - Ponerle un Buzzer?
  - Modo Calibracíón incluir external voltage sense?
  - Con anuncio de limite exedido, actualizar el limite excedido y parpadearlo
  - Uso para Shift paa ir a un modo directo o a Calibración S+C?
  - Activar el LOAD ON OFF por interrupción?, desactivar el mosfet con el procesador directamente?
  - Ver la frecuencia máxima de conmutación de los Trasient y limitarla a esa
  - Setear hora y fecha del RTC y poder mirarla boton Shift?.
      rtc.adjust(DateTime(F(__DATE__), F(__TIME__))); // El módulo está basado en el RTC DS1307 de MAXIM y cuenta con una EEPROM AT24C32 de ATMEL. ver uso posible
  - Recalcular los limites de W y R en funcion de la DC presente?..
  - Ajustar timing con encoder en TC mode?
  - Ver de agregar Heald Checks antes de inicializar e informar error de detectarse.
