## v1.71

**Mejoras:**
- Optimización de Read_Keypad.

**Fixes:**

**Bugs:**
- No detectados
  
**Trabajando:**
- Optimización de Read_Keypad y Value_Input. Ahorro FLASH

**A Trabajar:**
- No convertir ADC por rango de voltage, complicado al pedo... y peligroso. Ahorro FLASH
- Cambiar HW para aumentar presición, no es util el conversor actual.
- Uso para Shift+C para ir al un nuevo modo: CALIBRACION (MILSTONE)

**En Cola: (next Release)**
- Modo Calibracíón incluir external voltage sense?
- Revisar la lógica del encoder, ya no se requiere asceleración y usa dos identaciones para avanzar.

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
