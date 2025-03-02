## v1.71

**Mejoras:**
- Optimización de Read_Keypad Ahorro FLASH.
- Optimización de Value_Input y Funcion TC con Ahorro FLASH.
- Optimización Inicialización TC y BC y Read_Keypad con Handle_MSC_Keypad
- Reing. de ADC y DAC, reaguste de ganancias y factores de diseño. CAMBIO EN HW
- Cbio Q de Fans por MOSFET
- Buzzer en A3, Caps en VDD de DAC y ADC

**Fixes:**
- En BC Setup, Shift + "<" saltaba a TC, cambio a Handle_DSC_keys.
- Saco el E-Control no funciono bien en tests

**Bugs:**
- No detectados aún

**Trabajando:**
- Testing en HW

**A Trabajar:**
- Uso para Shift+C para ir al un nuevo modo: CALIBRACION (MILSTONE)

**En Cola: (next Release)**
- Cooler mas grande contra el Disipador.


**Posibles Mejoras SW:**

- Revisar la lógica del encoder, ya no se requiere asceleración y usa dos identaciones para avanzar.
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
- Medir frecuencia máxima TC y TL con el osciloscopio a ver hasta donde llega, usar Lipo.
- habiliar control externo de MOSFETs?
