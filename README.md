## v1.72

**Mejoras:**

**Fixes:**

**Bugs:**
- Config_Limits puede ser llamada a si misma varias veces.
- Mientras este en modo CA, no voy a ver el resultado de la calibración.
- A veces lee 10.000v y queda la v en el LCD.
- Calib. Current en P2 deriva la corriente por el R Shunt pedorro. Buscar uno bueno.

**Trabajando:**

**A Trabajar:**

No mas Storage, liberar.

**En Cola:**

**Posibles Mejoras SW:**

- Revisar la lógica del encoder, ya no se requiere asceleración y usa dos identaciones para avanzar.
- En CC, CP, CR y BC, con BTN encoder habilito cambiar de valor o de unidad para --CuPo, reqiuere reingenieriaa Cursor_Position
- Colocar un indicador de Shift? salvo BC, hay lugar en 20,3
- En TL poner "-" por cada step y marcar en cual se esta ej.: ---3----- -> ----4----
- En TC y TL: mostrar mSec decrecientes?
- En TC, ajustar timing con encoder?
- En CP y CR: Recalcular los limites de W y R en funcion de la DC presente?..
- Heald Checks en setup e informar errores con periféricos I2C, temp., etc. llamar a Check_Limits?
- Activar el LOAD ON OFF por interrupción?
-  Setear hora y fecha del RTC y poder mirarla boton Shift?.
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__))); //
- EL RTC es un DS1307 de MAXIM y cuenta con una EEPROM AT24C32 de ATMEL. Ver de aprovechar esta memoria.

**Posibles Mejoras de HW:**
- Medir frecuencia máxima TC y TL con el osciloscopio a ver hasta donde llega, usar Lipo.
- habilitar control externo de MOSFETs?
