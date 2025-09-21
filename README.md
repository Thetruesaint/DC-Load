## v1.72 ## WHATS NEXT!

**Mejoras:**
- lecturas de EEPROM "nan" son inicializadas a 0 o a 1
- Rediseño de funciones Load y Save Calibration, mas eficientes.

**Fixes:**
- Queme los MOSFET y lo cambie por IRF3205. Además, modifique HW (ADC de sensor de Curr sin Opamp seguidor y R 1k antes del pin2 del Opamp)
- El encoder, ya no tiene asceleración, era confuso, lo saque en alguna versión anterior (v1.69 tal vez).
- Cambie la ecuación de maxpwrdis para los nuevos MOSFET
- La simulación cargaba los limites de EEPROM (todos 1) pisando los default que se inicializan en Main
- Corregido: A veces mostraba 10.000v o 10.000a y quedaba la v y la a doble en el LCD.

**Bugs:**
- Config_Limits puede ser llamada a si misma varias veces.
- Mientras este en modo CA, no voy a ver el resultado de la calibración.
- Calib. Current en P2 deriva la corriente por el R Shunt pedorro. Buscar uno bueno.
- Si entro a Calibrar V o I pero cancelo, quedan los factores default cargados
- Cuando se encienden los FANs se consumen 15mA de mas, incluso con LOAD OFF. Puede ser tema de Masas, ver cambio de PCB

**Trabajando:**
- Liberando espacio y eficiencia.

**A Trabajar:**

**En Cola:**
- Hacer un solo menu de configuracion para Limits y Calibación!! como en Fuente Lineal
- Dejar solo TL?.. TC se puede hacer con TL.

**Posibles Mejoras SW:**

- Revisar la lógica del encoder, ya no se requiere asceleración y usa dos identaciones para avanzar.
- En CC, CP, CR y BC, con BTN encoder habilito cambiar de valor o de unidad para --CuPo, reqiuere reingenieriaa Cursor_Position
- Colocar un indicador de Shift? salvo BC, hay lugar en 20,3
- En TL poner "-" por cada step y marcar en cual se esta ej.: ---3----- -> ----4----
- En TC y TL: mostrar mSec decrecientes?
- En TC, ajustar timing con encoder?
- En CP y CR: Recalcular los limites de W y R en funcion de la DC presente?..
- Healt Checks en setup e informar errores con periféricos I2C, temp., etc. llamar a Check_Limits?
- Activar el LOAD ON OFF por interrupción?
-  Setear hora y fecha del RTC y poder mirarla boton Shift?: rtc.adjust(DateTime(F(__DATE__), F(__TIME__))); //
- EL RTC es un DS1307 de MAXIM y cuenta con una EEPROM AT24C32 de ATMEL. Ver de aprovechar esta memoria.

**Posibles Mejoras de HW:**
- Medir frecuencia máxima TC y TL con el osciloscopio a ver hasta donde llega, usar Lipo.
- Habilitar control externo de MOSFETs?
- Cambiar teclado a 4x5 y teclas con extrusor de 0.2mm
- Cambiar la PCB y ver otros proyectos
- Reg. de 4.096V para ADC.
- R Shunt con buen coheficiente de temperatura.