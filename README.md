## v2.12 ## NICE TIO SEE

## Hardware Versions

- v2.x → ESP32 (Plataforma actual)

**Trabajando:**
- Test en HW de nueva placa de control de Fans y menu de Fans

**A Trabajar:**
- Introducir mejoras en TFT
- Si ISNS es mayor que ISET no se debería apagar la carga e indicar error?
- Evaluando cambio de pines para poder controlar apagado de MOSFETs
- Entrar en modo update de firmware
- En TL los steps x/t empiecen contando de 1 y no de 0 como el index de la lista

**Bugs**
- Falta símbolo de "grado" centigrado en la temperatura y sigo de "ohms" en el modo CR
- Las advertencias de Limites maximos superados pisan el template en el quee se encuentre en ese momento.
- CRITICO: No poner carga cuando el USB esta conectado porque el DAC no puede controlar los MOSFET y quedan en corto.

**Fixes**
- 

**Mejoras**
- Agregue una placa aparte para el control de FANs
- Se agrega Menu para Test, y la 1era opción para ON/OFF de Fans.

**Posibles Mejoras SW:**
- Control de velocidad de Fans o test de encendido y apagado
- Menu de configuración ampliado (limites de descarga de baterias por ej.)
- En CC, CP, CR y BC, con BTN encoder habilito cambiar de valor o de unidad para --CuPo, reqiuere reingenieriaa Cursor_Position
- Colocar un indicador de Shift? salvo BC, hay lugar en 20,3
- En TL poner "-" por cada step y marcar en cual se esta ej.: ---3----- -> ----4----
- En TC y TL: mostrar mSec decrecientes?
- En TC, ajustar timing con encoder?
- En CP y CR: Recalcular los limites de W y R en funcion de la DC presente?..
- Setear hora y fecha del RTC y poder mirarla boton Shift?: rtc.adjust(DateTime(F(__DATE__), F(__TIME__))); //
- EL RTC es un DS1307 de MAXIM y cuenta con una EEPROM AT24C32 de ATMEL. Ver de aprovechar esta memoria.

**Posibles Mejoras de HW:**
- Medición de baterias por celda 
- Medir frecuencia máxima TC y TL con el osciloscopio a ver hasta donde llega, usar Lipo.
- Habilitar control externo de MOSFETs?
- Reg. de 4.096V para ADC?
- R Shunt con buen coheficiente de temperatura.
