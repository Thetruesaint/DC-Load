## v2.11 ## BY LCD HELLOW TFT & AI

## Hardware Versions

- v2.x → ESP32 (Plataforma actual)
- v1.x → Arduino Nano (legacy - ver rama nano-legacy)

**Trabajando:**
- Usando Codex para mejorar todo el código y modularizarlo para poder trabajar partes independientemente.

**A Trabajar:**
- Continuar migracion de arquitectura para luego introducir mejoras en TFT
    
**En Cola:**
- 

**Bugs**
- Falta símbolo de "grado" centigrado en la temperatura y sigo de "ohms" en el modo CR

**Fixes**
- Corrección de calibración del Out_Curr_Calib_Offs ya que se sumaba en amperes y no se le aplicaba el OUT_CURR_FACT para el seteo del DAC. Se ajusto tambien el almacenamiento en EEPROM para este factor y su rango de validación
- Se agregaron protecciónes para los puntos de calibración. 

**Mejoras**
- Se pasaron todas las funciones "lcd." a ui_lcd lo mismo para el manejo de EEPROM
- En OFF, el ADC registra 6mA y consumia 24mA de la fuente. Agregue un op amp con offset para corregir el offset del DAC que hacia que los mosfet conducieran con Set I = 0A.
- Vuelve a funcionar el teclado en sim WOKWI, puse resistencias de pullup en GPIO 34 y 35.
- Adios LCD, hola TFT!. Por ahora solo simula la grilla de 20x4 del LCD
- IMPORTANTE: Migración a nueva arquitectura asistido por CODEX
- Nuevo esquema de Menues para Calibración y Protección que incluye a Limites y seteos para el Fan.

**Posibles Mejoras SW:**
- Dejar solo TL?.. TC se puede hacer con TL.
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
