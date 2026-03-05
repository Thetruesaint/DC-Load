## v2.01 ## CALIBRATIOS BUG FIX

## Hardware Versions

- v2.x → ESP32 (Plataforma actual)
- v1.x → Arduino Nano (legacy - ver rama nano-legacy)

**Trabajando:**
- Simulacion WOKWI, haciendo que funcione con LCD, el teclado funciona pero responde muy lento
- Usando Codex para mejorar todo el código y modularizarlo para poder trabajar partes independientemente.

**A Trabajar:**
- Usar Codex para mejorar todo el código y modularizarlo para poder trabajar partes independientemente. 
    
**En Cola:**

- Hacer un solo menu de configuracion para Limits y Calibación!! como en Fuente Lineal

**Bugs**
- La Calibración de corriente no funciona bien, queda con un offset mayor a 0 y mide mas de lo seteado.
- En OFF, el ADC registra 6mA pero no hay consumo. A veces por algun motivo que desconozco, consume 24mA de la fuente, pero no es siempre.

**Fixes**
- Corrección de calibración del Out_Curr_Calib_Offs ya que se sumaba en amperes y no se le aplicaba el OUT_CURR_FACT para el seteo del DAC. Se ajusto tambien el almacenamiento en EEPROM para este factor y su rango de validación
- Se agregaron protecciónes para los puntos de calibración. 

**Mejoras**
- Se pasaron todas las funciones "lcd." a ui_lcd lo mismo para el manejo de EEPROM

**Posibles Mejoras SW:**
- Dejar solo TL?.. TC se puede hacer con TL.
- Control de velocidad de Fans.
- Menu de configuración ampliado (limites de descarga de baterias por ej.)
- En CC, CP, CR y BC, con BTN encoder habilito cambiar de valor o de unidad para --CuPo, reqiuere reingenieriaa Cursor_Position
- Colocar un indicador de Shift? salvo BC, hay lugar en 20,3
- En TL poner "-" por cada step y marcar en cual se esta ej.: ---3----- -> ----4----
- En TC y TL: mostrar mSec decrecientes?
- En TC, ajustar timing con encoder?
- En CP y CR: Recalcular los limites de W y R en funcion de la DC presente?..
- Healt Checks con beeps si encuentra errores
- Activar el LOAD ON OFF por interrupción?
- Setear hora y fecha del RTC y poder mirarla boton Shift?: rtc.adjust(DateTime(F(__DATE__), F(__TIME__))); //
- EL RTC es un DS1307 de MAXIM y cuenta con una EEPROM AT24C32 de ATMEL. Ver de aprovechar esta memoria.

**Posibles Mejoras de HW:**
- Pasar de LCD a TFT
- Medición de baterias por celda 
- Medir frecuencia máxima TC y TL con el osciloscopio a ver hasta donde llega, usar Lipo.
- Habilitar control externo de MOSFETs?
- Teclas con extrusor de 0.2mm
- Reg. de 4.096V para ADC.
- R Shunt con buen coheficiente de temperatura.
