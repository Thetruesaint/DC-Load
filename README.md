## v2.0 ## AN ALL NEW WORLD

**Trabajando:**
- Simulacion WOKWI: no funciona con el TFT se complica con el real.
- Resta pruebas adicionales de menues y ajustar las funciones relacionadas con el encoder a ver si generan algun bug (nueva librería) e integrar placa de potencia.
- Armar y probar placa de potencia


**A Trabajar:**

    
**En Cola:**
- Hacer un solo menu de configuracion para Limits y Calibación!! como en Fuente Lineal
- Dejar solo TL?.. TC se puede hacer con TL.

**Mejoras**
- Nueva placa de Potencia, pero tengo que separar la alimentación de los Coolers, meten ruido.
- Nuevo parámetros de protección para IRFP250N
- El ESP32 o el LCD se cuelga a veces con el encendido de +-5V o sensado de tensión. Si se enciende parejo no hay problema
- Voltages de Celda para descargas de Baterias: LIPO_DISC_CELL_VLTG = 3.6 y LION_DISC_CELL_VLTG = 3.5
- Migrando a ESP32
- Teclado 5x4 con teclar de dirección para mover cursor y subir o bajar valor 
- Remapeo de pines y ajuste de funciones de Temp_Control(), Read Encoder() y Encoder_Estatus() por la nueva libreria pero ver el tema del detach y attach porque tilda el ESP32
- Agrego Healt check en setup para informar errores con periféricos I2C, temp., etc.
- Tuve que modifica la libreria de Keypad para poder usar los pines que elegi... invirtiendo los que leen y los que escriben
- Se actualizo funciones de EEPROM en real para el ESP32. 
- Integre el LCD y funciona OK, pruebas con  el TFT pero en paralelo. EL LCD funciona solo con 5V.
- Botones de dirección, up down, izq. der.
- Estaba mal seteada la ganancia del AD3 de corriente y se iba de escala, se pone a dos tercios por las dudas. Se prueban sensado de voltajes y corrientes máximas simuladas.


**Posibles Mejoras SW:**
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
- Medición de baterias por celda 
- Medir frecuencia máxima TC y TL con el osciloscopio a ver hasta donde llega, usar Lipo.
- Habilitar control externo de MOSFETs?
- Teclas con extrusor de 0.2mm
- Reg. de 4.096V para ADC.
- R Shunt con buen coheficiente de temperatura.
