**DC LOAD based in a desing from Mr Louis Scully**

## v2.00 ## THE ESP32 HAS LANDED

**Fixes**
- En BC ahora las teclas de dirección funcionan

**Mejoras**
- Comence a usar Codex para mejorar el códgo, separando funciones de UI_LCD y STORAGE_EEPROM


## v2.00b ## AN ALL NEW WORLD

## Hardware Versions

- v2.x → ESP32 (Plataforma actual)
- v1.x → Arduino Nano (legacy - ver rama nano-legacy)

**Mejoras**
- SW migrado a nueva placa de control basada en ESP32
- Nueva placa de Potencia, Separe la alimentación de los Coolers, sino meten ruido.
- Nuevo parámetros de protección para IRFP250N
- El ESP32 o el LCD se cuelga a veces con el encendido de +-5V o sensado de tensión. Si se enciende parejo no hay problema
- Voltages de Celda para descargas de Baterias: LIPO_DISC_CELL_VLTG = 3.6 y LION_DISC_CELL_VLTG = 3.5
- Teclado 5x4 con teclar de dirección para mover cursor y subir o bajar valor 
- Remapeo de pines y ajuste de funciones de Temp_Control(), Read Encoder() y Encoder_Estatus() por la nueva libreria pero ver el tema del detach y attach porque tilda el ESP32
- Agrego Healt check en setup para informar errores con periféricos I2C, temp., etc.
- Tuve que modifica la libreria de Keypad para poder usar los pines que elegi... invirtiendo los que leen y los que escriben
- Se actualizo funciones de EEPROM en real para el ESP32. 
- Integre el LCD y funciona OK, pruebas con  el TFT pero en paralelo. EL LCD funciona solo con 5V.
- Botones de dirección, up down, izq. der.
- Estaba mal seteada la ganancia del AD3 de corriente y se iba de escala, se pone a dos tercios por las dudas. Se prueban sensado de voltajes y corrientes máximas simuladas.

**Fixes**
- Tuve que cambiar los pines del teclado para que funcionen.
- En placa de Potencia independice la alimentación de los Coolers, sino meten ruido.
