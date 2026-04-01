## v2.14b ## CLEANING IT UP!

**Trabajando:**
- Refresco de selecciòn en menues y healt check de TFT

**A Trabajar:**
- 

**Mejoras**
- En BC tiene status de "Pause" cuando se pasa a OFF sin resetear el modo.
- Ahora los valores de a y v y w se actualizan en pantalla cada 2mA y 5mV o 10mV w es consecuencia del producto. Solo en visualización.
- Calibración: Nueva evaluación de aceptación de P1 y P2 para Out y Sense: Out: usa abs(SET - REAL) / SET y Sense: usa abs(READ - REAL) / READ comparador con margenes seteables por constantes en system_constants.h
- Calibración con mensajes de abort más claros y espera con E-Accept
- Ahora `MOSFONOFF = HIGH` queda reservado para emergencia: si falla el `health check` de arranque o si salta una protección. En operación normal, `OFF` se controla con `DAC = 0` para evitar lecturas residuales de corriente que genera el apagado forzado de los MOSFET. Tras `E-Accept` en una protección, `MOSFONOFF` vuelve a `LOW` y el sistema queda listo para continuar.
 - Si falla el `health check`, la pantalla queda detenida mostrando el fallo y no avanza el arranque.
 - Calibraciòn de Sensado de Temperatura con su menu y almacenamiento de factor en EEPROM
 - Edicion de ajuste de limites y Fan Settings con encoder y otras mejoras visuales
 
**Fixes**
- `TC` y `TL`se refrescaba todo el render con cada ingreso de valor, ahora solo zona azul
- Runout deshabilitado para SET <= 10 mA dado que no se requiere y daba falsa alarma
- En Fan Setting ahora la opción 3-FAN muestra el estado real del FAN.
- El refresco de minutos y temperatura ya no refresca toda la pantalla.
- Optimizaciòn en refresco de menues de configuracion 

**Bugs**
- Pixeles remanentes cuando a y v se reacomodan por los digitos


**Posibles mejoras de software**
- Menu de configuracion ampliado, por ejemplo limites de descarga de baterias
- En `TC` y `TL`: mostrar mSec decrecientes
- En `CP` y `CR`: recalcular los limites de `W` y `R` en funcion de la `DC` presente
- El RTC es un `DS1307` de Maxim y cuenta con una `EEPROM AT24C32` de Atmel. Evaluar aprovechar esa memoria


**Posibles mejoras de hardware**
- Medicion de baterias por celda
- Medir frecuencia maxima en `TC` y `TL` con osciloscopio, usando LiPo
- Referencia de `4.096V` para ADC
- Resistencia shunt con mejor coeficiente de temperatura

## v2.13 ## LEVELING UP!

**Mejoras**
- Adjunto Documentación y reorganizo
- Mejoras esteticas en setups de `BC`, `TC`, `TL` y menues de configuracion
- Opcion para ajustar fecha y hora del RTC
- Indicador de `ON` en blanco y temperatura con grado de color de amarillo a rojo al incrementarse
- Histograma de `a/v/t` accesible con `S+0` para `CC`, `CP`, `CR` y `BC`

**Fixes**
- Indicacion de `sf` en menues de configuracion
- `BC`, `TC` y `TL` ahora muestran `a`, `v` y `w` durante el setup

**Bugs**
- Pixeles remanentes cuando a y v se reacomodan por los digitos

## v2.12 ## NICE TO SEE YOU

**Bugs**
- Pixeles remanentes cuando a y v se reacomodan por los dígitos

**Fixes**
- CRITICO: Cuando el USB estaba conectado porque el DAC no podia controlar los MOSFET y quedaban en corto. Parece que se soluciono con MOSFONOFF HIGHT. VERIFICAR BIEN
- IMPORTANTE: Corrección de calibración en SetCurrent: calibratedCurrent = (targetCurrent - Out_Curr_Calib_Offs) / outputFactor;
- En BC, TC y TL ahora se detecta el salto de modo con S+N

**Mejoras**
- Limpieza y organización de todo el código migrando/eliminando funciones basadas en grilla LCD 20x4 y ajuste de arquictura.
- Indicador de shift pressed
- En TL los steps x/t empiezan contando de 1 y no de 0.
- En TC se puede reajustar el periodo con teclado o encoder.
- Nuevos templates TFT para pantallas de inicio, menues y modos CC, CP, CR, BC, TC, TL, CA y advertencias!!
- Nueva simulacion WOKWI con TFT 240x320
- Nueva placa aparte provisoria para el control de FANs se integrará en la nueva versión de PCB v2.3
- Opción ON/OFF de Fans en Fans Settings para probarlos
- Reasignación de GPIOs para poder tener control de MOSFETs independiente del DAC: FAN_CTRL = 16, LOADONOFF = 39 y MOSFONOFF = 25. Usos MOSFET de placa de control Power para mandar Iset a GND forzando el apagado por si falla el DAC.
- Si MeasuredCurrent > SetCurrent, se advierte "RunOut Cutt Off!" y ahora Carga apagada = MOSFONOFF HIGHT
- **FW Update por Wifi! (OTA)**
    - El menu `Configuration -> FW Update` conecta el WiFi y muestra IP/Host.
    - En powershell ejecutar:
            pio run -e real
            & "C:\Users\thetr\.platformio\penv\Scripts\python.exe" "C:\Users\thetr\.platformio\packages\framework-arduinoespressif32\tools\espota.py" -i <IP_QUE_MUESTRA_EL_TFT> -p 3232 -f ".pio\build\real\firmware.bin"

## v2.11 ## BY LCD HELLO TFT & AI

**Bugs**
- Falta símbolo de "grado" centigrado en la temperatura y sigo de "ohms" en el modo CR
- Las advertencias de Limites maximos superados pisan el template en el quee se encuentre en ese momento.
- CRITICO: No poner carga cuando el USB esta conectado porque el DAC no puede controlar los MOSFET y quedan en corto.

**Fixes**
- Corrección de calibración del Out_Curr_Calib_Offs ya que se sumaba en amperes y no se le aplicaba el OUT_CURR_FACT para el seteo del DAC. Se ajusto tambien el almacenamiento en EEPROM para este factor y su rango de validación
- Se agregaron protecciónes para los puntos de calibración. 

**Mejoras**
- En OFF, el ADC registra 6mA y consumia 24mA de la fuente. Agregue un op amp con offset para corregir el offset del DAC que hacia que los mosfet conducieran con Set I = 0A.
- Vuelve a funcionar el teclado en sim WOKWI, puse resistencias de pullup en GPIO 34 y 35.
- Adios LCD, hola TFT!. Por ahora solo simula la grilla de 20x4 del LCD
- IMPORTANTE: Migración a nueva arquitectura asistido por CODEX
- Nuevo esquema de Menues para Calibración y Protección que incluye a Limites. seteos para el Fan y visualización de los factores y offset ajustados


## v2.01 ## CALIBRATION BUG FIX

**Fixes**
- Corrección de calibración del Out_Curr_Calib_Offs ya que se sumaba en amperes y no se le aplicaba el OUT_CURR_FACT para el seteo del DAC. Se ajusto tambien el almacenamiento en EEPROM para este factor y su rango de validación
- Se agregaron protecciónes para los puntos de calibración. 

**Mejoras**
- Se pasaron todas las funciones "lcd." a ui_lcd lo mismo para el manejo de EEPROM
- En OFF, el ADC registra 6mA y consumia 24mA de la fuente. Agregue un op amp con offset para corregir el offset del DAC que hacia que los mosfet conducieran con Set I = 0A.

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
