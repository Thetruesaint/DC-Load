## v2.12 ## NICE TO SEE YOU

**Trabajando:**
- Introduciendo mejoras con TFT

**A Trabajar:**
- Setear hora y fecha del RTC: rtc.adjust(DateTime(F(__DATE__), F(__TIME__))); y que no se pise con cada update de FW.
- Seguir optimizando la arquitectura

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

**Posibles Mejoras SW:**
- Unificar template para cuando se supera mas de un límite.
- Menu de configuración ampliado (limites de descarga de baterias por ej.)
- En TC y TL: mostrar mSec decrecientes?
- En CP y CR: Recalcular los limites de W y R en funcion de la DC presente?..
- EL RTC es un DS1307 de MAXIM y cuenta con una EEPROM AT24C32 de ATMEL. Ver de aprovechar esta memoria.
- Promediar los valores de a y v para que no cambien tanto y se deba refrescar continuamente

**Posibles Mejoras de HW:**
- Medición de baterias por celda 
- Medir frecuencia máxima TC y TL con el osciloscopio a ver hasta donde llega, usar Lipo.
- Habilitar control externo de MOSFETs?
- Reg. de 4.096V para ADC?
- R Shunt con buen coheficiente de temperatura.