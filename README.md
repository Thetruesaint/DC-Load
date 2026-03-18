## v2.12 ## NICE TO SEE

**Trabajando:**
- Introduciendo mejoras con TFT

**A Trabajar:**
- Migrar menues y pantallas iniciales a TFT para terminar de migrar lo legacy
- Setear hora y fecha del RTC: rtc.adjust(DateTime(F(__DATE__), F(__TIME__))); y que no se pise con cada update de FW.

**Bugs**
- Pixeles remanentes cuando a y v se reacomodan por los dígitos

**Fixes**
- CRITICO: Cuando el USB estaba conectado porque el DAC no podia controlar los MOSFET y quedaban en corto. Parece que se soluciono con MOSFONOFF HIGHT. VERIFICAR BIEN
- IMPORTANTE: Corrección en la aplicación de la calibración en el SetCurrent: calibratedCurrent = (targetCurrent - Out_Curr_Calib_Offs) / outputFactor;
- En BC, TC y TL ahoira se detecta el salto de modo con S+N

**Mejoras**
- Indicador de shift pressed
- En TL los steps x/t empiezan contando de 1 y no de 0.
- En TC se puede reajustar el periodo con teclado o encoder.
- Nuevos templates TFT para pantalla de inicio, modos CC, CP, CR, BC, TC, TL, CA y advertencias!!
- Nueva simulacion WOKWI con TFT 240x320
- Nueva placa aparte provisoria para el control de FANs se integrará en la nueva versión de PCB v2.3
- Se agrega opción para ON/OFF de Fans en Fans Settings
- Reasignación de GPIOs para poder tener control de MOSFETs independiente del DAC: FAN_CTRL = 16, LOADONOFF = 39 y MOSFONOFF = 25. Usare MOSFET de placa de control para Poner Iset a GND, agregue resistencia 1k en serie.
- Si MeasuredCurrent > SetCurrent, se advierte "RunOut Cutt Off!" y ahora Carga apagada = MOSFONOFF HIGHT
- Nueva pantallas de Protección para informar de limites superados.
- **OTA FW Update!:**
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
