## v2.12 ## NICE TO SEE

**Trabajando:**
- Introduciendo mejorar con TFT

**A Trabajar:**
- En TL los steps x/t empiecen contando de 1 y no de 0 como el index de la lista
- Setear hora y fecha del RTC: rtc.adjust(DateTime(F(__DATE__), F(__TIME__))); y que no se pise con cada update de FW.

**Bugs**
- Pixeles remanentes cuando a y v se reacomodan por los dígitos
- En otros modos que no sean CC, CP y CR el cursor se indica mal pero se corregira al convetir los que falten.

**Fixes**
- CRITICO: Cuando el USB estaba conectado porque el DAC no podia controlar los MOSFET y quedaban en corto. Parece que se soluciono con MOSFONOFF HIGHT. VERIFICAR BIEN
- IMPORTANTE: Corrección en la aplicación de la calibración en el SetCurrent: calibratedCurrent = (targetCurrent - Out_Curr_Calib_Offs) / outputFactor;

**Mejoras**
- Nuevos templates TFT para modos CC, CP, CR y BC!!
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
- Menu de configuración ampliado (limites de descarga de baterias por ej.)
- En CC, CP, CR y BC, con BTN encoder se podria hacer algo
- Colocar un indicador de Shift?
- En TL indicar cada step y marcar en cual se esta ej.: ---3----- -> ----4----
- En TC y TL: mostrar mSec decrecientes?
- En TC, ajustar timing con encoder?
- En CP y CR: Recalcular los limites de W y R en funcion de la DC presente?..
- EL RTC es un DS1307 de MAXIM y cuenta con una EEPROM AT24C32 de ATMEL. Ver de aprovechar esta memoria.

**Posibles Mejoras de HW:**
- Medición de baterias por celda 
- Medir frecuencia máxima TC y TL con el osciloscopio a ver hasta donde llega, usar Lipo.
- Habilitar control externo de MOSFETs?
- Reg. de 4.096V para ADC?
- R Shunt con buen coheficiente de temperatura.
