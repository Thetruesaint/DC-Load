## v2.13 ##

**Trabajando:**


**A Trabajar:**
- Seguir optimizando la arquitectura

**Bugs**
- Pixeles remanentes cuando a y v se reacomodan por los dígitos

**Fixes**
- indicacion de "sf" en menues de configuración

**Mejoras**
- Mejoras esteticas en setups de TC, TL y Menues de Configuración
- ya se puede setear fecha y hora del RTC.
- Indicador de ON en blanco y Temp con grado de color de amarillo al rojo al incrementarse.

**Posibles Mejoras SW:**
- Unificar template para cuando se supera mas de un límite.
- Menu de configuración ampliado (limites de descarga de baterias por ej.)
- En TC y TL: mostrar mSec decrecientes?
- En CP y CR: Recalcular los limites de W y R en funcion de la DC presente?..
- EL RTC es un DS1307 de MAXIM y cuenta con una EEPROM AT24C32 de ATMEL. Ver de aprovechar esta memoria.
- Promediar los valores de a y v para que no cambien tanto y se deba refrescar continuamente
- Graficar Y/t para mostrar el historico de a y v.

**Posibles Mejoras de HW:**
- Medición de baterias por celda 
- Medir frecuencia máxima TC y TL con el osciloscopio a ver hasta donde llega, usar Lipo.
- Habilitar control externo de MOSFETs?
- Reg. de 4.096V para ADC?
- R Shunt con buen coheficiente de temperatura.
