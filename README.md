## v1.71

**Mejoras:**
- Optimización de Read_Keypad Ahorro FLASH.
- Optimización de Value_Input y Funcion TC con Ahorro FLASH.
- Optimización Inicialización TC y BC y Read_Keypad con Handle_MSC_Keypad
- Reing. de ADC y DAC, reaguste de ganancias y factores de diseño. CAMBIO EN HW
- Cbio Q de Fans por MOSFET
- Buzzer en A3, Caps en VDD de DAC y ADC
- Shift+C para ir al un nuevo modo: CALIBRACION (MILSTONE)
- GND con Cap de 1uF a Tierra, resolvio ruido de sensor de Voltage con carga apagada.
- Beeps al terminar de descargar baterias.
- Cooler adicional, mas grande contra el Disipador.
- ADC directo a medición de Shunt, cambio de Shunt Resistor a 0.05ohm. Podría ser menos aún ya que en mas 7,5A deriva.
- Rutina de Calibrado de dos puntos con offset, con lectura y grabado en EEPROM.
- Promedio Móvil Exponencial (EMA) para medición de voltage y corriente. Test en HW no convence, no veo cambios.
- Subo el FAN_ON_DRTN a 60 Seg.

**Fixes:**
- En BC Setup, Shift + "<" saltaba a TC, cambio a Handle_DSC_keys.
- Saco el E-Control no funciono bien en tests
- Calib. ok en V y en I sense. Error en formula de out I con el factor.
- Fallo Test en HW, no lee bien la EEPROM, Tuve que inicializar las posiciones de la EEPROM. Hacer esto cada cambio de Nano.
- Amplio maxDigits en Read_Keypad para precisión de calibración y saco reseteo de Calib de Read_Voltage_Current
- Saco reseteo de Calib de DAC_Control. Se hace todo en modo CA.
- No apagaba la carga despues de tomar P2.
- Calibraba el Out con mA y A. Paso todo a A.
- Calibraciòn en 10A saturo el DAC. Recalculo el factor para que contemple el out_curr_factor de 1.1
- CRR_STEP_RDCTN = 10 para que pueda bajar mas rápido la descarga si lo necesita.

**Bugs:**
- Config_Limits puede ser llamada a si misma varias veces.
- Mientras este en modo CA, no voy a ver el resultado de la calibración.
- A veces lee 10.000v y queda la v en el LCD.
- Calib. Current en P2 deriva la corriente por el R Shunt pedorro. Buscar uno bueno.

**Trabajando:**
- No mas para este Release

**A Trabajar:**
- No mas Storage, liberar.

**En Cola:**

**Posibles Mejoras SW:**

- Revisar la lógica del encoder, ya no se requiere asceleración y usa dos identaciones para avanzar.
- En CC, CP, CR y BC, con BTN encoder habilito cambiar de valor o de unidad para --CuPo, reqiuere reingenieriaa Cursor_Position
- Colocar un indicador de Shift? salvo BC, hay lugar en 20,3
- En TL poner "-" por cada step y marcar en cual se esta ej.: ---3----- -> ----4----
- En TC y TL: mostrar mSec decrecientes?
- En TC, ajustar timing con encoder?
- En CP y CR: Recalcular los limites de W y R en funcion de la DC presente?..
- Heald Checks en setup e informar errores con periféricos I2C, temp., etc. llamar a Check_Limits?
- Activar el LOAD ON OFF por interrupción?
-  Setear hora y fecha del RTC y poder mirarla boton Shift?.
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__))); //
- EL RTC es un DS1307 de MAXIM y cuenta con una EEPROM AT24C32 de ATMEL. Ver de aprovechar esta memoria.

**Posibles Mejoras de HW:**
- Medir frecuencia máxima TC y TL con el osciloscopio a ver hasta donde llega, usar Lipo.
- habilitar control externo de MOSFETs?
