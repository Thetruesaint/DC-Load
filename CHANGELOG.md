  # Changelog

## v1.69 22/02/2025

Mejoras:

##Descarga de baterias, automatico## <- MileStone!
En TL ahora muestra la cantidad de instrucciones de la lista
En BC, reog. de print de Voltage Cutoff
Agrego simulacion de voltage con pote externo y en BC descarga de bateria
Nuevo diagram.json para Wokwi
Ajuste de tiempo de refresh del LCD a 200ms
Saco del Loop a Cursor_Position() y lo dejo solo para CC, CR, CP y BC
En BC saco Registro de datos por Serial, no lo uso la verdad
Fixes:

En TL, para lista de 10, quedaba el cero en el conteo de instrucciones.
En CR ya se puede limitar a 0,1ohms la resistencia.
Read_Encoder, encoderMax = 10000 (salvo TC y TL) para que no supere los 10A de ninguna manera ya que se llama por interrupción
En TL el periodo de la instrucción anterior no se borraba. Reingenieria de TL
En BC, traia el valor de Set del Modo CR, se resetea reading y encoder
Update_LCD corria la W y borraba col 0 row 3, afectando modos BC, TC y TL
Bugs detectados:

Tal vez falte ajustar un poco mejor a la descarga de BC, pero esta funcional
Trabajando:

Shift + Modo, resetea el modo? o shift + < va para atras en la config?
En Cola:

Mostrar el tipo de Baterria en la plantilla de BC? o los ctffV?
Ver de Cambiar "Set I =" que esta en todos los modos y ocupa mucho espacio
En modos TC y TS mostrar mSec decrecientes?
Posibles Mejoras:

Ponerle un Buzzer?
Modo Calibracíón incluir external voltage sense?
Con anuncio de limite exedido, actualizar el limite excedido y parpadearlo
Uso para Shift paa ir a un modo directo o a Calibración S+C?
Activar el LOAD ON OFF por interrupción?, desactivar el mosfet con el procesador directamente?
Ver la frecuencia máxima de conmutación de los Trasient y limitarla a esa
Setear hora y fecha del RTC y poder mirarla boton Shift?. Por ahora lo hago asi: rtc.adjust(DateTime(F(DATE), F(TIME))); // Para ajsutar la hora al momento de compilar el código pero luego se debe comentar para que reloj siga corriendo
Recalcular los limites de W y R en funcion de la DC presente?..
Ajustar timing con encoder en TC mode?
Ver de agregar Heald Checks antes de inicializar e informar error de detectarse. */

## v1.68 19/02/2025

Mejoras:

Value_Input parametrizada con coordenas con variables locales.
Ajustes de menues y seteos de BC, y reingenieria del algoritmo TC y TL.
Borra la C de la Temperatura a la derecha si fue mayor a 10 y vuelve a menos de 10
Cambio de variables String Mode a Enum, costo el cambio, pero ahorro FLASH
Organización de llamadas a Funciones con Switch, mucha horas a esto. Mas fácil para hacer cambios o debugs
Cbio. printLCDNumber()
Poder salir de menus de seteos o llamar a Config_Limits dentro de CC, CP y CR.
Config_Limits y Show_Limits ahora se llaman y vuelven al mismo modo. No para TC o TL, para BC, solo despues de Configurar el modo
El modo Trasient pasaron a ser dos modos Continuo o de Lista
Mejora en los mensajes de Check_Limites para que se muestren bien.
Fixes:

W > 100 borra la W de mas cuando baja el valor
TC iniciaba con 0A por el periodo y el timing no se reiniciaba con Load Off
Si hay warning por exceso de limites, resetear valores que se estaban ingresando
Refresh de LCD al salir y volver al mismo modo
Mejora Update LCD, refresca por limits, Limits Checks con adv. intermitentes.
Reorganización total de los modos y corrección de bugs en el proceso.
Bugs detectados:

En modo CR se puede poner una resistencia 0 (corriente alta, divide por cero)
Posibles Mejoras:

Sacar decimales en CR y CP? ver que presición quiero tener.
Mostrar el tipo de Baterria en la plantilla de BC
printLCD_S(0, 1, "Instruccion " + String(i + 1)); muestre las instrucciones totales tipo "1 de X"
Uso para Shift paa ir a un modo directo o a Calibración S+C?
Activar el LOAD ON OFF por interrupción?, desactivar el mosfet con el procesador directamente?
Ver de Cambiar "Set I =" que esta en todos los modos y ocupa mucho espacio
Descarga de baterias con corriente decreciente al llegar a batteryvoltagecutoff
Calibracíón incluir external voltage sense
Ver la frecuencia máxima de conmutación de los Trasient y limitarla a esa
Setear hora y fecha del RTC y poder mirarla boton Shift?. Por ahora lo hago asi: rtc.adjust(DateTime(F(DATE), F(TIME))); // Para ajsutar la hora al momento de compilar el código pero luego se debe comentar para que reloj siga corriendo
Recalcular los limites de W y R en funcion de la DC presente?..
Ajustar timing con encoder en TC mode?
Ver de agregar Heald Checks antes de inicializar e informar error de detectarse.

## v1.67 18/02/2025

Fixs:

Timming de Trasient mode, se inicia correctamente con el estado de la carga
Modo BC: Flickering y refresh de tiempo y mAH por segundo.
Optimización de código: Elimino timer_status y mejoro Load_ON_Status, el printLCD paso a UpdateLCD
Bugs:

En modo CR se puede poner una resistencia 0 (corriente alta, divide por cero)
Refresh de LCD al salir y volver al mismo modo o luego de un warning de limites excedidos.
Mejoras ptes.:

Poder salir de un modo, dentro de los menues de selección. (Requiere reorganización del algorinmo de modos)
Poder salir del menu de configuración en cualquier momento. (Requiere reorganización del algorinmo de modos)
Descarga de baterias con corriente decreciente al llegar a batteryvoltagecutoff
Calibracíón incluir external voltage sense

## v1.66 18/02/2025

Fixs:

Acorte mensajes y genere funcion printLCD con F()ahorrado bastante FLASH
Función Check_Limits mejorada
Actualiza Temp con new_temp en UpdateLCD
En modo CC, CR y CP ya aparece el indicador ">"
Bugs:

Refresh de LCD al salir y volver al mismo modo o luego de un warning de limites excedidos.
En TC y TL con ON, no resetea el timming, como que queda corriendo revisar.
En entorno real hay mediciones negativas de corriente con LOAD OFF
En modo CR se puede poner una resistencia 0 (corriente alta, divide por cero)
Mejoras ptes.:

Calibracíón, incluir external voltage sense
Descarga de baterias con corriente decreciente al llegar a batteryvoltagecutoff
Poder salir de un modo, dentro de los menues de selección
Poder salir del menu de configuración en cualquier momento

## v1.65 18/02/2025

Muchas mejoras de código. Mejoras de checklimits, Temp Control, Bugs:

Poca memoria FLASH
Refresh en salida de Config Limits
Refressh de Temp si no cambia
mensajes de exceso de limites afecta renglon 4 en todos los modos
En modo CC, CR y CP: Desaparece indicador ">" en pos. 0, renglon 4, ver si no se pisa por exseso de renglon 3 Mejoras ptes.:
Calibracíón (liberar FLASH)
Descarga de baterias con corriente decreciente al llegar a batteryvoltagecutoff (liberar FLASH)
Podes salir de un modo, dentro de los menues de selección

## v1.64 16/02/2025

Muchas mejoras en el código, en el manejo EEPROM, temp de Fans on a 40°C y ajusto factor de control de corriente.