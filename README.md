**Mejoras:**
  - ##Descarga de baterias, automatico## <- **MileStone!**
  - En TL ahora muestra la cantidad de instrucciones de la lista
  - En BC, reog. de print de Voltage Cutoff
  - Agrego simulacion de voltage con pote externo y en BC descarga de bateria
  - Nuevo diagram.json para Wokwi
  - Ajuste de tiempo de refresh del LCD a 200ms
  - Saco del Loop a Cursor_Position() y lo dejo solo para CC, CR, CP y BC
  - En BC saco Registro de datos por Serial, no lo uso la verdad

  **Fixes:**
  - En TL, para lista de 10, quedaba el cero en el conteo de instrucciones.
  - En CR ya se puede limitar a 0,1ohms la resistencia.
  - Read_Encoder, encoderMax = 10000 (salvo TC y TL) para que no supere los 10A de ninguna manera ya que se llama por interrupción
  - En TL el periodo de la instrucción anterior no se borraba. Reingenieria de TL
  - En BC, traia el valor de Set del Modo CR, se resetea reading y encoder
  - Update_LCD corria la W y borraba col 0 row 3, afectando modos BC, TC y TL

  **Bugs detectados:**
  - Tal vez falte ajustar un poco mejor a la descarga de BC, pero esta funcional
  - 
   
 **Trabajando:**
   - Shift + Modo, resetea el modo? o shift + < va para atras en la config?
  
  En Cola:
  - Mostrar el tipo de Baterria en la plantilla de BC? o los ctffV?
  - Ver de Cambiar "Set I =" que esta en todos los modos y ocupa mucho espacio
  - En modos TC y TS mostrar mSec decrecientes?

  **Posibles Mejoras:**
  - Ponerle un Buzzer?
  - Modo Calibracíón incluir external voltage sense?
  - Con anuncio de limite exedido, actualizar el limite excedido y parpadearlo
  - Uso para Shift paa ir a un modo directo o a Calibración S+C?
  - Activar el LOAD ON OFF por interrupción?, desactivar el mosfet con el procesador directamente?
  - Ver la frecuencia máxima de conmutación de los Trasient y limitarla a esa
  - Setear hora y fecha del RTC y poder mirarla boton Shift?. Por ahora lo hago asi:
      rtc.adjust(DateTime(F(__DATE__), F(__TIME__))); // Para ajsutar la hora al momento de compilar el código pero luego se debe comentar para que reloj siga corriendo
  - Recalcular los limites de W y R en funcion de la DC presente?..
  - Ajustar timing con encoder en TC mode?
  - Ver de agregar Heald Checks antes de inicializar e informar error de detectarse.
  */
