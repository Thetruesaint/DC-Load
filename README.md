# DC LOAD - Carga Electrónica

<p align="center">
  <img src="doc/images/DC%20LOAD%20Front.jpg" alt="DC Load ESP32 vista frontal" width="720">
</p>

Carga electronica programable basada en ESP32 para ensayo de fuentes, baterias y etapas de potencia. El firmware mide variables electricas y termicas, controla la carga en distintos modos y muestra la informacion en una interfaz TFT con menus de configuracion, calibracion y mantenimiento.

## Agradecimientos

Este proyecto toma como base el trabajo de Mr. Louis Scully y sigue evolucionando con redisenos propios de hardware y firmware. 
(Mr. Louis Scully Playlist: <https://youtube.com/playlist?list=PLUMG8JNssPPzbr4LydbTcBrhoPlemu5Dt&si=T7vzUby6amjt4Xsd>)


Las nuevas PCB se encuentran en fabricacion, ¡gracias PCBWay por ayudarme a mejorarlas y llevarlas al siguiente nivel!

<p>
  <img src="doc/images/PCBWay.jpg" alt="PCBWay" width="160">
</p>

## Funciones principales

- Modos de carga `CC`, `CP`, `CR`, `BC`, `TC`, `TL` y `CA`.
- Medicion de corriente, tension, potencia y temperatura.
- Interfaz TFT con pantallas de operacion, configuracion, calibracion y advertencias.
- Limites de proteccion configurables.
- Actualizacion de firmware por WiFi mediante OTA.
- Simulacion Wokwi para validar UI y parte del flujo de operacion.

## Uso basico

1. Conectar la fuente o bateria a ensayar.
2. Verificar limites de corriente, tension, potencia y temperatura.
3. Seleccionar el modo de trabajo deseado.
4. Ajustar el setpoint con teclado o encoder.
5. Activar la carga y supervisar la lectura en pantalla.

## Actualizacion OTA

1. En el equipo, ir a `Configuration -> FW Update`.
2. Esperar a que la pantalla muestre IP y host.
3. Compilar el firmware para hardware real.
4. Enviar el binario por OTA desde PowerShell.

## Fotos y material visual

- Vista frontal del equipo: [doc/images/DC LOAD Front.jpg](doc/images/DC%20LOAD%20Front.jpg)
- Fotos del desarrollo y del hardware: [doc/images/](doc/images/)
- Referencia de display TFT: [doc/images/TFT.webp](doc/images/TFT.webp)
- Pinout de ESP32 usado como referencia: [doc/images/ESP32 wroom nodemcu pinout.jpg](doc/images/ESP32%20wroom%20nodemcu%20pinout.jpg)

## Documentacion de hardware

- Esquematicos: [doc/hardware/Schematics/](doc/hardware/Schematics/)
- PCBs: [doc/hardware/PCBs/](doc/hardware/PCBs/)
- Datasheets: [doc/hardware/Datasheets/](doc/hardware/Datasheets/)

Actualmente el repositorio ya incluye esquematicos, layouts de PCB y hojas de datos de componentes clave para acompanar el firmware.


## Estructura de documentacion

- [README.md](README.md): descripcion general del proyecto y guia rapida.
- [CHANGELOG.md](CHANGELOG.md): mejoras, fixes y evolucion por version.
- [doc/images/](doc/images/): fotos, capturas y referencias visuales.
- [doc/hardware/](doc/hardware/): esquematicos, PCB y datasheets.
- [doc/usage/](doc/usage/): espacio para futuras guias de calibracion, operacion y servicio.

## Estado actual del Firmware

- Plataforma actual: `v2.x` basada en ESP32.
- Plataforma legacy: `v1.x` basada en Arduino Nano.
- Historial de cambios y avances: [CHANGELOG.md](CHANGELOG.md).

Versiones publicadas: [GitHub Releases](https://github.com/Thetruesaint/DC-Load/releases)
