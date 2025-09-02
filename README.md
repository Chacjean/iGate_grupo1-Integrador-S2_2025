# iGate LoRa/APRS - Proyecto

## 1. Introducción del Proyecto

Un iGate (gateway de Internet) en APRS LoRa es una estación base que recibe señales de otros dispositivos LoRa y las transmite a la red de Internet, haciendo la información disponible públicamente a través de la web.  

Los iGates no necesitan GPS porque están en una ubicación fija y son esenciales para expandir el alcance y la utilidad de la red APRS-LoRa. Recopilan datos de rastreo (trackers) y los envían a la plataforma de Internet para su visualización.

---

## 2. Conceptos Importantes

- **iGate:**  
  Puente entre la red LoRa e Internet, que recibe y retransmite los datos. Requiere un receptor de radio LoRa (generalmente en la frecuencia de 433 MHz) y un computador o mini-computador para conectarse a la web.

- **APRS (Automatic Packet Reporting System):**  
  Sistema de radioaficionados que transmite paquetes de datos con información como posición, mensajes y datos ambientales.

- **LoRa (Long Range):**  
  Tecnología de modulación que permite comunicación inalámbrica de largo alcance, mayor penetración de señal y bajo consumo energético.

- **Tracker:**  
  Dispositivo LoRa, usualmente con módulo GPS, que envía automáticamente su posición y otros datos.

- **Frecuencia:**  
  APRS-LoRa opera en frecuencias específicas; la banda de 433 MHz es común para estos sistemas.

- **Firmware:**  
  Software específico instalado en las placas de desarrollo para que los dispositivos funcionen como iGate o tracker.

- **Placa de desarrollo:**  
  Módulo electrónico con un módem LoRa, configurable como iGate o tracker mediante firmware.

## 2. Diseño y Planificación

Como aplicación para este sistema se pretende utilizar el LoRa iGate para el seguimiento en competiciones deportivas como por ejemplo carreras o ciclismo. Cada corredor, ciclista o atleta lleva un pequeño tracker LoRa APRS. En diferentes puntos estratégicos de la competencia se colocan iGates. Cada iGate recibe la señal LoRa y la reenvía automáticamente a la red APRS-IS a través de Internet. Los datos recopilados se pueden visualizar en plataformas como aprs.fi, o integrarse en un mapa personalizado del evento.

### Diagrama de Bloques Inicial
### Máquina de Estados Inicial

## 3. Lista de Hardware a utilizar

T3 LoRa32 V1.6.1 – LILYGO®

## 4. Diseño Planteado

Paso 1. Descargar e instalar ¨Visual Studio Code¨

Paso 2. Descargar desde Github los parquetes de iGate.

Paso 3. Configurar el iGate en VS Code y cargar su placa vía USB:

-Identificar el tipo de tarjeta---"LILYGO"---Import(paquetes)---Upload
-Conf.data
-Conectar la tarjeta al PC vía USB
-Build (compilar) --- Upload --- Upload Filesystem Image

## 5. Avance de Código de Programación


## 6. Cronograma Preliminar


