# 📡iGate LoRa/APRS - Proyecto

## 1. Introducción del Proyecto
Un iGate (gateway de Internet) en APRS LoRa es una estación base que recibe señales de otros dispositivos LoRa y las transmite a la red de Internet, haciendo la información disponible públicamente a través de la web.  

Los iGates no necesitan GPS porque están en una ubicación fija y son esenciales para expandir el alcance y la utilidad de la red APRS-LoRa. Recopilan datos de rastreo (trackers) y los envían a la plataforma de Internet para su visualización.

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
    
### 2.1 Conexiones de Hardware

<p align="center">
  <img src="Archivos/Imagenes/ConexionesHardware.jpg" alt="Diagrama general de buses" width="800">
</p>

**Figura 1:** Diagrama de referencia de los buses del LilyGO T3 LoRa32. Muestra las conexiones principales de la placa con los periféricos y fuentes de alimentación.

<p align="center">
  <img src="Archivos/Imagenes/MiDiagramaConexiones.png" alt="Diagrama propio de conexiones" width="500">
</p>

**Figura 2:** Diagrama de conexiones simplificado realizado por el grupo. Detalla los elementos clave:

- **USB-C → PC:** Permite programar la placa y proveer alimentación mientras está conectada al computador.  
- **Antena LoRa → SX1276:** El módulo **SX1276** es el transceptor LoRa integrado en la placa. Se encarga de **transmitir y recibir paquetes de datos** en la frecuencia LoRa (433 MHz en nuestro caso). La antena se conecta a este módulo para mejorar la cobertura y la calidad de la señal.  
- **OLED → I2C:** La pantalla OLED de 0.96” se comunica mediante **I2C**, mostrando información del iGate, como el número de paquetes recibidos o estado de conexión.  
- **Batería → JST:** Conector para una batería Li-Po 3.7V que permite que el iGate funcione sin necesidad de estar conectado al USB.  

> 💡 Nota: Todas estas conexiones son internas en la placa LilyGO T3 LoRa32, excepto la antena y la batería, que se conectan externamente. La correcta conexión garantiza que el iGate pueda recibir paquetes LoRa, mostrarlos en el OLED y enviarlos a APRS-IS.

## 3. Diseño y Aplicación

Como aplicación para este sistema se pretende utilizar el LoRa iGate para el seguimiento en competiciones deportivas como por ejemplo carreras o ciclismo. Cada corredor, ciclista o atleta lleva un pequeño tracker LoRa APRS. En diferentes puntos estratégicos de la competencia se colocan iGates. Cada iGate recibe la señal LoRa y la reenvía automáticamente a la red APRS-IS a través de Internet. Los datos recopilados se pueden visualizar en plataformas como aprs.fi, o integrarse en un mapa personalizado del evento.

El iGate se configurará mediante programación directa en la placa LilyGO T3 LoRa32, siguiendo un flujo planificado que garantice la recepción y transmisión correcta de los datos. La implementación se realizará en varias etapas:

1. **Inicialización de Hardware y Firmware:**  
   - Configuración del microcontrolador ESP32 y del módulo LoRa SX1276.  
   - Inicialización de la pantalla OLED para mostrar información de estado.  
   - Verificación de la conexión a la fuente de alimentación (USB o batería Li-Po).

2. **Conexión a Internet:**  
   - Establecer conexión WiFi para poder enviar los paquetes recibidos hacia APRS-IS.  
   - Implementar control de errores para reconectar automáticamente en caso de caída de la red.

3. **Recepción de Paquetes LoRa:**  
   - Escuchar de forma continua la frecuencia de 433 MHz para recibir paquetes de trackers.  
   - Validar que los paquetes tengan el formato correcto de APRS-LoRa.

4. **Procesamiento y Registro de Datos:**  
   - Guardar los datos en un log local para seguimiento y depuración.  
   - Mostrar información resumida en la pantalla OLED (por ejemplo, número de paquetes recibidos, estado de conexión WiFi).

5. **Transmisión a APRS-IS:**  
   - Reenviar automáticamente los paquetes validados a la red APRS-IS.  
   - Implementar reintentos en caso de fallos de envío.

6. **Ciclo Continuo de Operación:**  
   - Repetir de manera indefinida los pasos anteriores, garantizando la disponibilidad del iGate como puente entre los trackers LoRa y la red de Internet.  

> 💡 Nota: Este enfoque asegura un **control total del sistema**, sin depender de aplicaciones externas de configuración. Cada etapa está planificada para facilitar depuración, escalabilidad y mantenimiento del iGate.

## 4. Lista de Hardware a utilizar

- T3 LoRa32 V1.6.1 – LILYGO®

## Requisitos adicionales

- **Fuente de alimentación**:  
  - USB-Micro USB desde la PC o cargador de 5 V  
  - (Opcional) Batería Li-Po de 3.7 V (ej. 1000–2000 mAh) para autonomía  

- **Antena LoRa**:  
  - Conector IPEX incluido  
  - **Importante:** siempre conectar la antena antes de transmitir para evitar dañar el módulo  

- **Cable USB-Micho USB**:  
  - Para programación y alimentación desde la computadora  

- **(Opcional) Regulador o power bank**:  
  - Alternativa de alimentación si no se usa batería Li-Po  

## Especificaciones de Hardware – LILYGO® T3 LoRa32 V1.6.1

<p align="center">
| Característica       | Especificación |
|-----------------------|----------------|
| **Microcontrolador** | ESP32 (Wi-Fi 802.11 b/g/n y Bluetooth 4.2 BR/EDR & BLE) |
| **Flash**            | 4 MB (SPI) |
| **SRAM**             | 520 KB |
| **Pantalla**         | OLED 0.96” (128x64) integrada |
| **Módulo LoRa**      | SX1276 LoRa transceiver |
| **Frecuencias**      | 433 MHz (en este caso) / 868 MHz / 915 MHz (según modelo) |
| **Interfaz**         | USB Type-C (programación y alimentación) |
| **GPIOs**            | Compatible con ESP32 estándar (UART, SPI, I2C, ADC, DAC, PWM) |
| **Alimentación**     | 5V vía USB-C o batería Li-Po 3.7V (conector JST) |
| **Carga de batería** | Circuito de carga integrado para Li-Po |
| **Antena**           | Conector SMA/IPEX para antena externa LoRa |
| **Dimensiones**      | 25.6 x 51.2 mm aprox. |
</p>

## 5. Diseño Planteado

### 5.1 Diagrama de Bloques
<p align="center">
  <img src="Archivos/Imagenes/DiagramadeBloques.png" alt="Diagrama de Bloques" width="250">
</p>

### 5.1 Pseudocódigo del iGate LoRa/APRS

A continuación se presenta el pseudocódigo del funcionamiento del iGate LoRa/APRS:

```text
Inicio del programa

// Configuración
Definir la red WiFi (nombre y contraseña)
Definir las credenciales de APRS-IS (indicativo y passcode)
Definir el servidor APRS-IS (dirección y puerto)

// Inicialización (Setup)
Iniciar la comunicación serial (para depuración)
Conectar a la red WiFi
Si la conexión WiFi falla, detener el programa

Configurar e iniciar el módulo LoRa
Si el módulo LoRa falla, detener el programa

Conectar al servidor APRS-IS
Si la conexión es exitosa, enviar la línea de login con el indicativo, passcode y versión
Si la conexión falla, mostrar un mensaje de error

// Bucle principal (Loop)
Bucle infinito:

  // Función de repetición (iGate)
  Revisar si hay un paquete de LoRa disponible:
    Si hay un paquete:
      Leer el mensaje completo del paquete LoRa
      Mostrar el mensaje recibido en el monitor serial
      Si la conexión a APRS-IS está activa:
        Construir un mensaje de APRS para reenvío (tu indicativo, >APRS, el mensaje recibido)
        Enviar el mensaje a APRS-IS
        Mostrar el mensaje enviado en el monitor serial

  // Función de baliza (Beacon)
  Comprobar si han pasado 30 segundos desde el último envío de beacon:
    Si han pasado 30 segundos Y la conexión a APRS-IS está activa:
      Construir el mensaje de beacon APRS (tu indicativo, ">APRS", ubicación fija y mensaje de prueba)
      Enviar el mensaje de beacon a APRS-IS
      Mostrar el mensaje de beacon enviado en el monitor serial
      Reiniciar el temporizador del beacon

Fin del programa
```

### 5.2 Máquina de Estados-Máquina de Estados (firmware interno en LILYGO)

<p align="center">
  <img src="Archivos/Imagenes/DiagramadeEstados.png" alt="Máquina de Estados" width="500">
</p>

## 6. Avance de Código de Programación

### 6.1 Preparación del entorno y programación

1. Instalar Arduino IDE

Para Linux (Ubuntu) seguir los siguientes pasos:
- **Descargar la última versión:**

```bash
mkdir -p ~/Documents/arduino
cd ~/Documents/arduino
wget https://downloads.arduino.cc/arduino-ide/arduino-ide_2.3.4_Linux_64bit.AppImage
```
2. **Dar permisos de ejecución**
```bash
chmod +x arduino-ide_2.3.4_Linux_64bit.AppImage
```
3. **Instalar dependencias necesarias (Recomendado)**
```bash
sudo apt update
ssudo apt install libfuse2 libcanberra-gtk-module libcanberra-gtk3-module
```
4. **Ejecutar el IDE**
```bash
./arduino-ide_2.3.4_Linux_64bit.AppImage
```
5. **Si hay problemas de SandBox, ejecutar:**
```bash
./arduino-ide_2.3.4_Linux_64bit.AppImage --no-sandbox
```
6. **Configuración del Arduino IDE para ESP32**
 - 6.1. Abrir Arduino IDE:
 - 6.2. Ir a Archivo → Preferencias.
 - 6.3. En Gestor de URLs adicionales de tarjetas, agregar (paquetes necesarios para el uso de ESP32):
 
```bash
https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
```
- 6.4 **Clic en OK.**
- 6.5 **Instalar soporte ESP32:**
    - 6.5.1 Ir a Herramientas → Placa → Gestor de placas…
    - Buscar ESP32 y hacer clic en Instalar en la versión estable. (Espressif Systems)
    - 6.5.2 Instalar librerías necesarias: En Arduino IDE: Programa → Incluir Librería → Gestionar Librerías…
    - Buscar e instalar: LoRa (para el módulo SX1276(Sandeep Mistry)), WiFi (ya viene incluida con ESP32), TinyGPS++ (si se quiere simular GPS (Mikal Hart))
    
7. **Conectar la LilyGO**
- Conecta la placa vía USB - Micro USB.
- Ir a Herramientas → Puerto y seleccionar el puerto correspondiente.
- Ir a Herramientas → Placa → ESP32 Arduino → LilyGO T3 LoRa32 (o genérica ESP32 si no aparece).

## Programación

### 6.2 iGate LoRa APRS
Este código inicializa un iGate LoRa APRS en un ESP32. Se conecta a WiFi, recibe paquetes LoRa y los reenvía a la red APRS-IS. Además, envía un beacon de prueba cada 30 segundos. La comunicación LoRa se hace mediante el módulo SX1276 conectado por SPI, y APRS-IS mediante la librería APRSIS.

1. **Preparar los datos**:
- **Datos de Configuración importantes**:
```cpp
// WiFi
const char* ssid = "TU_WIFI";         // Aquí va el nombre de tu red WiFi
const char* password = "TU_PASSWORD"; // Aquí la contraseña de tu WiFi

// APRS-IS
const char* callsign = "TU_CALLSIGN";      // Tu indicativo (ej. TI0IE1-10)
const char* aprsPasscode = "TU_PASSCODE"; // Código que generaste en https://aprs.fi/passcode
const char* aprsServer = "euro.aprs2.net";// Servidor APRS-IS a conectar
const int aprsPort = 14580;                // Puerto del servidor APRS-IS
```
💡 Tip: Si no tienes passcode, ve a APRS Passcode Generator

2. **Conectar el hardware**:
- Conecta la LilyGO T3 LoRa32 al PC usando el USB-Micro.
- Asegúrate de tener antena LoRa conectada, aunque sea solo para pruebas de recepción.
- Si quieres, puedes usar la batería Li-Po para pruebas sin USB, pero no es obligatorio ahora.

3. **Seleccionar placa y puerto en Arduino IDE**

- Abre Arduino IDE.
- Ve a Herramientas → Placa → ESP32 Arduino → LilyGO T3 LoRa32 (o ESP32 genérico si no aparece).
- Ve a Herramientas → Puerto y selecciona el puerto USB donde está conectada la LilyGO.

4. **Subir el código**
- Haz clic en “Subir” (Upload) en Arduino IDE.
- Espera a que compile y cargue.
- Abre el Monitor Serial (Herramientas → Monitor Serial) a 115200 baudios para ver la salida en tiempo real.

5. **Verificar funcionamiento**
- En el Monitor Serial deberías ver algo como:

```
Iniciando iGate LoRa APRS...
Conectando a WiFi...
WiFi conectado!
LoRa inicializado en 433.775 MHz
Conectado a APRS-IS!
Paquete recibido: TI0IE1>APRS,TCPIP*:... 
Paquete reenviado a APRS-IS
Beacon enviado a APRS-IS
```
Cada paquete LoRa que reciba la placa se imprimirá y se enviará a APRS-IS.
Cada 30 segundos se envía un beacon de prueba automáticamente.

**Código (C++)**

```c++

#include <WiFi.h> // Incluye la biblioteca para gestionar la conexión WiFi.
#include <SPI.h> // Incluye la biblioteca para la comunicación SPI, necesaria para el chip LoRa.
#include <LoRa.h> // Incluye la biblioteca para el módulo LoRa.

// --- Configuración de la red y APRS-IS ---
// Reemplaza "TU_WIFI" y "TU_PASSWORD" con las credenciales de tu red WiFi.
const char* ssid     = "TU_WIFI";
const char* password = "TU_PASSWORD";

// Configuración del servidor APRS.
const char* aprsServer = "euro.aprs2.net"; // Servidor APRS al que te conectarás.
const int   aprsPort   = 14580; // Puerto estándar para APRS-IS.

// Reemplaza "TU_CALLSIGN" y "TU_PASSCODE" con tus credenciales de APRS.
const char* callsign   = "TU_CALLSIGN";
const char* passcode   = "TU_PASSCODE";  // Passcode único generado para tu indicativo.

WiFiClient client; // Objeto cliente que gestiona la conexión con el servidor.

void setup() {
  Serial.begin(115200); // Inicia la comunicación serial para depuración, a 115200 baudios.

  // --- Conexión WiFi ---
  Serial.println("Conectando a WiFi...");
  WiFi.begin(ssid, password); // Inicia la conexión a la red WiFi.
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print("."); // Imprime un punto cada 500 ms hasta que se conecte.
  }
  Serial.println("\nWiFi conectado"); // Mensaje de éxito al conectar.

  // --- Inicialización LoRa ---
  // Configura los pines del módulo LoRa. (LilyGo T3 usa 18, 14, 26)
  LoRa.setPins(18, 14, 26);
  // Inicia la comunicación LoRa en la frecuencia 433.775 MHz.
  if (!LoRa.begin(433775E3)) {
    Serial.println("Error LoRa"); // Mensaje si el módulo LoRa no se inicializa.
    while (1); // Detiene el programa si hay un error.
  }

  // --- Conexión a APRS-IS ---
  Serial.println("Conectando a APRS-IS...");
  // Intenta conectar al servidor.
  if (client.connect(aprsServer, aprsPort)) {
    Serial.println("Conectado a APRS-IS");
    // Envía la línea de login al servidor.
    client.print("user "); client.print(callsign);
    client.print(" pass "); client.print(passcode);
    client.print(" vers ESP32_iGate 1.0\r\n");
  } else {
    Serial.println("Error de conexión a APRS-IS");
  }
}

void loop() {
  // --- Recibir paquetes LoRa y reenviar a APRS ---
  // Revisa si hay un paquete LoRa disponible.
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    String loraMsg = "";
    while (LoRa.available()) loraMsg += (char)LoRa.read();
    Serial.println("LoRa recibido: " + loraMsg);

    // Si hay conexión, reenvía el mensaje a APRS-IS.
    if (client.connected()) {
      String aprsMsg = String(callsign) +
                       ">APRS,TCPIP*:" + loraMsg;
      client.print(aprsMsg + "\r\n"); // Añade el CRLF (retorno de carro y salto de línea)
      Serial.println("Enviado a APRS-IS: " + aprsMsg);
    }
  }

  // --- Enviar un Beacon cada 30 segundos ---
  static unsigned long lastBeacon = 0;
  // Comprueba si han pasado 30000 ms (30 segundos) desde el último beacon y si la conexión está activa.
  if (millis() - lastBeacon > 30000 && client.connected()) {
    // Formato del mensaje de beacon APRS.
    String beacon = String(callsign) +
                    ">APRS,TCPIP*:!0903.50N/07902.45W-Test iGate";
    client.print(beacon + "\r\n");
    Serial.println("Beacon enviado: " + beacon);
    lastBeacon = millis(); // Actualiza el tiempo del último beacon.
  }
}

```
### 6.3 Pruebas Preliminares

Durante las primeras pruebas con el iGate LoRa/APRS, se realizaron las siguientes observaciones:

1. **Compilación y carga del código**  
   El código inicial se compiló y cargó al LilyGO sin errores en el IDE.  

<p align="center">
  <img src="Archivos/Imagenes/MonitorIdle.jpeg" alt="Monitor Serial sin errores" width="800">
</p>

**Figura 1:** Monitor serial mostrando que el código se cargó correctamente y no se presentan errores (*Idle*).

2. **Modo Listening del LilyGO**  
   Al encenderse, el dispositivo mostraba en la pantalla el mensaje **Listening**, indicando que estaba listo para recibir paquetes LoRa.  

<p align="center">
  <img src="Archivos/Imagenes/Listening.jpeg" alt="LilyGO en modo Listening" width="600">
</p>

**Figura 2:** Pantalla del LilyGO mostrando *Listening*, indicando que el iGate está activo y a la espera de paquetes.

3. **Visualización en APRS.fi**  
   Buscando el callsign del iGate en [aprs.fi](https://aprs.fi), el dispositivo aparecía en la ubicación programada, confirmando la conexión correcta con la red APRS-IS.  

<p align="center">
  <img src="Archivos/Imagenes/Mapa_iGate.png" alt="Mapa con iGate" width="600">
</p>

**Figura 3:** Mapa de APRS.fi mostrando la ubicación del iGate.

4. **Recepción de Raw Packets**  
   Se observaron **raw packets** en la página de APRS.fi.  

<p align="center">
  <img src="Archivos/Imagenes/RawPackets.png" alt="Raw Packets APRS" width="600">
</p>

**Figura 4:** Sección de *raw packets* en APRS.fi. Los raw packets son los mensajes APRS recibidos tal como llegaron, sin procesamiento. Esto confirma que el iGate está recibiendo y retransmitiendo correctamente los paquetes LoRa a APRS-IS.

5. **Prueba de Beacons**  
   Al intentar enviar beacons personalizados, el iGate continuaba transmitiendo la ubicación del GPS integrado, en lugar de los datos fijos configurados en los beacons. Esto indica que algún firmware previo o el módulo GPS sobrescribe la ubicación manual.  

**Resumen:**  
Las pruebas preliminares confirman que el iGate recibe y retransmite paquetes LoRa a APRS-IS, visualizables en el mapa y en raw packets. Sin embargo, el envío de beacons personalizados no funcionó como se esperaba debido a la interferencia del GPS integrado. Esta información servirá para la depuración en futuras entregas.


## 6. Cronograma Preliminar

| Semana | Actividad / Objetivo | Avance Estimado |
|--------|--------------------|----------------|
| 5      | Configuración de entorno de programación y pruebas iniciales con la LilyGO T3. | ✅ Completado |
| 6      | Desarrollo inicial del código: <br> - Recepción de paquetes LoRa <br> - Conexión a WiFi <br> - Estructura básica para envío a APRS-IS | ✅ Completado |
| 7      | Integración hardware-software y pruebas de funcionamiento: <br> - Conectar antena LoRa <br> - Verificar envío de datos a APRS-IS <br> - Depurar errores iniciales | ✅ Completado |
| 8      | **Entrega parcial (Informe y Presentación Parcial)**: <br> - Diagramas de bloques y máquina de estados <br> - Listado de hardware y justificación técnica <br> - Planteamiento estructurado del diseño <br> - Implementación en hardware y bus de conexión <br> - Tipo de comunicación de cada periférico <br> - Pseudo-código para control del sistema <br> - Definición de tramas de datos <br> - Cronograma y presupuesto para las semanas restantes <br> - Código inicial en GitHub documentado | ✅ Completado |
| 9-15   | Implementación y Evaluación Continua: <br> - Pruebas de recepción y transmisión de trackers <br> - Depuración y refinamiento del código <br> - Commits regulares y documentados en GitHub <br> - Visualización de datos en APRS.fi y aprsdirect.de | ⚙️ En progreso |
| 16     | Defensa del Proyecto Final: <br> - Entrega de informe completo <br> - Presentación final <br> - Código final documentado y funcional <br> - Confirmación de publicación de datos en APRS | ⏳ Planeado |



