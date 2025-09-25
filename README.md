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


## 5. Diseño Planteado

Paso 1. **Descargar e instalar ¨Visual Studio Code¨**

Paso 2. **Descargar desde Github los parquetes de iGate.**

Paso 3. **Configurar el iGate en VS Code y cargar su placa vía USB:**

-Identificar el tipo de tarjeta---"LILYGO"---Import(paquetes)---Upload
-Conf.data
-Conectar la tarjeta al PC vía USB
-Build (compilar) --- Upload --- Upload Filesystem Image

### 5.1 Diagrama de Bloques
<p align="center">
  <img src="Archivos/Imagenes/DiagramadeBloques.png" alt="Diagrama de Bloques" width="250">
</p>

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
    - Buscar e instalar: LoRa (para el módulo SX1276(Sandeep Mistry)), WiFi (ya viene incluida con ESP32),TinyGPS++ (si quieres simular GPS (Mikal Hart))
    
7. **Conectar la LilyGO**
- Conecta la placa vía USB-C.
- Ir a Herramientas → Puerto y seleccionar el puerto correspondiente.
- Ir a Herramientas → Placa → ESP32 Arduino → LilyGO T3 LoRa32 (o genérica ESP32 si no aparece).

## Programación

### 6.2 Probar la recepción de LoRa
1. Abrir el monitor serial en 115200 bps.
2. Usar función temporal para ver si la placa “escucha” paquetes LoRa:

```cpp
#include <LoRa.h>

#define LORA_SS 18
#define LORA_RST 14
#define LORA_DIO0 26

void setup() {
  Serial.begin(115200);
  while (!Serial);

  if (!LoRa.begin(433775E3)) {
    Serial.println("Error inicializando LoRa");
    while (1);
  }
  Serial.println("LoRa listo para recibir paquetes");
}

void loop() {
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    Serial.print("Paquete recibido: ");
    while (LoRa.available()) {
      Serial.print((char)LoRa.read());
    }
    Serial.println();
  }
}
```
### 6.3 Conectar a APRS-IS
1. Agregar la librería APRS-IS Client → Para enviar paquetes a la red APRS-IS.
- Descarga la librería (se obtiene de manera externa y se agrega en Arduino IDE):
https://github.com/markqvist/LibAPRS (Code → Download Zip)
- Abre Arduino IDE: Ve a Programa → Incluir Librería → Añadir .ZIP de librería....Selecciona el archivo ZIP descargado y haz clic en Abrir.
- Verificar instalación: Ve a Programa → Incluir Librería, y ver LibAPRS en la lista de librerías disponibles.

```cpp
#include <WiFi.h>
#include <LoRa.h>
#include <APRS_IS.h>  // Librería para enviar a APRS-IS

// --- Configuración WiFi ---
const char* ssid = "TU_WIFI";
const char* password = "TU_PASSWORD";

// --- Configuración LoRa ---
#define LORA_SS 18
#define LORA_RST 14
#define LORA_DIO0 26
#define LORA_FREQ 433775E3

// --- Configuración APRS-IS ---
const char* callsign = "T00IE1-10";      // Cambiar por tu callsign
const char* passcode = "12345";          // Código APRS generado online
const char* server = "euro.aprs2.net";
const int port = 14580;

APRS_IS aprs(callsign, passcode, server, port);

void setup() {
  Serial.begin(115200);
  while(!Serial);

  // Conectar a WiFi
  WiFi.begin(ssid, password);
  Serial.print("Conectando a WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi conectado!");

  // Inicializar LoRa
  if (!LoRa.begin(LORA_FREQ)) {
    Serial.println("Error inicializando LoRa");
    while (1);
  }
  Serial.println("LoRa listo");

  // Conectar a APRS-IS
  if (aprs.begin()) {
    Serial.println("Conectado a APRS-IS!");
  } else {
    Serial.println("Error al conectar a APRS-IS");
  }
}

void loop() {
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    String mensaje = "";
    while (LoRa.available()) {
      mensaje += (char)LoRa.read();
    }
    Serial.print("Paquete recibido: ");
    Serial.println(mensaje);

    // Enviar a APRS-IS
    aprs.sendMicE(mensaje); // O sendPacket si tu librería lo requiere
    Serial.println("Paquete enviado a APRS-IS");
  }

  // Mantener conexión WiFi
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi desconectado. Reconectando...");
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) delay(500);
    Serial.println("WiFi reconectado!");
  }

  delay(100);
}
```
El código anterior:
1. Conecta tu LilyGO al WiFi.
2. Inicializa el módulo LoRa en 433.775 MHz.
3. Escucha paquetes LoRa y los imprime en el monitor serial.
4. Reenvía los paquetes recibidos a APRS-IS usando tu callsign.
5. Mantiene la conexión WiFi viva.

### 6.4 iGate LoRa APRS
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

const char* ssid     = "TU_WIFI";      // ✅ Nombre exacto de tu red WiFi (SSID)
const char* password = "TU_PASSWORD";  // ✅ Contraseña de esa red WiFi

const char* aprsServer = "euro.aprs2.net"; // ✅ Puedes dejar este servidor APRS-IS
const int   aprsPort   = 14580;            // ✅ Puerto estándar (deja igual)

const char* callsign   = "TU_CALLSIGN";    // ✅ Tu indicativo APRS, por ejemplo: "TI0ABC-10"
const char* passcode   = "TU_PASSCODE";    // ✅ Passcode APRS generado para ese indicativo


```cpp
#include <WiFi.h>
#include <SPI.h>
#include <LoRa.h>

const char* ssid     = "TU_WIFI";
const char* password = "TU_PASSWORD";

const char* aprsServer = "euro.aprs2.net";
const int   aprsPort   = 14580;
const char* callsign   = "TU_CALLSIGN";
const char* passcode   = "TU_PASSCODE";  // Genera tu passcode APRS

WiFiClient client;

void setup() {
  Serial.begin(115200);

  // --- WiFi ---
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi conectado");

  // --- LoRa ---
  LoRa.setPins(18, 14, 26);
  if (!LoRa.begin(433775E3)) {
    Serial.println("Error LoRa");
    while (1);
  }

  // --- APRS-IS ---
  if (client.connect(aprsServer, aprsPort)) {
    Serial.println("Conectado a APRS-IS");
    client.print("user "); client.print(callsign);
    client.print(" pass "); client.print(passcode);
    client.print(" vers ESP32_iGate 1.0\r\n");
  } else {
    Serial.println("Error de conexión a APRS-IS");
  }
}

void loop() {
  // Recibir LoRa y reenviar a APRS
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    String loraMsg = "";
    while (LoRa.available()) loraMsg += (char)LoRa.read();
    Serial.println("LoRa recibido: " + loraMsg);

    if (client.connected()) {
      // Mensaje APRS válido (ejemplo beacon)
      String aprsMsg = String(callsign) +
                       ">APRS,TCPIP*:" + loraMsg;
      client.print(aprsMsg + "\r\n");
      Serial.println("Enviado a APRS-IS: " + aprsMsg);
    }
  }

  // Beacon cada 30s
  static unsigned long lastBeacon = 0;
  if (millis() - lastBeacon > 30000 && client.connected()) {
    String beacon = String(callsign) +
                    ">APRS,TCPIP*:!0903.50N/07902.45W-Test iGate";
    client.print(beacon + "\r\n");
    Serial.println("Beacon enviado: " + beacon);
    lastBeacon = millis();
  }
}

```

**SEGUNDA PRUEBA DE BEACON**
```
#include <WiFi.h>
#include <SPI.h>
#include <LoRa.h>

// ---------- CONFIGURACIÓN WiFi ----------
const char* ssid     = "TU_WIFI";       // Tu red WiFi
const char* password = "TU_PASSWORD";   // Contraseña WiFi

// ---------- CONFIGURACIÓN APRS-IS ----------
const char* aprsServer = "euro.aprs2.net";
const int   aprsPort   = 14580;
const char* callsign   = "TU_CALLSIGN"; // Tu indicativo APRS
const char* passcode   = "TU_PASSCODE"; // Passcode generado en APRS

WiFiClient client;

// ---------- CONFIGURACIÓN LoRa ----------
#define LORA_SS 18
#define LORA_RST 14
#define LORA_DIO0 26
#define LORA_FREQUENCY 433775E3 // 433.775 MHz

// Coordenadas reales para tu beacon
const char* beaconLatitude  = "09 03.50N";
const char* beaconLongitude = "079 02.45W";
const char* beaconMessage   = "iGate activo - Test";

void setup() {
  Serial.begin(115200);

  // --- Conexión WiFi ---
  WiFi.begin(ssid, password);
  Serial.print("Conectando a WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi conectado");

  // --- Inicializar LoRa ---
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);
  if (!LoRa.begin(LORA_FREQUENCY)) {
    Serial.println("Error LoRa");
    while (1);
  }
  Serial.println("LoRa inicializado");

  // --- Conectar a APRS-IS ---
  if (client.connect(aprsServer, aprsPort)) {
    Serial.println("Conectado a APRS-IS");
    client.print("user "); client.print(callsign);
    client.print(" pass "); client.print(passcode);
    client.print(" vers ESP32_iGate 1.0\r\n");
  } else {
    Serial.println("Error de conexión a APRS-IS");
  }
}

void loop() {
  // ---------------- Recibir paquetes LoRa ----------------
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    String loraMsg = "";
    while (LoRa.available()) loraMsg += (char)LoRa.read();
    Serial.println("LoRa recibido: " + loraMsg);

    // Reenviar a APRS-IS
    if (client.connected()) {
      String aprsMsg = String(callsign) + ">APRS,TCPIP*:" + loraMsg;
      client.print(aprsMsg + "\r\n");
      Serial.println("Enviado a APRS-IS: " + aprsMsg);
    }
  }

  // ---------------- Enviar beacon cada 30s ----------------
  static unsigned long lastBeacon = 0;
  if (millis() - lastBeacon > 30000 && client.connected()) {
    String beacon = String(callsign) + ">APRS,TCPIP*:!" +
                    beaconLatitude + "/" + beaconLongitude + "-" +
                    beaconMessage;
    client.print(beacon + "\r\n");
    Serial.println("Beacon enviado: " + beacon);
    lastBeacon = millis();
  }
}
```

**TERCERA PRUEBA**

```
#include <WiFi.h>
#include <SPI.h>
#include <LoRa.h>

// --- Configuración de la Red y APRS ---
// Reemplaza los valores con tu configuración.
const char* ssid = "TU_WIFI";
const char* password = "TU_PASSWORD";

const char* aprsServer = "euro.aprs2.net";
const int aprsPort = 14580;
const char* callsign = "TU_CALLSIGN";
const char* passcode = "TU_PASSCODE"; // Genera tu passcode APRS en http://www.aprs-is.net/ask/portpasscode.aspx

WiFiClient client;

void setup() {
  Serial.begin(115200);

  Serial.println("Iniciando iGate LoRa a APRS...");

  // --- Conexión WiFi ---
  Serial.print("Conectando a WiFi: ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi conectado.");
  Serial.print("Dirección IP: ");
  Serial.println(WiFi.localIP());

  // --- Inicialización de LoRa ---
  LoRa.setPins(18, 14, 26);
  if (!LoRa.begin(433775E3)) {
    Serial.println("Error al inicializar LoRa. Verifica el cableado.");
    while (1);
  }
  Serial.println("LoRa inicializado correctamente.");

  // --- Conexión y Login a APRS-IS ---
  connectToAprsIs();
}

void loop() {
  // Manejo de la conexión: si se pierde, intenta reconectar.
  if (!client.connected()) {
    Serial.println("Conexión con APRS-IS perdida. Reintentando en 5 segundos...");
    delay(5000);
    connectToAprsIs();
  }

  // Leer y descartar los datos del servidor para mantener la conexión viva.
  while (client.available()) {
    client.read(); // Simplemente lee los bytes para mantener el flujo de datos.
  }

  // --- Recibir LoRa y reenviar a APRS ---
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    String loraMsg = "";
    while (LoRa.available()) {
      loraMsg += (char)LoRa.read();
    }
    Serial.println("LoRa recibido: " + loraMsg);

    // Solo reenvía si la conexión está activa.
    if (client.connected()) {
      // Formato del mensaje APRS
      String aprsMsg = String(callsign) + ">APRS,TCPIP*:" + loraMsg;
      client.print(aprsMsg + "\r\n");
      Serial.println("Enviado a APRS-IS: " + aprsMsg);
    }
  }

  // --- Enviar Beacon cada 30 segundos ---
  static unsigned long lastBeacon = 0;
  if (millis() - lastBeacon > 30000 && client.connected()) {
    // La cadena del beacon debe ser un mensaje APRS válido.
    String beacon = String(callsign) +
                    ">APRS,TCPIP*:!0903.50N/07902.45W-Test iGate";
    client.print(beacon + "\r\n");
    Serial.println("Beacon enviado: " + beacon);
    lastBeacon = millis();
  }
}

// Función para conectar y loguear en el servidor APRS-IS.
void connectToAprsIs() {
  if (client.connect(aprsServer, aprsPort)) {
    Serial.println("Conectado a APRS-IS.");
    // Envía la línea de login.
    client.print("user ");
    client.print(callsign);
    client.print(" pass ");
    client.print(passcode);
    client.print(" vers ESP32_iGate 1.0\r\n");
    Serial.println("Login enviado. Esperando respuesta...");

    // Espera un máximo de 5 segundos para leer la respuesta inicial del servidor.
    unsigned long startTime = millis();
    while (client.connected() && (millis() - startTime < 5000)) {
      if (client.available()) {
        String response = client.readStringUntil('\n');
        Serial.print("Respuesta APRS-IS: ");
        Serial.println(response);
        // Si la respuesta no contiene un error, asumimos que el login fue exitoso.
        if (response.indexOf("# logreq") == -1) {
          Serial.println("Login exitoso.");
          return;
        }
      }
      delay(10);
    }
    
    // Si llegamos aquí, hubo un problema.
    Serial.println("No se recibió una respuesta válida o se perdió la conexión después del login.");
    client.stop(); // Detener la conexión para un nuevo intento.
  } else {
    Serial.println("Error de conexión a APRS-IS.");
  }
}
```
**Prueba sin GPS para ver si envia beacon**
```
#include <WiFi.h>
#include <SPI.h>
#include <LoRa.h>

// ---------- CONFIGURACIÓN WiFi ----------
const char* ssid     = "TU_WIFI";       // Tu red WiFi
const char* password = "TU_PASSWORD";   // Contraseña WiFi

// ---------- CONFIGURACIÓN APRS-IS ----------
const char* aprsServer = "euro.aprs2.net";
const int   aprsPort   = 14580;
const char* callsign   = "TU_CALLSIGN"; // Tu indicativo APRS
const char* passcode   = "TU_PASSCODE"; // Passcode generado en APRS

WiFiClient client;

// ---------- CONFIGURACIÓN LoRa ----------
#define LORA_SS 18
#define LORA_RST 14
#define LORA_DIO0 26
#define LORA_FREQUENCY 433775E3 // 433.775 MHz

// Coordenadas fijas para el beacon
const char* beaconLatitude  = "09 03.50N";
const char* beaconLongitude = "079 02.45W";
const char* beaconMessage   = "iGate Test Beacon";

void setup() {
  Serial.begin(115200);
  delay(1000);

  // --- Conectar WiFi ---
  WiFi.begin(ssid, password);
  Serial.print("Conectando a WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi conectado");

  // --- Inicializar LoRa ---
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);
  if (!LoRa.begin(LORA_FREQUENCY)) {
    Serial.println("Error al inicializar LoRa");
    while (1);
  }
  Serial.println("LoRa inicializado");

  // --- Conectar a APRS-IS ---
  if (client.connect(aprsServer, aprsPort)) {
    Serial.println("Conectado a APRS-IS");
    client.print("user "); client.print(callsign);
    client.print(" pass "); client.print(passcode);
    client.print(" vers ESP32_iGate 1.0\r\n");
  } else {
    Serial.println("Error de conexión a APRS-IS");
  }
}

void loop() {
  // ---------------- Recibir mensajes LoRa ----------------
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    String loraMsg = "";
    while (LoRa.available()) loraMsg += (char)LoRa.read();
    Serial.println("LoRa recibido: " + loraMsg);

    if (client.connected()) {
      // Enviar mensaje recibido a APRS-IS
      String aprsMsg = String(callsign) + ">APRS,TCPIP*:" + loraMsg;
      client.print(aprsMsg + "\r\n");
      Serial.println("Reenviado a APRS-IS: " + aprsMsg);
    }
  }

  // ---------------- Enviar beacon cada 30s ----------------
  static unsigned long lastBeacon = 0;
  if (millis() - lastBeacon > 30000 && client.connected()) {
    String beacon = String(callsign) + ">APRS,TCPIP*:!" +
                    beaconLatitude + "/" + beaconLongitude + "-" +
                    beaconMessage;
    client.print(beacon + "\r\n");
    Serial.println("Beacon enviado: " + beacon);
    lastBeacon = millis();
  }
}
```



## 6. Cronograma Preliminar

| Semana | Actividad / Objetivo | Avance Estimado |
|--------|--------------------|----------------|
| 5      | Configuración de entorno de programación y pruebas iniciales con la LilyGO T3. | ⚙️ En progreso |
| 6      | Desarrollo inicial del código: <br> - Recepción de paquetes LoRa <br> - Conexión a WiFi <br> - Estructura básica para envío a APRS-IS | ⚙️ En progreso |
| 7      | Integración hardware-software y pruebas de funcionamiento: <br> - Conectar antena LoRa <br> - Verificar envío de datos a APRS-IS <br> - Depurar errores iniciales | ⚙️ En progreso |
| 8      | **Entrega parcial (Informe y Presentación Parcial)**: <br> - Diagramas de bloques y máquina de estados <br> - Listado de hardware y justificación técnica <br> - Planteamiento estructurado del diseño <br> - Implementación en hardware y bus de conexión <br> - Tipo de comunicación de cada periférico <br> - Pseudo-código para control del sistema <br> - Definición de tramas de datos <br> - Cronograma y presupuesto para las semanas restantes <br> - Código inicial en GitHub documentado | ⚙️ En progreso |



