# ESP32gateway
Este proyecto implementa un nodo IoT utilizando una placa **ESP32-S3** capaz de:
- Recibir datos vía **Bluetooth Low Energy (BLE)** desde un dispositivo móvil o una segunda placa ESP32.
- Obtener sus propios datos internos (temperatura, RAM libre, voltaje, etc.).
- Empaquetar toda la información en un payload compacto y enviarla a través de **LoRaWAN** usando la infraestructura de **The Things Network (TTN)**.
- Visualizar los datos en tiempo real en **ThingsBoard**.

## Características principales

- Comunicación BLE con:
  - Dispositivo móvil (Android) mediante AppInventor.
  - Segundo ESP32 sensor (opcional).
- Sensado interno:
  - Temperatura del microcontrolador.
  - RAM libre.
  - Voltaje de entrada (ADC).
  - Uptime (tiempo de funcionamiento).
  - Modo de energía actual.
- Comunicación LoRaWAN (OTAA) con TTN.
- Envío de paquetes binarios de 43 bytes cada 30 segundos.
- Formato compacto optimizado para regiones EU868.

## Hardware requerido

- Placa **LILYGO T3-S3 v1.0** o compatible con ESP32-S3 y LoRa.
- Antena LoRa 868 MHz.
- Fuente de alimentación (USB o batería LiPo).

## Librerías necesarias

Instalar en el entorno Arduino IDE:

- `LMIC` (MCCI LoRaWAN)
- `BLEDevice` (incluido con el core de ESP32)
- `esp_system`, `esp_sleep`, `esp_wifi` (de ESP-IDF)
- Otras utilidades como `WiFi.h`, `Arduino.h`, etc.

## Parámetros LoRaWAN

Modificar las claves en el código:

```cpp
static const u1_t PROGMEM DEVEUI[8] = {...};
static const u1_t PROGMEM APPEUI[8] = {...};
static const u1_t PROGMEM APPKEY[16] = {...};

Se utiliza el protocolo OTAA (Over-The-Air Activation).

🔄 Flujo de funcionamiento
El ESP32 establece conexión BLE.
Recibe datos en formato delimitado por ;.
Procesa los campos y los combina con sensores internos.
Construye un payload binario de 43 bytes.
Envía el payload vía LoRaWAN a TTN cada 30 segundos.
TTN reenvía los datos a ThingsBoard vía integración HTTP/MQTT.

 Observaciones
El payload fue optimizado para ajustarse al límite de 51 bytes permitido por TTN en EU868.
La RAM libre se mantiene estable (~191 KB), indicando buena gestión de memoria.
BLE y LoRa no interfieren significativamente entre sí en la transmisión.

