#include <lmic.h>
#include <hal/hal.h>
#define ANALOG_PIN 16
#include "utilities.h"
#define BUILTIN_LED BOARD_LED
#include "esp_system.h"
#include "esp_sleep.h"
#include "WiFi.h"
#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// Variables Globales
unsigned long lastReceivedSequenceNumber = 0;  // Último número de secuencia recibido
bool dataLost = false;  // Indicador de si se ha perdido algún dato

// Configuración BLE
String receivedValue = "";
bool deviceConnected = false;
bool bleDataValid = false;
unsigned long lastReceiveTime = 0;
unsigned long lastValidBleData = 0;
const unsigned long TIMEOUT_MS = 1000;
const unsigned long BLE_TIMEOUT = 15000;
static BLEAddress *pServerAddress;
static BLERemoteCharacteristic* txCharacteristic;

#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

// Variables BLE
uint8_t codigoNum = 0; // "codigo" ahora es un número (1 byte)
String campoAdicional = "";  // Para el campo "34" en los datos
String phoneType = "off";
String networkCountry = "off";
String networkOperator = "0";
String simCountryCode = "off";
String simOperator = "0";
String phoneCode = "0";
String networkType = "off";
int lteSignal = -999;

// Función para resetear valores BLE
void resetBLEValues() {
    codigoNum = 0;
    phoneType = "off";
    networkCountry = "off";
    networkOperator = "0";
    simCountryCode = "off";
    simOperator = "0";
    phoneCode = "0";
    networkType = "off";
    lteSignal = -999;
    bleDataValid = false;
    Serial.println("[BLE] Valores reseteados");
}

String getNextToken(String &data, int &lastIndex) {
    String token;
    int nextIndex = data.indexOf(';', lastIndex);
    if (nextIndex == -1) {
        token = data.substring(lastIndex);
        lastIndex = data.length();
    } else {
        token = data.substring(lastIndex, nextIndex);
        lastIndex = nextIndex + 1;
    }
    return token;
}

void processReceivedData(String &data) {
    Serial.println("[BLE] Datos crudos recibidos: " + data);

    int lastIndex = 0;
    int fieldCount = 0;
    String fields[10];

    while (lastIndex < data.length() && fieldCount < 10) {
        fields[fieldCount] = getNextToken(data, lastIndex);
        fieldCount++;
    }

    // Asignación corregida de campos
    codigoNum = fields[0].toInt(); // Convertimos a número (1 byte)
    phoneType = fields[1].length() > 0 ? fields[1] : "off";
    networkCountry = fields[2].length() > 0 ? fields[2] : "off";
    networkOperator = fields[3].length() > 0 ? fields[3] : "0";
    simCountryCode = fields[4].length() > 0 ? fields[4] : "off";
    simOperator = fields[5].length() > 0 ? fields[5] : "0";
    phoneCode = fields[6].length() > 0 ? fields[6] : "0";
    campoAdicional = fields[7].length() > 0 ? fields[7] : "";  // Nuevo campo
    networkType = fields[8].length() > 0 ? fields[8] : "off";
    lteSignal = fields[9].toInt();

    lastValidBleData = millis();
    bleDataValid = true;
    
    // Imprimir valores finales después de asignarlos
    Serial.println("[BLE] Datos BLE actualizados:");
    Serial.println("Codigo: " + String(codigoNum));
    Serial.println("Phone Type: " + phoneType);
    Serial.println("Network Country: " + networkCountry);
    Serial.println("Network Operator: " + networkOperator);
    Serial.println("SIM Country Code: " + simCountryCode);
    Serial.println("SIM Operator: " + simOperator);
    Serial.println("Phone Code: " + phoneCode);
    Serial.println("Network Type: " + networkType);
    Serial.println("LTE Signal: " + String(lteSignal));
}

class MyServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
        deviceConnected = true;
        Serial.println("Device connected");
        receivedValue = "";
        resetBLEValues();
    }

    void onDisconnect(BLEServer* pServer) {
        deviceConnected = false;
        Serial.println("Device disconnected");
        resetBLEValues();
        pServer->getAdvertising()->start();
    }
};

class MyCallbacks : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
        String rxValue = String(pCharacteristic->getValue().c_str());

        if (rxValue.length() > 0) {
            receivedValue += rxValue;
            lastReceiveTime = millis();

            // Extraer el número de secuencia de los últimos dos bytes
            unsigned long receivedSeqNumber = (rxValue[41] << 8) | rxValue[42];  // Últimos dos bytes

            // Comprobar si hay pérdida de secuencia
            if (receivedSeqNumber != lastReceivedSequenceNumber + 1) {
                dataLost = true;  // Si el número de secuencia no es consecutivo, los datos se han perdido
                Serial.print("[LoRa] ¡Datos perdidos! Esperaba el paquete número: ");
                Serial.println(lastReceivedSequenceNumber + 1);
            } else {
                // Si no hay pérdida de datos, muestra un mensaje
                dataLost = false;
                Serial.println("[LoRa] No se ha perdido ningún dato.");
            }

            // Actualizar el último número de secuencia recibido
            lastReceivedSequenceNumber = receivedSeqNumber;

            // Procesar el resto de los datos
            processReceivedData(rxValue);

            // Imprimir la cantidad de bytes recibidos y la trama de bytes
            Serial.print("[BLE] Bytes recibidos: ");
            Serial.println(rxValue.length());
            Serial.print("[BLE] Trama de bytes: ");
            for (int i = 0; i < rxValue.length(); i++) {
                Serial.print(rxValue[i], HEX);
                Serial.print(" ");
            }
            Serial.println();
        }
    }
};

// Configuración LoRaWAN
static const u1_t PROGMEM APPEUI[8]={0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}

static const u1_t PROGMEM DEVEUI[8]={ 0xAD, 0xCC, 0x06, 0xD0, 0x7E, 0xD5, 0xB3, 0x70 };
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}

static const u1_t PROGMEM APPKEY[16] = { 0x86, 0x65, 0x16, 0x04, 0xF7, 0x3D, 0x5A, 0x5F, 0xF5, 0x1B, 0x28, 0xE3, 0x4B, 0xCE, 0x4D, 0x9A };
void os_getDevKey (u1_t* buf) {  memcpy_P(buf, APPKEY, 16);}

static uint8_t mydata[] = "Hello, world!";
static osjob_t sendjob;
const unsigned TX_INTERVAL = 30;

// Pin mapping
#define RADIO_CS            RADIO_CS_PIN
#define RADIO_RESET         RADIO_RST_PIN
#define RADIO_DIO_0         RADIO_DIO0_PIN
#define RADIO_DIO_1         RADIO_DIO1_PIN
#define RADIO_DIO_2         LMIC_UNUSED_PIN

const lmic_pinmap lmic_pins = {
     .nss = RADIO_CS,
     .rxtx = LMIC_UNUSED_PIN,
     .rst = RADIO_RESET,
     .dio = {RADIO_DIO_0, RADIO_DIO_1, RADIO_DIO_2}  
};

// Variables LMIC
int16_t last_rssi = -128;
int8_t last_snr = -99;
unsigned long packetSequenceNumber = 0;  // Número de secuencia de los paquetes enviados

void onEvent(ev_t ev) {
    switch(ev) {
        case EV_TXCOMPLETE:
            digitalWrite(BUILTIN_LED, LOW);
            last_rssi = LMIC.rssi - 64;
            last_snr = LMIC.snr;
            os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);
            break;
        default:
            break;
    }
}

void do_send(osjob_t* j) {
    if (LMIC.opmode & OP_TXRXPEND) return;

    // Construir payload
    uint8_t mydata[43] = {0};  // 41 bytes + 2 para el número de secuencia
    uint8_t *ptr = mydata;

   // 1. Cabecera (9 bytes)
    int16_t rssi_signed = (int16_t)last_rssi;
    int16_t lte_signal_signed = (int16_t)(bleDataValid ? lteSignal : -999);

    *ptr++ = rssi_signed & 0xFF;          // Byte menos significativo
    *ptr++ = (rssi_signed >> 8) & 0xFF;   // Byte más significativo
    *ptr++ = last_snr;                    // Byte 2

    float voltage = analogRead(ANALOG_PIN) * (3.3 / 4095.0);  // Suponiendo 3.3V como referencia
    *ptr++ = (uint8_t)(voltage * 20);      // Multiplicar para mejor precisión
    *ptr++ = (int8_t)(temperatureRead() * 2); // Byte 4
    *ptr++ = ESP.getFreeHeap() / 1024;     // Byte 5 (RAM libre en KB)
    Serial.print("[RAM] Libre: ");
    Serial.print(ESP.getFreeHeap() / 1024);
    Serial.println(" KB");
    

    uint32_t uptime = millis() / 1000;
    *ptr++ = (uptime >> 8) & 0xFF;         // Byte 6-7 (Tiempo en segundos)
    *ptr++ = uptime & 0xFF;

    *ptr++ = (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_TIMER) ? 1 : 0; // Byte 8
    *ptr++ = codigoNum; // "codigo" ahora en 1 byte

    // 2. Campos BLE - Evitar caracteres nulos reemplazándolos con espacios
    auto copyField = [&ptr](const String &str, int len) {
        for (int i = 0; i < len; i++) {
            *ptr++ = (i < str.length()) ? str[i] : ' ';  // Rellenar con espacio en vez de \0
        }
    };

    copyField(phoneType, 6);    // 6 bytes
    copyField(networkCountry, 2); // 2 bytes
    copyField(networkOperator, 5); // 5 bytes
    copyField(simCountryCode, 6); // 6 bytes
    copyField(simOperator, 2);  // 2 bytes
    copyField(phoneCode, 5);    // 5 bytes
    copyField(networkType, 3);  // 3 bytes

    // LTE Signal ahora en formato correcto (int16_t)
    *ptr++ = lte_signal_signed & 0xFF;    // Byte menos significativo
    *ptr++ = (lte_signal_signed >> 8) & 0xFF; // Byte más significativo

    *ptr++ = (packetSequenceNumber >> 8) & 0xFF;  // Byte más significativo
    *ptr++ = packetSequenceNumber & 0xFF;         // Byte menos significativo

    // Enviar los datos
    LMIC_setTxData2(1, mydata, ptr - mydata, 1);

    // Imprimir la cantidad de bytes enviados y la trama de bytes
    Serial.print("[LoRa] Enviando datos: ");
    Serial.print(ptr - mydata);
    Serial.println(" bytes.");
    Serial.print("[LoRa] Trama de bytes: ");
    for (int i = 0; i < ptr - mydata; i++) {
        Serial.print(mydata[i], HEX);
        Serial.print(" ");
    }
    Serial.println();

    // Incrementar el número de secuencia para el siguiente paquete
    packetSequenceNumber++;
}


void setup() {
    Serial.begin(115200);
    initBoard();
    delay(1500);

    // Inicialización BLE
    BLEDevice::init("ESP32-BLE-Receiver");
    BLEServer *pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());
    
    BLEService *pService = pServer->createService(SERVICE_UUID);
    BLECharacteristic *pTx = pService->createCharacteristic(CHARACTERISTIC_UUID_TX, BLECharacteristic::PROPERTY_NOTIFY);
    pTx->addDescriptor(new BLE2902());
    
    BLECharacteristic *pRx = pService->createCharacteristic(CHARACTERISTIC_UUID_RX, BLECharacteristic::PROPERTY_WRITE);
    pRx->setCallbacks(new MyCallbacks());
    
    pService->start();
    pServer->getAdvertising()->start();
    String macAddress = BLEDevice::getAddress().toString().c_str();
    Serial.println("Dirección MAC del ESP32 receptor: " + macAddress);

    // Inicialización LoRaWAN
    os_init();
    LMIC_reset();
    LMIC_setAdrMode(false);
    do_send(&sendjob);
    pinMode(BUILTIN_LED, OUTPUT);
}

void loop() {
    os_runloop_once();

    // Manejo de datos BLE
    if (receivedValue.length() > 0 && (millis() - lastReceiveTime > TIMEOUT_MS)) {
        if (receivedValue.endsWith(";")) {
            processReceivedData(receivedValue);
        } else {
            Serial.println("[BLE] Mensaje incompleto descartado");
        }
        receivedValue = "";
    }

    // Verificación periódica de estado BLE
    static unsigned long lastCheck = 0;
    if (millis() - lastCheck > 1000) {
        lastCheck = millis();
        if (bleDataValid && (millis() - lastValidBleData > BLE_TIMEOUT)) {
            Serial.println("[BLE] Sin datos recientes - Marcando como offline");
            resetBLEValues();
        }
    }
}


