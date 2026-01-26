#ifndef BLE_H
#define BLE_H
#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#define SERVICE_UUID "a15b89c6-1042-4c05-af06-52bb41e51c1e"
#define CHARACTERISTIC_UUID "a15b89c6-1042-4c05-af06-52bb41e51c1e"

class BLE
{
public:
    void initBLE(void);
    void reconnect(void);
    uint8_t recieved_data[4];
    int8_t *send_data;
    BLEServer *pServer;
    BLEService *pService;
    BLECharacteristic *pCharacteristic;
    bool _BLEClientConnected = false;
    uint8_t state_data[162];
    void (*act)(void);
    QueueHandle_t bleQueue = xQueueCreate(10, sizeof(char[20]));
    ;

    class MyServerCallbacks : public BLEServerCallbacks
    {
    public:
        void onConnect(BLEServer *pServer);
        void onDisconnect(BLEServer *pServer);
    };
    class MyCallbacks : public BLECharacteristicCallbacks
    {
        void onWrite(BLECharacteristic *pCharacteristic);
        // void onRead(BLECharacteristic *pCharacteristic);
    };

    void send_state_buffer(uint8_t *state_buffer);
};

extern BLE postman;
#endif