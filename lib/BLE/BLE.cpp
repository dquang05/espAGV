#include "BLE.h"
#include <stdio.h>
#include <Arduino.h>
#include "freertos/queue.h"

BLE postman;

void BLE::MyServerCallbacks::onConnect(BLEServer *pServer)
{
    postman._BLEClientConnected = true;
}

void BLE::MyServerCallbacks::onDisconnect(BLEServer *pServer)
{
    postman._BLEClientConnected = false;
}

void BLE::initBLE()
{
    BLEDevice::init("Quadrup");
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks);

    pService = pServer->createService(SERVICE_UUID);
    pCharacteristic = pService->createCharacteristic(
        CHARACTERISTIC_UUID,
        BLECharacteristic::PROPERTY_READ |
            BLECharacteristic::PROPERTY_NOTIFY |
            BLECharacteristic::PROPERTY_WRITE);
    pCharacteristic->setCallbacks(new MyCallbacks());
    pCharacteristic->addDescriptor(new BLE2902());

    pService->start();

    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    pAdvertising->setMinPreferred(0x06); // functions that help with iPhone connections issue
    pAdvertising->setMinPreferred(0x12);
    BLEDevice::startAdvertising();
}

void BLE::reconnect()
{
    pService->start();
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->setScanResponse(true);
    pAdvertising->setMinPreferred(0x06); // functions that help with iPhone connections issue
    pAdvertising->setMinPreferred(0x12);
    BLEDevice::startAdvertising();
}

void BLE::MyCallbacks::onWrite(BLECharacteristic *pCharacteristic)
{
    std::string value = pCharacteristic->getValue();
    if (value.length() > 0)
    {
        // Send received data to queue
        if (postman.bleQueue != nullptr)
        {
            xQueueSend(postman.bleQueue, value.c_str(), 0);
        }
    }
}

// void BLE::MyCallbacks::onRead(BLECharacteristic *pCharacteristic) {
//     postman.pCharacteristic->setValue(postman.state_data,sizeof(postman.state_data));
//     postman.pCharacteristic->notify();
// }

void BLE::send_state_buffer(uint8_t *state_buffer)
{
    postman.pCharacteristic->setValue(state_buffer, 162);
    postman.pCharacteristic->notify();
}