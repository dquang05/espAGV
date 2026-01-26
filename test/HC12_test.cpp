#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_task_wdt.h"
#include "freertos/stream_buffer.h"

// #include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#include "motor_control.h"
#include "lidar.h"
#include "esp_mac.h"
// UUIDs for BLE service and characteristic
#define SERVICE_UUID "a15b89c6-1042-4c05-af06-52bb41e51c1e"
#define CHARACTERISTIC_UUID "a15b89c6-1042-4c05-af06-52bb41e51c1e"

// Core assignments for FreeRTOS tasks
#define LIDAR_TASK_CORE 1 // LIDAR task on core 1 to balance load
#define MOTOR_TASK_CORE 1 // Motor task on core 1
#define BLE_TASK_CORE 0   // BLE on core 0 to avoid conflict with WiFi/BT core

// LIDAR settings
#define LIDAR_TX_PIN 10
#define LIDAR_RX_PIN 9
#define STREAM_BUF_SIZE 4096 // Buffer size (depends on LIDAR speed and BLE throughput)
#define TRIGGER_LEVEL 1      // Unblock when there is at least 1 byte in the buffer

// HC-12 Serial Communication
#define HC12_TX_PIN 22
#define HC12_RX_PIN 21
HardwareSerial HC12(2); // Use UART1 for HC-12

// Global variables
TaskHandle_t sendTaskHandle = NULL;
TaskHandle_t recieveTaskHandle = NULL;
TaskHandle_t motorTaskHandle = NULL;     // Handle for motor task (unused)
TaskHandle_t lidarTaskHandle = NULL;     // Handle for LIDAR task
StreamBufferHandle_t lidarStream = NULL; // Handle for LIDAR stream buffer

float v = 0; // Linear wheel speed
float w = 0; // Angular wheel speed
uint16_t dis = 0, dir = 0;

BLEServer *pServer = nullptr;
BLECharacteristic *pCharacteristic = nullptr;
BLEService *pService = nullptr;

// Flags
bool deviceConnected = false;
bool isAdvertising = false;
bool motor_state = false;

// Queues and mutexes
QueueHandle_t uartQueue;
SemaphoreHandle_t motorCommandMutex = NULL; // Mutex for motor commands

// BLE server callback handlers
class MyServerCallbacks : public BLEServerCallbacks
{
  void onConnect(BLEServer *pServer)
  {
    deviceConnected = true;
    isAdvertising = false; // Stop advertising when connected
    Serial.println("Device connected, stopping advertising");
  }

  void onDisconnect(BLEServer *pServer)
  {
    deviceConnected = false;
    isAdvertising = false;       // Stop advertising when disconnected
    pServer->startAdvertising(); // Restart advertising
    Serial.println("Device disconnected, restarting advertising");
  }
};

// class MyCallbacks : public BLECharacteristicCallbacks
// {
//   void onWrite(BLECharacteristic *pCharacteristic)
//   {
//     std::string value = pCharacteristic->getValue();
//     if (value.length() == 4 && bleQueue != nullptr && value[0] == 48 && value[3] == 48)
//     { // Check if value is 4 bytes long and starts/ends with 48
//       uint8_t buffer[4];
//       memcpy(buffer, value.data(), 4); // Copy 4 bytes: [48, w, v, 48]

//       xQueueSend(bleQueue, buffer, 0);
//     }
//   }
// };

// BLE task for sending LIDAR data via notifications
void sendTask(void *parameter)
{
  // Register this task with the watchdog
  esp_task_wdt_add(NULL);

  Serial.print("BLE Task running on core ");
  Serial.println(xPortGetCoreID());

  TickType_t xLastWakeTime;
  const TickType_t xFrequency = pdMS_TO_TICKS(100); // periodic interval
  xLastWakeTime = xTaskGetTickCount();
  uint8_t recv_buf[512];

  // Transmission stats
  uint32_t pointsSent = 0; // total points sent
  uint32_t bytesSent = 0;  // total bytes sent

  while (1)
  {
    // Reset watchdog timer
    esp_task_wdt_reset();

    if (deviceConnected == true)
    {
      // Read available data from stream buffer
      size_t n = xStreamBufferReceive(lidarStream, recv_buf, sizeof(recv_buf), 0);
      if (n > 0)
      {
        // Each LIDAR point = 4 bytes (angle + distance)
        uint32_t pointsInBuffer = n / 4;
        pointsSent += pointsInBuffer;
        bytesSent += n;

        size_t maxChunkSize = 200; // adjust if needed (MTU considerations)
        size_t offset = 0;
        while (offset < n)
        {
          size_t chunk = (n - offset > maxChunkSize) ? maxChunkSize : (n - offset);
          pCharacteristic->setValue(recv_buf + offset, chunk);
          pCharacteristic->notify();
          offset += chunk;
        }
      }
    }
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

void recieveTask(void *parameter)
{
  esp_task_wdt_add(NULL);
  Serial.print("Receive Task running on core ");
  Serial.println(xPortGetCoreID());

  static uint8_t buffer[4];
  static uint8_t index = 0;
  static uint32_t lastByteTime = 0;
  static uint32_t lastPacketTime = 0;

  const uint32_t BYTE_TIMEOUT = 100;   // ms - timeout giữa các byte trong cùng gói
  const uint32_t PACKET_TIMEOUT = 500; // ms - timeout mất tín hiệu tổng thể

  while (1)
  {
    esp_task_wdt_reset();

    // Nếu không có dữ liệu, kiểm tra timeout tổng
    if (HC12.available() == 0)
    {
      if (millis() - lastPacketTime > PACKET_TIMEOUT)
      {
        if (xSemaphoreTake(motorCommandMutex, pdMS_TO_TICKS(10)) == pdTRUE)
        {
          v = 0.0f;
          w = 0.0f;
          xSemaphoreGive(motorCommandMutex);
        }
      }
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }

    // Có dữ liệu -> đọc
    while (HC12.available())
    {
      uint8_t byteIn = HC12.read();
      uint32_t now = millis();

      // Kiểm tra timeout giữa các byte (mất nhịp)
      if (index > 0 && (now - lastByteTime > BYTE_TIMEOUT))
      {
        index = 0; // reset buffer
      }

      lastByteTime = now;

      // Kiểm tra byte đầu tiên phải là 0xAA
      if (index == 0 && byteIn != 0xAA)
        continue;

      buffer[index++] = byteIn;

      // Nếu đủ 4 byte → xử lý
      if (index == 4)
      {
        if (buffer[0] == 0xAA && buffer[3] == 0x55)
        {
          uint8_t rw = buffer[1];
          uint8_t rv = buffer[2];

          if (xSemaphoreTake(motorCommandMutex, pdMS_TO_TICKS(10)) == pdTRUE)
          {
            v = (rv - 80);
            w = (rw - 80);
            xSemaphoreGive(motorCommandMutex);
          }

          lastPacketTime = now; // cập nhật thời gian nhận gói hợp lệ
        }
        else
        {
          Serial.println("Invalid packet");
        }
        index = 0; // reset sau khi xử lý
      }
    }

    vTaskDelay(pdMS_TO_TICKS(5));
  }
}


void BLE_begin()
{
  BLEDevice::init("Quadrup");
  BLEDevice::setMTU(517); // Set MTU size to 517 bytes
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  pService = pServer->createService(SERVICE_UUID);
  pCharacteristic = pService->createCharacteristic(
      CHARACTERISTIC_UUID,
      // Reduced properties for compatibility
      BLECharacteristic::PROPERTY_READ |
          BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_INDICATE);

  // pCharacteristic->setCallbacks(new MyCallbacks());
  pCharacteristic->addDescriptor(new BLE2902());
}

// Task to read LIDAR data and push to stream buffer
void lidarTask(void *parameter)
{
  // Register this task with the watchdog
  esp_task_wdt_add(NULL);

  while (1)
  {
    // Reset watchdog
    esp_task_wdt_reset();

    // Read a coordinate from the RPLIDAR wrapper
    coordinate point = rlidar.coordinate_take();
    // coordinate point = simulateLidar();
    if (point.distance > 0 && point.distance < 7000 &&
        point.angle >= 0 && point.angle < 360)
    {
      // Each point: 4 bytes {angle(uint16_t), distance(uint16_t)}
      uint16_t ang = (uint16_t)round(point.angle * 10);
      uint16_t dist = (uint16_t)round(point.distance);

      uint8_t pkt[4];
      pkt[0] = ang & 0xFF;
      pkt[1] = ang >> 8;
      pkt[2] = dist & 0xFF;
      pkt[3] = dist >> 8;

      // Send into the stream buffer, block up to 5ms if buffer is full
      xStreamBufferSend(lidarStream, pkt, sizeof(pkt), pdMS_TO_TICKS(5));
    }

    vTaskDelay(pdMS_TO_TICKS(1)); // short delay for LIDAR sampling
  }
}

void motorTask(void *parameter)
{
  esp_task_wdt_add(NULL); // Register task with watchdog
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(10);
  while (1)
  {
    esp_task_wdt_reset();

    motor_state = true; // Mark motors as active
    motor_run(true);    // Ensure motors are enabled
    // speed_control();
    // Protect motor command variables with mutex
    if (xSemaphoreTake(motorCommandMutex, pdMS_TO_TICKS(10)) == pdTRUE)
    {
      float local_v = v;
      float local_w = w;
      xSemaphoreGive(motorCommandMutex);
      motor_speed_set(local_v, local_w); // Apply velocity commands
      // Serial.print("v = ");
      // Serial.print(local_v);
      // Serial.print(", w = ");
      // Serial.println(local_w);
    }
    else
    {
      motor_speed_set(0, 0); // Stop motors if mutex not available
    }

    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

// Setup function
void setup()
{
  // pinMode(23, OUTPUT);
  Serial.begin(115200); // Initialize serial
  HC12.begin(9600, SERIAL_8N1, HC12_RX_PIN, HC12_TX_PIN); // Initialize HC-12 UART
  esp_task_wdt_init(5, true);                                        // 5-second watchdog with panic
  uartQueue = xQueueCreate(10, 4*sizeof(uint8_t));
  motorCommandMutex = xSemaphoreCreateMutex();                       // Create mutex for motor commands
  lidarStream = xStreamBufferCreate(STREAM_BUF_SIZE, TRIGGER_LEVEL); // Create stream buffer for LIDAR data

  if (motorCommandMutex == NULL)
  {
    Serial.println("Failed to create motor command mutex");
  }
  if (lidarStream == NULL)
  {
    Serial.println("Failed to create LIDAR stream buffer");
  }

  // Initialize BLE
  BLE_begin();
  pService->start();
  pServer->startAdvertising();
  isAdvertising = true;
  Serial.println("BLE advertising started");

  // Initialize LIDAR
  rlidar.begin();
  rlidar.lidar_run(); // Start LIDAR motor

  motor_begin();         // Initialize motors
  motor_speed_set(0, 0); // Set initial motor speed
  motor_run(true);       // Enable motors
  motor_state = true;    // Mark motors as active

  // Create only the receive task here (other tasks can be created as needed)
  xTaskCreatePinnedToCore(sendTask, "Send Task", 4096, NULL, 5, &sendTaskHandle, BLE_TASK_CORE);
  xTaskCreatePinnedToCore(recieveTask, "Receive Task", 4096, NULL, 2, &recieveTaskHandle, BLE_TASK_CORE);
  xTaskCreatePinnedToCore(motorTask, "Motor Task", 4096, NULL, 3, &motorTaskHandle, MOTOR_TASK_CORE);
  xTaskCreatePinnedToCore(lidarTask, "LIDAR Task", 4096, NULL, 4, &lidarTaskHandle, LIDAR_TASK_CORE);
}

// Main loop
void loop()
{
  esp_task_wdt_reset(); // Reset watchdog
  // Serial.print("[LOG] v="); Serial.print(v);
  // Serial.print(" w="); Serial.print(w);
  // Serial.printf("set_speed1: %.2f\n", set_speed1);
  // Serial.printf("cur_speed1: %.2f\n", cur_speed1);
  // Serial.printf("set_speed2: %.2f\n", set_speed2);
  // Serial.printf("cur_speed2: %.2f\n", cur_speed2);
    // Serial.printf("dir11: %d, dir12: %d, dir21: %d, dir22: %d\n",
    //               digitalRead(dir11),
    //               digitalRead(dir12),
    //               digitalRead(dir21),
    //               digitalRead(dir22));
  // Serial.print("ISR Count: ");
  // Serial.println(isrCounter);
  // Serial.printf("PID1 out: %.2f, PID2 out: %.2f\n", pid_ctr1.getOutput(), pid_ctr2.getOutput());
  

  vTaskDelay(pdMS_TO_TICKS(500)); // Yield CPU
}