#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/stream_buffer.h"

#include "esp_task_wdt.h"

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#include "motor_control.h"
#include "lidar.h"
#include "esp_mac.h"

// ================== BLE UUIDs ==================
#define SERVICE_UUID "a15b89c6-1042-4c05-af06-52bb41e51c1e"
#define CHARACTERISTIC_UUID "a15b89c6-1042-4c05-af06-52bb41e51c1e"

// ================== Core assignments ==================
#define LIDAR_TASK_CORE 1 // LIDAR task on core 1
#define MOTOR_TASK_CORE 1 // Motor task on core 1
#define BLE_TASK_CORE 0   // BLE on core 0

// ================== LIDAR settings ==================
#define LIDAR_TX_PIN 10
#define LIDAR_RX_PIN 9
#define STREAM_BUF_SIZE 4096
#define TRIGGER_LEVEL 1

// Mỗi điểm LIDAR: 2 byte angle + 2 byte distance + 1 byte quality
#define LIDAR_POINT_SIZE 7

// ================== HC-12 Serial ==================
#define HC12_TX_PIN 22
#define HC12_RX_PIN 21
HardwareSerial HC12(2); // UART1 cho HC-12

// ================== Types ==================
typedef struct
{
  float v;
  float w;
  uint16_t dis;
  uint16_t dir;
} ManualCommand_t;

// ================== Global variables ==================
QueueHandle_t manualCommandQueue = NULL;
QueueHandle_t uartQueue = NULL;
SemaphoreHandle_t motorCommandMutex = NULL;
StreamBufferHandle_t lidarStream = NULL;

TaskHandle_t sendTaskHandle = NULL;
TaskHandle_t recieveTaskHandle = NULL;
TaskHandle_t simpleMotorTaskHandle = NULL;
TaskHandle_t manualMotorTaskHandle = NULL;
TaskHandle_t lidarTaskHandle = NULL;

BLEServer *pServer = nullptr;
BLECharacteristic *pCharacteristic = nullptr;
BLEService *pService = nullptr;

float v = 0.0f; // linear wheel speed
float w = 0.0f; // angular wheel speed
uint16_t dis = 0;
uint16_t dir = 0;

bool deviceConnected = false;
bool isAdvertising = false;
bool manual_drive_state = false;

float global_angle = 0.0f;
float global_distance = 0.0f;
float global_quality = 0.0f;

// ================== BLE Callbacks ==================
class MyServerCallbacks : public BLEServerCallbacks
{
  void onConnect(BLEServer *pServer) override
  {
    deviceConnected = true;
    isAdvertising = false;
    Serial.println("Device connected, stopping advertising");
  }

  void onDisconnect(BLEServer *pServer) override
  {
    deviceConnected = false;
    isAdvertising = false;
    pServer->startAdvertising();
    Serial.println("Device disconnected, restarting advertising");
  }
};

// ================== BLE Init ==================
void BLE_begin()
{
  BLEDevice::init("Quadrup");
  BLEDevice::setMTU(517);

  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  pService = pServer->createService(SERVICE_UUID);

  pCharacteristic = pService->createCharacteristic(
      CHARACTERISTIC_UUID,
      BLECharacteristic::PROPERTY_READ |
          BLECharacteristic::PROPERTY_WRITE |
          BLECharacteristic::PROPERTY_NOTIFY |
          BLECharacteristic::PROPERTY_INDICATE);

  pCharacteristic->addDescriptor(new BLE2902());
}

// ================== LIDAR Task ==================
void lidarTask(void *parameter)
{
  esp_task_wdt_add(NULL);

  while (true)
  {
    esp_task_wdt_reset();

    // Lấy 1 điểm từ LIDAR wrapper
    coordinate point = rlidar.coordinate_take();
    // coordinate point = simulateLidar();

    if (point.distance > 0 &&
        point.distance < 7000 &&
        point.angle >= 0 &&
        point.angle < 360)
    {
      // angle: lưu với độ phân giải 0.1°
      uint16_t ang = (uint16_t)roundf(point.angle * 10.0f);
      // distance: mm
      uint16_t dist = (uint16_t)roundf(point.distance);
      // quality: 0–20 trong 1 byte
      float q_raw = point.quality;
      if (q_raw < 0.0f)
        q_raw = 0.0f;
      if (q_raw > 20.0f)
        q_raw = 20.0f;
      uint8_t qual = (uint8_t)(q_raw);

      // cập nhật biến global cho debug
      global_angle = point.angle;
      global_distance = point.distance;
      global_quality = q_raw;

      // Packet: [ang_L, ang_H, dist_L, dist_H, qual]
      uint8_t pkt[LIDAR_POINT_SIZE];
      pkt[0] = 0xAA;                         // start of frame
    pkt[1] = 0x01;                         // type = LIDAR point
    pkt[2] = (uint8_t)(ang & 0xFF);
    pkt[3] = (uint8_t)((ang >> 8) & 0xFF);
    pkt[4] = (uint8_t)(dist & 0xFF);
    pkt[5] = (uint8_t)((dist >> 8) & 0xFF);
    pkt[6] = qual;

      // Gửi 7 byte vào stream buffer (block tối đa 5ms nếu đầy)
      xStreamBufferSend(lidarStream, pkt, LIDAR_POINT_SIZE, pdMS_TO_TICKS(5));
    }

    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

// ================== BLE Send Task ==================
void sendTask(void *parameter)
{
  esp_task_wdt_add(NULL);

  Serial.print("BLE Task running on core ");
  Serial.println(xPortGetCoreID());

  const TickType_t xFrequency = pdMS_TO_TICKS(100);
  TickType_t xLastWakeTime = xTaskGetTickCount();

  uint8_t recv_buf[512];

  // Bộ đệm tích lũy để đảm bảo gửi đúng bội số LIDAR_POINT_SIZE
  static uint8_t accum[512];
  static size_t accumLen = 0;

  uint32_t pointsSent = 0;
  uint32_t bytesSent = 0;

  while (true)
  {
    esp_task_wdt_reset();

    if (deviceConnected)
    {
      // Đọc dữ liệu mới từ stream buffer
      size_t n = xStreamBufferReceive(lidarStream, recv_buf, sizeof(recv_buf), 0);
      if (n > 0)
      {
        // Chống tràn accum
        if (n > sizeof(accum) - accumLen)
        {
          n = sizeof(accum) - accumLen;
        }

        if (n > 0)
        {
          memcpy(accum + accumLen, recv_buf, n);
          accumLen += n;

          // Số byte hợp lệ là bội số của kích thước 1 điểm
          size_t valid = (accumLen / LIDAR_POINT_SIZE) * LIDAR_POINT_SIZE;

          if (valid >= LIDAR_POINT_SIZE)
          {
            uint32_t pointsInBuffer = valid / LIDAR_POINT_SIZE;
            pointsSent += pointsInBuffer;
            bytesSent += valid;

            const size_t maxChunkSize = 200;
            size_t offset = 0;

            while (offset < valid)
            {
              size_t chunk = (valid - offset > maxChunkSize)
                                 ? maxChunkSize
                                 : (valid - offset);

              pCharacteristic->setValue(accum + offset, chunk);
              pCharacteristic->notify();

              offset += chunk;
            }

            // Giữ lại phần dư chưa đủ 1 điểm
            size_t leftover = accumLen - valid;
            if (leftover > 0)
            {
              memmove(accum, accum + valid, leftover);
            }
            accumLen = leftover;
          }
        }
      }
    }

    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

// ================== HC-12 Receive Task ==================
void recieveTask(void *parameter)
{
  esp_task_wdt_add(NULL);

  Serial.print("Receive Task running on core ");
  Serial.println(xPortGetCoreID());

  static uint8_t buffer[10];
  static uint8_t index = 0;
  static uint32_t lastByteTime = 0;
  static uint32_t lastPacketTime = 0;

  const uint32_t BYTE_TIMEOUT = 100;   // ms
  const uint32_t PACKET_TIMEOUT = 200; // ms

  while (true)
  {
    esp_task_wdt_reset();

    if (HC12.available() == 0)
    {
      if (millis() - lastPacketTime > PACKET_TIMEOUT)
      {
        if (xSemaphoreTake(motorCommandMutex, pdMS_TO_TICKS(10)) == pdTRUE)
        {
          v = 0.0f;
          w = 0.0f;
          if(!manual_drive_state)
          {
            dis = 0;
            dir = 0;
          }
          xSemaphoreGive(motorCommandMutex);
        }
      }

      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }

    while (HC12.available())
    {
      uint8_t byteIn = HC12.read();
      uint32_t now = millis();

      if (index > 0 && (now - lastByteTime > BYTE_TIMEOUT))
      {
        index = 0;
      }

      lastByteTime = now;

      if (index == 0 && byteIn != 0xAA)
      {
        continue;
      }

      buffer[index++] = byteIn;

      if (index == 10)
      {
        if (buffer[0] == 0xAA && buffer[9] == 0x55)
        {
          uint16_t distance = buffer[1] | (buffer[2] << 8);
          uint16_t direction = buffer[3] | (buffer[4] << 8);

          int16_t w_raw = (int16_t)(buffer[5] | (buffer[6] << 8));
          int16_t v_raw = (int16_t)(buffer[7] | (buffer[8] << 8));

          float v_float = v_raw / 10.0f;
          float w_float = w_raw / 10.0f;

          ManualCommand_t cmd = {v_float, w_float, distance, direction};

          bool current_manual = (distance != 0 || direction != 0);

          if (xSemaphoreTake(motorCommandMutex, pdMS_TO_TICKS(10)) == pdTRUE)
          {
            v = v_float;
            w = w_float;
            dis = distance;
            dir = direction;

            if (!current_manual)
            {
              dis = 0;
              dir = 0;
              turn_done = false;
              move_done = false;
            }

            xSemaphoreGive(motorCommandMutex);
          }

          if (current_manual && manualMotorTaskHandle != NULL && !manual_drive_state)
          {
            if (xQueueSend(manualCommandQueue, &cmd, pdMS_TO_TICKS(10)) == pdPASS)
            {
              Serial.println("[HC12] Manual command queued");
            }
            else
            {
              Serial.println("[HC12] Manual queue FULL, drop cmd");
            }
          }
          lastPacketTime = now;
        }
        else
        {
          Serial.println("Invalid packet");
        }

        index = 0;
      }
    }

    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

// ================== Simple Motor Task ==================
void simpleMotorTask(void *parameter)
{
  esp_task_wdt_add(NULL);

  const TickType_t xFrequency = pdMS_TO_TICKS(100);
  TickType_t xLastWakeTime = xTaskGetTickCount();

  while (true)
  {
    esp_task_wdt_reset();

    motor_run(true);

    if (xSemaphoreTake(motorCommandMutex, pdMS_TO_TICKS(10)) == pdTRUE)
    {
      float local_v = v;
      float local_w = w;
      bool local_manual = manual_drive_state;
      xSemaphoreGive(motorCommandMutex);

      if (!local_manual)
      {
        motor_speed_set(local_v, local_w);
      }
    }
    else
    {
      motor_speed_set(0, 0);
    }

    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

// ================== Manual Motor Task (hiện đang không tạo trong setup) ==================
void manualMotorTask(void *parameter)
{
  ManualCommand_t cmd;

  while (true)
  {
    if (xQueueReceive(manualCommandQueue, &cmd, portMAX_DELAY) == pdPASS)
    {
      Serial.println("[MANUAL TASK] Command received");

      // Bật cờ manual
      if (xSemaphoreTake(motorCommandMutex, pdMS_TO_TICKS(100)) == pdTRUE)
      {
        manual_drive_state = true;
        // (tuỳ chọn) copy sang global để debug
        dis = cmd.dis;
        dir = cmd.dir;
        v   = cmd.v;
        w   = cmd.w;
        xSemaphoreGive(motorCommandMutex);
      }

      // 1) Quay nếu có góc
      if (cmd.dir != 0)
      {
        turn_angle(cmd.dir, cmd.w);
      }

      // 2) Chạy thẳng nếu có quãng đường
      if (cmd.dis != 0)
      {
        move_straight(cmd.dis, cmd.v);
      }

      // 3) Dừng hẳn motor
      motor_speed_set(0, 0);

      // Tắt cờ manual, reset biến
      if (xSemaphoreTake(motorCommandMutex, pdMS_TO_TICKS(100)) == pdTRUE)
      {
        dis = 0;
        dir = 0;
        v   = 0;
        w   = 0;
        manual_drive_state = false;
        xSemaphoreGive(motorCommandMutex);
      }

      turn_done = false;
      move_done = false;

      Serial.println("[MANUAL TASK] Done.");
    }
  }
}

// ================== Setup ==================
void setup()
{
  Serial.begin(115200);

  HC12.begin(9600, SERIAL_8N1, HC12_RX_PIN, HC12_TX_PIN);

  esp_task_wdt_init(5, true);

  uartQueue = xQueueCreate(10, 4 * sizeof(uint8_t));
  manualCommandQueue = xQueueCreate(5, sizeof(ManualCommand_t));
  motorCommandMutex = xSemaphoreCreateMutex();
  lidarStream = xStreamBufferCreate(STREAM_BUF_SIZE, TRIGGER_LEVEL);

  if (manualCommandQueue == NULL)
    Serial.println("Failed to create manualCommandQueue");
  if (motorCommandMutex == NULL)
    Serial.println("Failed to create motor command mutex");
  if (lidarStream == NULL)
    Serial.println("Failed to create LIDAR stream buffer");

  // BLE init
  BLE_begin();
  pService->start();
  pServer->startAdvertising();
  isAdvertising = true;
  Serial.println("BLE advertising started");

  // LIDAR init
  rlidar.begin();
  rlidar.lidar_run();

  // Motor init
  motor_begin();
  motor_speed_set(0, 0);
  motor_run(true);
  manual_drive_state = false;

  // Tasks
  xTaskCreatePinnedToCore(sendTask,
                          "Send Task",
                          4096,
                          NULL,
                          5,
                          &sendTaskHandle,
                          BLE_TASK_CORE);

  xTaskCreatePinnedToCore(recieveTask,
                          "Receive Task",
                          4096,
                          NULL,
                          2,
                          &recieveTaskHandle,
                          BLE_TASK_CORE);

  xTaskCreatePinnedToCore(simpleMotorTask,
                          "Motor Task",
                          4096,
                          NULL,
                          6,
                          &simpleMotorTaskHandle,
                          MOTOR_TASK_CORE);

  xTaskCreatePinnedToCore(manualMotorTask,
                          "Manual Motor Task",
                          4096,
                          NULL,
                          3,
                          &manualMotorTaskHandle,
                          MOTOR_TASK_CORE);

  xTaskCreatePinnedToCore(lidarTask,
                          "LIDAR Task",
                          4096,
                          NULL,
                          4,
                          &lidarTaskHandle,
                          LIDAR_TASK_CORE);
}

// ================== Main loop ==================
void loop()
{
  esp_task_wdt_reset();

  // Bật lại nếu muốn debug:
  // Serial.print("LIDAR Point - Angle: ");
  // Serial.print(global_angle);
  // Serial.print("°, Distance: ");
  // Serial.print(global_distance);
  // Serial.print(" mm, Quality: ");
  // Serial.println(global_quality);
  // Serial.print("Dir 11: ");
  // Serial.print(digitalRead(dir11) ? "HIGH" : "LOW");
  // Serial.print("\nDir 12: ");
  // Serial.print(digitalRead(dir12) ? "HIGH" : "LOW");
  // Serial.print("\nDir 21: ");
  // Serial.print(digitalRead(dir21) ? "HIGH" : "LOW");
  // Serial.print("\nDir 22: ");
  // Serial.println(digitalRead(dir22) ? "HIGH" : "LOW");

  vTaskDelay(pdMS_TO_TICKS(1000));
}
