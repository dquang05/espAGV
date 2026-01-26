#include <Arduino.h>

// FreeRTOS
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

// ESP-IDF
#include "esp_task_wdt.h"

// Your libs
#include "myPcnt.h"
#include "motor_control.h"
#include "lidar.h"
#include "pid.h"          // để gọi pid_set_control_task_handle()

// ================== Core assignments ==================
#define MOTOR_TASK_CORE 1
#define RX_TASK_CORE    0

// ================== HC-12 Serial ==================
#define HC12_TX_PIN 22
#define HC12_RX_PIN 21
HardwareSerial HC12(2);

// ================== Manual Command ==================
typedef struct
{
  float v_mm_s;      // mm/s (có thể âm)
  float w_deg_s;     // deg/s (có thể âm)
  int16_t dist_mm;   // mm (có thể âm)
  int16_t angle_deg; // deg (có thể âm)
} ManualCommand_t;

// ================== Globals ==================
static QueueHandle_t manualCommandQueue = NULL;
static SemaphoreHandle_t motorCommandMutex = NULL;

static TaskHandle_t recieveTaskHandle = NULL;
static TaskHandle_t simpleMotorTaskHandle = NULL;
static TaskHandle_t manualMotorTaskHandle = NULL;
static TaskHandle_t g_ctrlTask = NULL;

// command state (protected by mutex)
static float v_mm_s = 0.0f;
static float w_deg_s = 0.0f;
static int16_t dis_mm = 0;
static int16_t dir_deg = 0;

static bool manual_drive_state = false;

// ================== PID Control Task ==================
static void ctrlTask(void *arg)
{
  (void)arg;
  esp_task_wdt_add(NULL);

  while (1) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY); // chờ tick từ ISR
    speed_control();                        // PID + set PWM (an toàn trong task)
    esp_task_wdt_reset();
  }
}

// ================== HC-12 Receive Task ==================
void recieveTask(void *parameter)
{
  (void)parameter;
  esp_task_wdt_add(NULL);

  Serial.print("Receive Task running on core ");
  Serial.println(xPortGetCoreID());

  static uint8_t buffer[10];
  static uint8_t index = 0;
  static uint32_t lastByteTime = 0;
  static uint32_t lastPacketTime = 0;

  const uint32_t BYTE_TIMEOUT   = 100; // ms
  const uint32_t PACKET_TIMEOUT = 200; // ms

  while (true)
  {
    esp_task_wdt_reset();

    if (HC12.available() == 0)
    {
      // timeout -> stop teleop (không phá manual motion)
      if (millis() - lastPacketTime > PACKET_TIMEOUT)
      {
        if (xSemaphoreTake(motorCommandMutex, pdMS_TO_TICKS(10)) == pdTRUE)
        {
          v_mm_s = 0.0f;
          w_deg_s = 0.0f;
          if (!manual_drive_state)
          {
            dis_mm = 0;
            dir_deg = 0;
          }
          xSemaphoreGive(motorCommandMutex);
        }
      }

      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }

    while (HC12.available())
    {
      uint8_t byteIn = (uint8_t)HC12.read();
      uint32_t now = millis();

      if (index > 0 && (now - lastByteTime > BYTE_TIMEOUT)) index = 0;
      lastByteTime = now;

      if (index == 0 && byteIn != 0xAA) continue;

      buffer[index++] = byteIn;

      if (index == 10)
      {
        if (buffer[0] == 0xAA && buffer[9] == 0x55)
        {
          // NOTE: dist/angle đọc dạng int16 để hỗ trợ âm (nếu bên TX gửi signed)
          int16_t distance_mm = (int16_t)(buffer[1] | (buffer[2] << 8));
          int16_t angle_deg   = (int16_t)(buffer[3] | (buffer[4] << 8));

          int16_t w_raw = (int16_t)(buffer[5] | (buffer[6] << 8));
          int16_t v_raw = (int16_t)(buffer[7] | (buffer[8] << 8));

          float v_f = v_raw / 10.0f; // mm/s
          float w_f = w_raw / 10.0f; // deg/s

          ManualCommand_t cmd = {v_f, w_f, distance_mm, angle_deg};

          bool is_manual = (distance_mm != 0 || angle_deg != 0);

          // cập nhật trạng thái chung
          if (xSemaphoreTake(motorCommandMutex, pdMS_TO_TICKS(10)) == pdTRUE)
          {
            v_mm_s = v_f;
            w_deg_s = w_f;
            dis_mm = distance_mm;
            dir_deg = angle_deg;

            // nếu không manual -> clear flags
            if (!is_manual)
            {
              dis_mm = 0;
              dir_deg = 0;
              move_done = false;
              turn_done = false;
            }
            xSemaphoreGive(motorCommandMutex);
          }

          // enqueue manual command (chỉ khi chưa đang manual)
          if (is_manual && !manual_drive_state)
          {
            // queue length=1 => overwrite để không spam lệnh
            xQueueOverwrite(manualCommandQueue, &cmd);
            Serial.println("[HC12] Manual command overwritten to queue");
          }

          lastPacketTime = now;
        }
        else
        {
          Serial.println("[HC12] Invalid packet");
        }

        index = 0;
      }
    }

    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

// ================== Simple Motor Task (teleop v/w) ==================
void simpleMotorTask(void *parameter)
{
  (void)parameter;
  esp_task_wdt_add(NULL);

  const TickType_t xFrequency = pdMS_TO_TICKS(100);
  TickType_t xLastWakeTime = xTaskGetTickCount();

  while (true)
  {
    esp_task_wdt_reset();

    if (xSemaphoreTake(motorCommandMutex, pdMS_TO_TICKS(10)) == pdTRUE)
    {
      float local_v = v_mm_s;
      float local_w = w_deg_s;
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

// ================== Manual Motor Task (dist+angle => ONE call) ==================
void manualMotorTask(void *parameter)
{
  (void)parameter;
  esp_task_wdt_add(NULL);

  ManualCommand_t cmd;

  while (true)
  {
    if (xQueueReceive(manualCommandQueue, &cmd, portMAX_DELAY) == pdPASS)
    {
      Serial.println("[MANUAL TASK] Command received");

      // set manual flag
      if (xSemaphoreTake(motorCommandMutex, pdMS_TO_TICKS(100)) == pdTRUE)
      {
        manual_drive_state = true;
        v_mm_s = cmd.v_mm_s;
        w_deg_s = cmd.w_deg_s;
        dis_mm  = cmd.dist_mm;
        dir_deg = cmd.angle_deg;
        xSemaphoreGive(motorCommandMutex);
      }

      // >>> GỘP: 1 hàm duy nhất (đi thẳng, lùi, xoay tại chỗ, hoặc quẹo cung tròn)
      // dist_mm và angle_deg có thể độc lập 0 hoặc khác 0
      drive_motion((float)cmd.dist_mm, (float)cmd.angle_deg, cmd.v_mm_s, cmd.w_deg_s);

      // stop
      motor_speed_set(0, 0);

      // clear manual
      if (xSemaphoreTake(motorCommandMutex, pdMS_TO_TICKS(100)) == pdTRUE)
      {
        manual_drive_state = false;
        v_mm_s = 0;
        w_deg_s = 0;
        dis_mm = 0;
        dir_deg = 0;
        xSemaphoreGive(motorCommandMutex);
      }

      move_done = false;
      turn_done = false;

      Serial.println("[MANUAL TASK] Done.");
      esp_task_wdt_reset();
    }
  }
}

// ================== Setup ==================
void setup()
{
  Serial.begin(115200);
  delay(300);

  HC12.begin(9600, SERIAL_8N1, HC12_RX_PIN, HC12_TX_PIN);

  esp_task_wdt_init(5, true);

  manualCommandQueue = xQueueCreate(1, sizeof(ManualCommand_t)); // overwrite queue
  motorCommandMutex  = xSemaphoreCreateMutex();

  if (!manualCommandQueue) Serial.println("Failed to create manualCommandQueue");
  if (!motorCommandMutex)  Serial.println("Failed to create motor command mutex");

  // LIDAR init (nếu cần)
  rlidar.begin();
  rlidar.lidar_run();

  // Motor init (chưa start PID timer ở đây)
  motor_begin();
  motor_speed_set(0, 0);
  manual_drive_state = false;

  // Create ctrl task FIRST, then register handle to PID ISR
  xTaskCreatePinnedToCore(ctrlTask, "ctrl", 4096, NULL, 10, &g_ctrlTask, MOTOR_TASK_CORE);
  pid_set_control_task_handle(g_ctrlTask);

  // Now start PWM + PID timer
  motor_run(true);

  // Tasks
  xTaskCreatePinnedToCore(recieveTask, "Receive Task", 4096, NULL, 2, &recieveTaskHandle, RX_TASK_CORE);
  xTaskCreatePinnedToCore(simpleMotorTask, "Motor Task", 4096, NULL, 6, &simpleMotorTaskHandle, MOTOR_TASK_CORE);
  xTaskCreatePinnedToCore(manualMotorTask, "Manual Motor Task", 4096, NULL, 3, &manualMotorTaskHandle, MOTOR_TASK_CORE);

  Serial.println("Setup done.");
}

// ================== Main loop ==================
void loop()
{
  // Debug encoder properly (đọc realtime, không dùng biến global init sớm)
  int32_t c0 = encoder_get_count32_v44(PCNT_UNIT_0);
  int32_t c1 = encoder_get_count32_v44(PCNT_UNIT_1);
  Serial.printf("ENC0=%ld | ENC1=%ld | manual=%d | v=%.1f w=%.1f dis=%d dir=%d\n",
                (long)c0, (long)c1,
                (int)manual_drive_state,
                (double)v_mm_s, (double)w_deg_s,
                (int)dis_mm, (int)dir_deg);

  vTaskDelay(pdMS_TO_TICKS(1000));
}
