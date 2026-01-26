#include <Arduino.h>
#include "motor_control.h"
#include "pid.h"
#include "esp_task_wdt.h"

// ================== CONFIG ==================
static const float MAX_PPS = 800.0f;      // max setpoint pulse/s (chỉnh dần)
static const float ACC_PPS2 = 600.0f;     // gia tốc ramp (pulse/s^2)
static const uint32_t RAMP_DT_MS = 50;    // chu kỳ cập nhật setpoint
static const uint32_t LOG_DT_MS  = 200;   // chu kỳ log

static TaskHandle_t g_ctrl_task = NULL;
static volatile float g_sp_pps = 0.0f;    // setpoint hiện tại (pulse/s)

// Nếu bạn muốn đổi chiều encoder/motor, chỉ cần đổi 2 chân dir dưới đây
static inline void set_forward_dir_both()
{
  // motor 1 forward
  digitalWrite(dir11, HIGH);
  digitalWrite(dir12, LOW);
  // motor 2 forward
  digitalWrite(dir21, HIGH);
  digitalWrite(dir22, LOW);
}

static inline void stop_dir_both()
{
  digitalWrite(dir11, LOW);
  digitalWrite(dir12, LOW);
  digitalWrite(dir21, LOW);
  digitalWrite(dir22, LOW);
}

// ================== CTRL TASK ==================
static void ctrlTask(void *param)
{
  (void)param;
  esp_task_wdt_add(NULL);

  while (true) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);  
    speed_control();                          
    esp_task_wdt_reset();
  }
}

// ================== RAMP TASK ==================
static void rampTask(void *param)
{
  (void)param;
  esp_task_wdt_add(NULL);

  const float dt_s = (float)RAMP_DT_MS / 1000.0f;

  while (true) {
    // ramp up
    while (g_sp_pps < MAX_PPS) {
      g_sp_pps += ACC_PPS2 * dt_s;
      if (g_sp_pps > MAX_PPS) g_sp_pps = MAX_PPS;

      // setpoint pps 
      set_speed1 = g_sp_pps;
      set_speed2 = g_sp_pps;

      vTaskDelay(pdMS_TO_TICKS(RAMP_DT_MS));
      esp_task_wdt_reset();
    }

    vTaskDelay(pdMS_TO_TICKS(800)); // hold

    // ramp down
    while (g_sp_pps > 0.0f) {
      g_sp_pps -= ACC_PPS2 * dt_s;
      if (g_sp_pps < 0.0f) g_sp_pps = 0.0f;

      set_speed1 = g_sp_pps;
      set_speed2 = g_sp_pps;

      vTaskDelay(pdMS_TO_TICKS(RAMP_DT_MS));
      esp_task_wdt_reset();
    }

    // stop
    set_speed1 = 0.0f;
    set_speed2 = 0.0f;
    vTaskDelay(pdMS_TO_TICKS(800));
  }
}

// ================== LOG TASK ==================
static void logTask(void *param)
{
  (void)param;
  esp_task_wdt_add(NULL);

  while (true) {
    Serial.printf(
      "t=%lu ms | sp=%.1f pps | "
      "set[L=%.1f R=%.1f] | cur[L=%.1f R=%.1f] | "
      "duty[L=%.1f R=%.1f] | dPulse[L=%ld R=%ld]\n",
      (unsigned long)millis(),
      (double)g_sp_pps,
      (double)set_speed1, (double)set_speed2,
      (double)cur_speed1, (double)cur_speed2,
      (double)pid_ctr1.output, (double)pid_ctr2.output,
      (long)delta_pulse1, (long)delta_pulse2
    );

    vTaskDelay(pdMS_TO_TICKS(LOG_DT_MS));
    esp_task_wdt_reset();
  }
}

void setup()
{
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== PID SETPOINT (PULSE/S) RAMP TEST ===");
  Serial.println("No wheels: use pulse/s setpoint. Lift motors for safety.");

  // init motor + encoder + pid timer 
  motor_begin();

  set_forward_dir_both();

  // create ctrl task -> set handle -> then run
  xTaskCreatePinnedToCore(ctrlTask, "ctrl", 4096, NULL, 10, &g_ctrl_task, 1);
  pid_set_control_task_handle(g_ctrl_task);

  // start everything
  set_speed1 = 0.0f;
  set_speed2 = 0.0f;
  motor_run(true);

  // tasks
  xTaskCreatePinnedToCore(rampTask, "ramp", 4096, NULL, 8, NULL, 1);
  xTaskCreatePinnedToCore(logTask,  "log",  4096, NULL, 5, NULL, 1);

  Serial.println("Ramp started.");
}

void loop()
{
  vTaskDelay(pdMS_TO_TICKS(1000));
}
