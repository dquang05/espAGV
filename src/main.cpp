#include <Arduino.h>
// #include <WiFi.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#include "driver/uart.h"
#include "lidar.h"
#include "wifiBridge.h"
#include "motor_control.h"

// ===== WiFi =====
static const char *WIFI_SSID = "YOUR_SSID";
static const char *WIFI_PASS = "YOUR_PASS";
static const uint16_t PORT_LIDAR = 9000;
static const uint16_t PORT_CTRL = 9001;

// ===== Lidar UART =====
static const int LIDAR_BAUD = 115200;

// lidar motor pwm via your lidar lib + pwmgen
static const int LIDAR_PWM_CASE = 3;
static const uint32_t LIDAR_PWM_FREQ = 20000;
static const float LIDAR_PWM_DUTY = 85.0f;

// ===== Control protocol bytes =====
static const uint8_t CMD_STOP = 0xFF;
static const uint8_t CMD_DRIVE = 0xD1; // + int16 v, int16 w
static const uint8_t CMD_AUTO = 0xA1;  // + int16 dist, int16 angle, int16 v, int16 w
static const uint8_t CMD_MODE = 0xC1;  // + uint8 mode (0 drive, 1 auto)

// ===== Mode =====
enum RobotMode : uint8_t
{
  MODE_DRIVE = 0,
  MODE_AUTO = 1
};
static volatile RobotMode g_mode = MODE_DRIVE;

// Protect motor commands vs PID/other tasks
static SemaphoreHandle_t g_motorMutex = nullptr;
static SemaphoreHandle_t g_modeMutex = nullptr;

// AUTO queue
typedef struct
{
  int16_t dist_mm;
  int16_t angle_deg;
  int16_t v_mm_s;
  int16_t w_deg_s;
} AutoCmd;

static QueueHandle_t g_autoQ = nullptr;

// ===== Objects =====
static LidarA1 lidar;
static WifiBridgeTCP lidarStream; // port 9000

// ===== Control socket server minimal =====
#include "lwip/sockets.h"
#include "lwip/inet.h"
#include <fcntl.h>
#include <errno.h>
#include <netinet/tcp.h>

static int s_listen = -1;
static int s_client = -1;

// Release socket fd if valid
static void closeFd(int &fd)
{
  if (fd >= 0)
  {
    close(fd);
    fd = -1;
  }
}

// Initialize control server socket (TCP:9001)
static bool startCtrlServer(uint16_t port)
{
  s_listen = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
  if (s_listen < 0)
    return false;

  int yes = 1;
  setsockopt(s_listen, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));

  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(port);
  addr.sin_addr.s_addr = htonl(INADDR_ANY);

  if (bind(s_listen, (sockaddr *)&addr, sizeof(addr)) < 0)
  {
    closeFd(s_listen);
    return false;
  }
  if (listen(s_listen, 1) < 0)
  {
    closeFd(s_listen);
    return false;
  }

  // non-block accept
  int flags = fcntl(s_listen, F_GETFL, 0);
  fcntl(s_listen, F_SETFL, flags | O_NONBLOCK);

  Serial.printf("[CTRL] listen %u\n", port);
  return true;
}

// Accept client connection non-blocking and store descriptor in `s_client`
static void pollAccept()
{
  if (s_client >= 0)
    return;
  if (s_listen < 0)
    return;
  // Minimal blocking control socket

  sockaddr_in6 ca{};
  socklen_t len = sizeof(ca);
  int c = accept(s_listen, (sockaddr *)&ca, &len);
  if (c >= 0)
  {
    s_client = c;
    int one = 1;
    setsockopt(s_client, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
    Serial.println("[CTRL] client connected");
  }
}

// Blocking read exact N bytes (with select timeout)
static bool recvExact(uint8_t *out, int n, int timeoutMs)
{
  int got = 0;
  while (got < n)
  {
    fd_set rfds;
    FD_ZERO(&rfds);
    FD_SET(s_client, &rfds);

    timeval tv{};
    tv.tv_sec = timeoutMs / 1000;
    tv.tv_usec = (timeoutMs % 1000) * 1000;

    int r = select(s_client + 1, &rfds, nullptr, nullptr, &tv);
    if (r <= 0)
      return false; // timeout or error

    int k = recv(s_client, (char *)out + got, n - got, 0);
    if (k == 0)
    {
      Serial.println("[CTRL] client disconnected");
      closeFd(s_client);
      return false;
    }
    if (k < 0)
    {
      Serial.println("[CTRL] recv error");
      closeFd(s_client);
      return false;
    }
    got += k;
  }
  return true;
}

// Thread-safe set mode
static inline void setMode(RobotMode m)
{
  xSemaphoreTake(g_modeMutex, portMAX_DELAY);
  g_mode = m;
  xSemaphoreGive(g_modeMutex);

  // safety stop when switching
  xSemaphoreTake(g_motorMutex, portMAX_DELAY);
  motor_speed_set(0, 0);
  xSemaphoreGive(g_motorMutex);

  Serial.printf("[MODE] %s\n", (m == MODE_DRIVE) ? "DRIVE" : "AUTO");
}

// Thread-safe get mode
static inline RobotMode getMode()
{
  xSemaphoreTake(g_modeMutex, portMAX_DELAY);
  RobotMode m = g_mode;
  xSemaphoreGive(g_modeMutex);
  return m;
}

// Lidar streaming task (TCP:9000)
static void taskLidar(void *)
{
  static bool scanning = false;
  static uint8_t buf[2048];
  bool connected = lidarStream.isClientConnected();
  for (;;)
  {
    lidarStream.poll();
    if (connected && !scanning)
    {
      lidar.motor_start();
      lidar.motor_set(85);
      lidar.start_scan();
      scanning = true;
    }

    if (!connected && scanning)
    {
      lidar.stop();
      lidar.motor_stop();
      scanning = false;
    }
    else
    {
      vTaskDelay(pdMS_TO_TICKS(20));
    }
  }
}

// AUTO task: process AUTO commands from queue
static void taskAuto(void *)
{
  AutoCmd cmd;
  for (;;)
  {
    if (xQueueReceive(g_autoQ, &cmd, portMAX_DELAY) == pdTRUE)
    {
      if (getMode() != MODE_AUTO)
        continue;

      Serial.printf("[AUTO] dist=%d angle=%d v=%d w=%d\n",
                    cmd.dist_mm, cmd.angle_deg, cmd.v_mm_s, cmd.w_deg_s);

      xSemaphoreTake(g_motorMutex, portMAX_DELAY);
      drive_motion((float)cmd.dist_mm, (float)cmd.angle_deg, (float)cmd.v_mm_s, (float)cmd.w_deg_s);
      motor_speed_set(0, 0);
      xSemaphoreGive(g_motorMutex);

      Serial.println("[AUTO] done");
    }
  }
}

// Control task: handle control commands from TCP client
static void taskCtrl(void *)
{
  static uint32_t lastDriveMs = 0;
  static bool wdStopped = false;
  for (;;)
  {
    pollAccept();
    if (s_client < 0)
    {
      vTaskDelay(pdMS_TO_TICKS(50));
      continue;
    }

    // read header (1 byte)
    uint8_t hdr = 0;
    if (!recvExact(&hdr, 1, 200))
    {
      vTaskDelay(pdMS_TO_TICKS(1));
      continue;
    }

    if (hdr == CMD_STOP)
    {
      // STOP has highest priority
      xSemaphoreTake(g_motorMutex, portMAX_DELAY);
      motor_speed_set(0, 0);
      xSemaphoreGive(g_motorMutex);
      Serial.println("[CTRL] STOP");
      continue;
    }

    if (hdr == CMD_MODE)
    {
      uint8_t m = 0;
      if (!recvExact(&m, 1, 200))
        continue;
      setMode(m ? MODE_AUTO : MODE_DRIVE);
      continue;
    }

    if (hdr == CMD_DRIVE)
    {
      lastDriveMs = millis();
      wdStopped = false;
      // payload 4 bytes: int16 v, int16 w
      uint8_t p[4];
      if (!recvExact(p, 4, 200))
        continue;

      int16_t v = (int16_t)(p[0] | (p[1] << 8));
      int16_t w = (int16_t)(p[2] | (p[3] << 8));

      if (getMode() == MODE_DRIVE)
      {
        xSemaphoreTake(g_motorMutex, portMAX_DELAY);
        motor_speed_set((float)v, (float)w);
        xSemaphoreGive(g_motorMutex);
      }
      if (getMode() == MODE_DRIVE && !wdStopped)
      {
        if (millis() - lastDriveMs > 400)
        {
          xSemaphoreTake(g_motorMutex, portMAX_DELAY);
          motor_speed_set(0, 0);
          xSemaphoreGive(g_motorMutex);
          wdStopped = true;
          Serial.println("[CTRL] watchdog STOP (no DRIVE cmd)");
        }
      }
      continue;
    }

    if (hdr == CMD_AUTO)
    {
      // payload 8 bytes: int16 dist, angle, v, w
      uint8_t p[8];
      if (!recvExact(p, 8, 200))
        continue;

      AutoCmd cmd{};
      cmd.dist_mm = (int16_t)(p[0] | (p[1] << 8));
      cmd.angle_deg = (int16_t)(p[2] | (p[3] << 8));
      cmd.v_mm_s = (int16_t)(p[4] | (p[5] << 8));
      cmd.w_deg_s = (int16_t)(p[6] | (p[7] << 8));

      if (getMode() == MODE_AUTO)
      {
        xQueueOverwrite(g_autoQ, &cmd); // chỉ giữ lệnh mới nhất
      }
      continue;
    }

    // Unknown header -> ignore (or flush)
    Serial.printf("[CTRL] unknown hdr 0x%02X\n", hdr);
  }
}

void setup()
{
  Serial.begin(115200);
  delay(800);
  Serial.println("\n=== AGV binary control + lidar stream ===");

  g_motorMutex = xSemaphoreCreateMutex();
  g_modeMutex = xSemaphoreCreateMutex();
  g_autoQ = xQueueCreate(1, sizeof(AutoCmd));

  // motors
  motor_begin();
  motor_run(true);

  // lidar
  esp_err_t e = lidar.begin(LIDAR_UART, UART_TX_PIN, UART_RX_PIN, LIDAR_BAUD,
                            LIDAR_PWM_CASE, LIDAR_PWM_FREQ, LIDAR_PWM_DUTY);
  Serial.printf("[LIDAR] begin=%d\n", (int)e);
  delay(600);
  lidar.stop();
  delay(80);
  lidar.start_scan();

  // wifi + stream server
  if (!lidarStream.begin(WIFI_SSID, WIFI_PASS, PORT_LIDAR))
  {
    Serial.println("[WIFI] stream begin failed");
  }
  else
  {
    Serial.printf("[WIFI] stream tcp://%s:%u\n", lidarStream.ipString(), PORT_LIDAR);
  }

  // ctrl server
  if (!startCtrlServer(PORT_CTRL))
  {
    Serial.println("[CTRL] server begin failed");
  }
  else
  {
    Serial.printf("[WIFI] ctrl tcp://%s:%u\n", lidarStream.ipString(), PORT_CTRL);
  }

  setMode(MODE_DRIVE);

  // tasks
  xTaskCreatePinnedToCore(taskLidar, "lidar", 4096, nullptr, 10, nullptr, 0);
  xTaskCreatePinnedToCore(taskCtrl, "ctrl", 4096, nullptr, 9, nullptr, 1);
  xTaskCreatePinnedToCore(taskAuto, "auto", 6144, nullptr, 8, nullptr, 1);
}

// Arduino `loop`: vòng lặp chính rất nhẹ, chỉ sleep để nhường CPU cho các task
void loop()
{
  // nothing heavy here
  vTaskDelay(pdMS_TO_TICKS(20));
}
