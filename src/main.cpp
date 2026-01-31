#include <Arduino.h>
#include "debug.h"
#include "esp_sleep.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#include "driver/uart.h"
#include "lidar.h"
#include "wifiBridge.h"
#include "udpBeacon.h"
#include "motor_control.h"

// ===== WiFi =====
// static const char *WIFI_SSID = "TP-Link_C718";
// static const char *WIFI_PASS = "20017231";
static const char *WIFI_SSID = "VIETTEL";
static const char *WIFI_PASS = "0906608600";
static const uint16_t PORT_LIDAR = 9000;
static const uint16_t PORT_CTRL = 9001;
static const uint16_t PORT_BEACON = 50000;

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

// ===== WIFI POWER MANAGER =====
static bool g_wifiSleepOn = false;         
static uint32_t g_wifiPmLastMs = 0;
static const uint32_t WIFI_PM_PERIOD_MS = 300; 

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

// Buffer payload for udp beacon
typedef struct
{
  uint8_t data[128];
  size_t len;
} UdpPayload;

static QueueHandle_t g_autoQ = nullptr;

// ===== Objects =====
static LidarA1 lidar;
static WifiBridgeTCP lidarStream; // port 9000
static UdpBeacon udpBeacon; // port 50000
static UdpPayload payload;

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

  DBG_PRINTF("[CTRL] listen %u\n", port);
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
    DBG_PRINTLN("[CTRL] client connected");
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
      DBG_PRINTLN("[CTRL] client disconnected");
      closeFd(s_client);
      return false;
    }
    if (k < 0)
    {
      DBG_PRINTLN("[CTRL] recv error");
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

  DBG_PRINTF("[MODE] %s\n", (m == MODE_DRIVE) ? "DRIVE" : "AUTO");
}

// Thread-safe get mode
static inline RobotMode getMode()
{
  xSemaphoreTake(g_modeMutex, portMAX_DELAY);
  RobotMode m = g_mode;
  xSemaphoreGive(g_modeMutex);
  return m;
}

static inline bool clientActive(){return lidarStream.isClientConnected() || (s_client >= 0);}

static void wifiPowerManagerTick()
{
  uint32_t now = millis();
  if (now - g_wifiPmLastMs < WIFI_PM_PERIOD_MS) return;
  g_wifiPmLastMs = now;

  bool active = clientActive();

  // Active => sleep OFF (realtime)
  if (active && g_wifiSleepOn)
  {
    WiFi.setSleep(false);
    g_wifiSleepOn = false;
    DBG_PRINTLN("[WIFI_PM] sleep OFF (client active)");
  }
  // Idle => sleep ON (save power)  (MỨC 1)
  else if (!active && !g_wifiSleepOn)
  {
    WiFi.setSleep(true);
    g_wifiSleepOn = true;
    DBG_PRINTLN("[WIFI_PM] sleep ON (idle)");
  }
}

// Lidar streaming task (TCP:9000)
static void taskLidar(void *)
{
  static bool scanning = false;
  static uint8_t buf[2048];

  for (;;)
  {
    wifiPowerManagerTick();  
    lidarStream.poll();
    bool connected = lidarStream.isClientConnected();

    if (connected && !scanning)
    {
      lidar.motor_start();
      lidar.motor_set(85);
      lidar.start_scan();
      scanning = true;
      DBG_PRINTLN("[LIDAR] streaming ON");
    }
    else if (!connected && scanning)
    {
      lidar.stop();
      lidar.motor_stop();
      scanning = false;
      DBG_PRINTLN("[LIDAR] streaming OFF");
    }

    if (connected)
    {
      int n = uart_read_bytes(LIDAR_UART, buf, sizeof(buf), pdMS_TO_TICKS(10));
      if (n > 0)
        lidarStream.sendBytes(buf, (size_t)n);
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
    wifiPowerManagerTick();  
    if (xQueueReceive(g_autoQ, &cmd, portMAX_DELAY) == pdTRUE)
    {
      if (getMode() != MODE_AUTO)
        continue;

      DBG_PRINTF("[AUTO] dist=%d angle=%d v=%d w=%d\n",
                    cmd.dist_mm, cmd.angle_deg, cmd.v_mm_s, cmd.w_deg_s);

      xSemaphoreTake(g_motorMutex, portMAX_DELAY);
      drive_motion((float)cmd.dist_mm, (float)cmd.angle_deg, (float)cmd.v_mm_s, (float)cmd.w_deg_s);
      motor_speed_set(0, 0);
      xSemaphoreGive(g_motorMutex);

      DBG_PRINTLN("[AUTO] done");
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
    wifiPowerManagerTick();  
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
      DBG_PRINTLN("[CTRL] STOP");
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
    if (getMode() == MODE_DRIVE && !wdStopped)
      {
        if (millis() - lastDriveMs > 400)
        {
          xSemaphoreTake(g_motorMutex, portMAX_DELAY);
          motor_speed_set(0, 0);
          xSemaphoreGive(g_motorMutex);
          wdStopped = true;
          DBG_PRINTLN("[CTRL] watchdog STOP (no DRIVE cmd)");
        }
      }

    // Unknown header -> ignore (or flush)
    DBG_PRINTF("[CTRL] unknown hdr 0x%02X\n", hdr);
  }
}

// UDP Beacon task: send periodic UDP beacon
static void taskBeacon(void *)
{
    DBG_PRINTLN("[BEACON] Task started");
    for (;;)
    {
        udpBeacon.tick();
        // Delay 100ms để không chiếm dụng CPU, 
        // tick() sẽ tự biết khi nào đủ 2s để gửi gói tin.
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void setup()
{
  Serial.begin(115200);
  delay(800);
  DBG_PRINTLN("\n=== AGV binary control + lidar stream ===");

  g_motorMutex = xSemaphoreCreateMutex();
  g_modeMutex = xSemaphoreCreateMutex();
  g_autoQ = xQueueCreate(1, sizeof(AutoCmd));

  // motors
  motor_begin();
  motor_run(true);

  // lidar
  esp_err_t e = lidar.begin(LIDAR_UART, UART_TX_PIN, UART_RX_PIN, LIDAR_BAUD,
                            LIDAR_PWM_CASE, LIDAR_PWM_FREQ, LIDAR_PWM_DUTY);
  DBG_PRINTF("[LIDAR] begin=%d\n", (int)e);
  delay(600);
  lidar.stop();
  delay(80);

  // wifi + stream server
  if (!lidarStream.begin(WIFI_SSID, WIFI_PASS, PORT_LIDAR))
  {
    DBG_PRINTLN("[WIFI] stream begin failed");
  }
  else
  {
    DBG_PRINTF("[WIFI] stream tcp://%s:%u\n", lidarStream.ipString(), PORT_LIDAR);
  }
  WiFi.setSleep(true);      
g_wifiSleepOn = true;
DBG_PRINTLN("[WIFI_PM] init sleep ON");

  // ctrl server
  if (!startCtrlServer(PORT_CTRL))
  {
    DBG_PRINTLN("[CTRL] server begin failed");
  }
  else
  {
    DBG_PRINTF("[WIFI] ctrl tcp://%s:%u\n", lidarStream.ipString(), PORT_CTRL);
  }
  if (udpBeacon.begin(PORT_BEACON))
{
    payload.len = snprintf(
        (char*)payload.data,
        sizeof(payload.data),
        "AGV_01|%s|%u|%u",
        lidarStream.ipString(),
        PORT_LIDAR,
        PORT_CTRL
    );

    udpBeacon.setPayload(payload.data, payload.len);

    DBG_PRINTF("[BEACON] Started on port %u\n", PORT_BEACON);

    xTaskCreatePinnedToCore(
        taskBeacon,
        "beacon",
        2048,
        nullptr,
        5,
        nullptr,
        0
    );
}
else
{
    DBG_PRINTLN("[BEACON] Failed to start");
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
  wifiPowerManagerTick();  
  // idleLightSleepTick();
  vTaskDelay(pdMS_TO_TICKS(20));
}
