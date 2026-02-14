#include <Arduino.h>
#include "debug.h"
// #include "esp_sleep.h"
#include "esp_timer.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#include "driver/uart.h"
#include "lidar.h"
#include "wifiBridge.h"
#include "telemetry_packet.h"
#include "mpu6050.h"
#include "twi_esp32.h"
#include "udpBeacon.h"
#include "motor_control.h"
 #if DEBUG_ENABLE
int imuCount = 0;
int lidarCount = 0;
int ctrCount = 0;
 #endif

// ===== WiFi =====
// static const char *WIFI_SSID = "TP-Link_C718";
// static const char *WIFI_PASS = "20017231";
// static const char *WIFI_SSID = "VIETTEL";
// static const char *WIFI_PASS = "0906608600";
static const char *WIFI_SSID = "Danh Van";
static const char *WIFI_PASS = "danhvan123";
static const uint16_t PORT_LIDAR = 9000;
static const uint16_t PORT_CTRL = 9001;
static const uint16_t PORT_BEACON = 50000;
static const uint16_t PORT_TELEM = 9002;

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

// ===== IMU I2C PINS =====
static const gpio_num_t SDA_PIN = GPIO_NUM_21;
static const gpio_num_t SCL_PIN = GPIO_NUM_22;

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

// ===== I2C bus for IMU =====
static twi_bus_t bus = {
    .port = I2C_NUM_0,
    .sda = SDA_PIN,
    .scl = SCL_PIN,
    .clk_hz = 400000, // try 1000000 for long wires or noisy env
    .pullup = true};

// ====== IMU OBJECT ======
static MPU6050 imu(I2C_NUM_0, 0x68);

// ===== Subsystem states =====
enum SubsysState : uint8_t
{
  SUBSYS_OFF = 0,
  SUBSYS_OK = 1,
  SUBSYS_FAULT = 2
};

static volatile SubsysState g_lidar_state = SUBSYS_OFF;
static volatile SubsysState g_imu_state = SUBSYS_OFF;
static volatile SubsysState g_enc_state = SUBSYS_OFF;
static volatile SubsysState g_wifi_state = SUBSYS_OFF;

static QueueHandle_t g_autoQ = nullptr;

// ===== Objects =====
static LidarA1 lidar;
static WifiBridgeTCP lidarStream; // port 9000
static WifiBridgeTCP telemStream; // port 9002
static UdpBeacon udpBeacon;       // port 50000
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

// Command failsafe: stop motors and log reason
static void failsafe_stop(const char *reason)
{
  DBG_PRINTF("[FAILSAFE] %s\n", reason);
  xSemaphoreTake(g_motorMutex, portMAX_DELAY);
  motor_speed_set(0, 0);
  xSemaphoreGive(g_motorMutex);
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

static inline bool clientActive() { return lidarStream.isClientConnected() || telemStream.isClientConnected() || (s_client >= 0); }

static void wifiPowerManagerTick()
{
  uint32_t now = esp_timer_get_time() / 1000;
  if (now - g_wifiPmLastMs < WIFI_PM_PERIOD_MS)
    return;
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
// Polling task


// Lidar streaming task (TCP:9000)
static void taskLidar(void *)
{
  static bool scanning = false;
  static uint8_t buf[2048];
  uint32_t last_rx_ms = esp_timer_get_time() / 1000;
  uint32_t backoff_until_ms = 0;
  for (;;)
  {
    lidarStream.poll();
    wifiPowerManagerTick();
    DEBUG_LIDAR_INC();
    bool connected = lidarStream.isClientConnected();
    uint32_t now = esp_timer_get_time() / 1000;

    // Không có client -> chỉ idle, KHÔNG coi là lỗi
    if (!connected)
    {
      if (scanning)
      {
        lidar.stop();
        lidar.motor_stop();
        scanning = false;
        DBG_PRINTLN("[LIDAR] streaming OFF");
      }
      // idle = OK (hoặc bạn có thể giữ nguyên state hiện tại)
      if (g_lidar_state != SUBSYS_FAULT)
        g_lidar_state = SUBSYS_OK;
      vTaskDelay(pdMS_TO_TICKS(20));
      continue;
    }

    // Nếu đang FAULT thì chờ backoff rồi mới thử lại
    if (g_lidar_state == SUBSYS_FAULT && now < backoff_until_ms)
    {
      vTaskDelay(pdMS_TO_TICKS(50));
      continue;
    }

    // Có client mà chưa scan -> start scan
    if (!scanning)
    {
      lidar.motor_start();
      lidar.motor_set(85);
      lidar.start_scan();
      scanning = true;
      last_rx_ms = now;
      DBG_PRINTLN("[LIDAR] streaming ON");
    }

    int n = uart_read_bytes(LIDAR_UART, buf, sizeof(buf), pdMS_TO_TICKS(10));
    if (n > 0)
    {
      last_rx_ms = now;
      g_lidar_state = SUBSYS_OK;
      lidarStream.sendBytes(buf, (size_t)n);
    }
    else
    {
      // Connected but no data => fault
      if (now - last_rx_ms > 500)
      {
        g_lidar_state = SUBSYS_FAULT;
        lidar.stop();
        lidar.motor_stop();
        scanning = false;
        backoff_until_ms = now + 1000; // 1s rồi thử lại
        DBG_PRINTLN("[LIDAR] no data -> FAULT, will retry");
      }
    }
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

// IMU task: read IMU and send telemetry packets
static void taskTelemetryIMU(void *)
{
  const TickType_t period = pdMS_TO_TICKS(10); // 100 Hz
  TickType_t lastWake = xTaskGetTickCount();

  uint64_t last_us = (uint64_t)esp_timer_get_time();
  uint32_t seq = 0;

  int err_streak = 0;
  int recover_div = 0; // dùng để thử recover mỗi ~500ms
  static bool last_conn = false;
  for (;;)
  {
    vTaskDelayUntil(&lastWake, period);
    telemStream.poll();
    DEBUG_IMU_INC();
    wifiPowerManagerTick();
    bool conn = telemStream.isClientConnected();
    if (conn && !last_conn)
      DBG_PRINTLN("[TELEM] client connected");
    if (!conn && last_conn)
      DBG_PRINTLN("[TELEM] client disconnected");
    last_conn = conn;

    if (!conn)
      continue;

    // ===== fault mode: thử hồi phục định kỳ (không delay phá nhịp) =====
    if (g_imu_state == SUBSYS_FAULT)
    {
      if (++recover_div >= 50) // 50 * 10ms = ~500ms
      {
        recover_div = 0;
        uint8_t who = 0;
        if (imu.whoAmI(&who) == ESP_OK && (who == 0x68 || who == 0x69))
        {
          g_imu_state = SUBSYS_OK;
          err_streak = 0;
        }
      }
      continue;
    }
    recover_div = 0;

    // ===== dt bằng esp_timer_get_time() là OK cho filter =====
    uint64_t now_us = (uint64_t)esp_timer_get_time();
    float dt = (now_us - last_us) * 1e-6f;
    last_us = now_us;
    if (dt <= 0)
      dt = 0.01f;
    if (dt > 0.1f)
      dt = 0.1f;

    mpu6050_scaled_t s{}; // NOTE: đây là BODY AXES sau mapping
    float roll = 0.0f, pitch = 0.0f;

    if (imu.readScaledAndAngles(&s, &roll, &pitch, dt) != ESP_OK)
    {
      if (++err_streak >= 10)
        g_imu_state = SUBSYS_FAULT;
      continue;
    }
    err_streak = 0;

    TeleImuPktV1 pkt{};
    pkt.magic = 0xA55A;
    pkt.ver = 1;
    pkt.type = 1;
    pkt.seq = seq++;
    pkt.t_ms = (uint32_t)(now_us / 1000ULL);

    pkt.roll_cdeg = (int16_t)(roll * 100.0f);
    pkt.pitch_cdeg = (int16_t)(pitch * 100.0f);

    pkt.ax_mg = (int16_t)(s.ax_g * 1000.0f);
    pkt.ay_mg = (int16_t)(s.ay_g * 1000.0f);
    pkt.az_mg = (int16_t)(s.az_g * 1000.0f);

    pkt.gx_dps10 = (int16_t)(s.gx_dps * 10.0f);
    pkt.gy_dps10 = (int16_t)(s.gy_dps * 10.0f);
    pkt.gz_dps10 = (int16_t)(s.gz_dps * 10.0f);

    pkt.temp_c10 = (int16_t)(s.temp_c * 10.0f);

    telemStream.sendBytes((const uint8_t *)&pkt, sizeof(pkt));
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
  static uint64_t lastDriveMs = 0;
  static bool wdStopped = false;
  for (;;)
  {
    pollAccept();
    wifiPowerManagerTick();
    DEBUG_CTR_INC();
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
      lastDriveMs = esp_timer_get_time() / 1000;
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
      if (esp_timer_get_time() / 1000 - lastDriveMs > 400)
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

  // ===== init flags =====
  bool wifi_ok = false;
  bool telem_ok = false;
  bool imu_ok = false;
  bool lidar_drv_ok = false;

  // ===== motors (failsafe) =====
  motor_begin();
  motor_run(true);
  motor_speed_set(0, 0); // safe at boot

  // ===== lidar driver init =====
  {
    esp_err_t e = lidar.begin(LIDAR_UART, UART_TX_PIN, UART_RX_PIN, LIDAR_BAUD,
                              LIDAR_PWM_CASE, LIDAR_PWM_FREQ, LIDAR_PWM_DUTY);
    DBG_PRINTF("[LIDAR] begin=%d\n", (int)e);

    lidar_drv_ok = (e == ESP_OK);
    g_lidar_state = lidar_drv_ok ? SUBSYS_OK : SUBSYS_FAULT;

    // ensure lidar is stopped at boot
    lidar.stop();
    lidar.motor_stop();
  }

  // ===== wifi + stream server (9000) =====
  wifi_ok = lidarStream.begin(WIFI_SSID, WIFI_PASS, PORT_LIDAR);
  g_wifi_state = wifi_ok ? SUBSYS_OK : SUBSYS_FAULT;

  if (!wifi_ok)
  {
    DBG_PRINTLN("[WIFI] stream begin failed");
  }
  else
  {
    DBG_PRINTF("[WIFI] stream tcp://%s:%u\n", lidarStream.ipString(), PORT_LIDAR);
  }

  // ===== telemetry server (9002) =====
  if (wifi_ok)
  {
    telem_ok = telemStream.beginServerOnly(PORT_TELEM);
    if (!telem_ok)
      DBG_PRINTLN("[TELEM] begin failed");
    else
      DBG_PRINTF("[WIFI] telem tcp://%s:%u\n", telemStream.ipString(), PORT_TELEM);
  }

  // ===== WiFi power manager init =====
  WiFi.setSleep(true);
  g_wifiSleepOn = true;
  DBG_PRINTLN("[WIFI_PM] init sleep ON");

  // ===== ctrl server (9001) =====
  if (!startCtrlServer(PORT_CTRL))
  {
    DBG_PRINTLN("[CTRL] server begin failed");
  }
  else if (wifi_ok)
  {
    DBG_PRINTF("[WIFI] ctrl tcp://%s:%u\n", lidarStream.ipString(), PORT_CTRL);
  }

  // ===== UDP beacon =====
  if (udpBeacon.begin(PORT_BEACON))
  {
    payload.len = snprintf((char *)payload.data, sizeof(payload.data),
                           "AGV_01|%s|%u|%u|%u",
                           wifi_ok ? lidarStream.ipString() : "0.0.0.0",
                           PORT_LIDAR, PORT_CTRL, PORT_TELEM);

    udpBeacon.setPayload(payload.data, payload.len);
    DBG_PRINTF("[BEACON] Started on port %u\n", PORT_BEACON);

    xTaskCreatePinnedToCore(taskBeacon, "beacon", 2048, nullptr, 5, nullptr, 0);
  }
  else
  {
    DBG_PRINTLN("[BEACON] Failed to start");
  }

  // ===== default mode =====
  setMode(MODE_DRIVE);

  // ===== IMU / I2C init =====
  {
    esp_err_t ei = twi_init(&bus);
    DBG_PRINTF("[IMU] twi_init=%d\n", (int)ei);

    if (ei == ESP_OK)
    {
      ei = imu.begin();
      DBG_PRINTF("[IMU] begin=%d\n", (int)ei);
      vTaskDelay(pdMS_TO_TICKS(10));
      if (ei == ESP_OK)
      {
        DBG_PRINTLN("[IMU] Hold still... calibrating gyro");
        ei = imu.calibrateGyro(400, 5); // ~2s
        DBG_PRINTF("[IMU] calibrateGyro=%d\n", (int)ei);

        if (ei == ESP_OK)
        {
          imu.resetAngles(0, 0);
          imu.setAlpha(0.96f);
          imu_ok = true;
        }
        else
        {
          imu_ok = false;
          DBG_PRINTLN("[IMU] gyro calibration failed");
        }
      }
      vTaskDelay(pdMS_TO_TICKS(10));  
    }

    g_imu_state = imu_ok ? SUBSYS_OK : SUBSYS_FAULT;
  }

  // ===== tasks =====
  BaseType_t ok;

ok = xTaskCreatePinnedToCore(taskCtrl, "ctrl", 4096, nullptr, 9, nullptr, 1);
DBG_PRINTF("[TASK] ctrl create=%d\n", (int)ok);

ok = xTaskCreatePinnedToCore(taskAuto, "auto", 4096, nullptr, 8, nullptr, 1);
DBG_PRINTF("[TASK] auto create=%d\n", (int)ok);





  if (wifi_ok && lidar_drv_ok /* && lidar_ok (sau này probe) */)
  {
    xTaskCreatePinnedToCore(taskLidar, "lidar", 4096, nullptr, 10, nullptr, 0);
    DBG_PRINTLN("[LIDAR] taskLidar created");

    // TODO (sau): lidar_ok = lidar.probe(200);
    // nếu fail thì set g_lidar_state=SUBSYS_FAULT và taskLidar backoff/stop motor
  }
  else
  {
    DBG_PRINTLN("[LIDAR] skip taskLidar (wifi or driver not OK)");
  }

  // Telemetry IMU task
  if (wifi_ok && telem_ok && imu_ok)
  {
    xTaskCreatePinnedToCore(taskTelemetryIMU, "telem_imu", 4096, nullptr, 7, nullptr, 1);
    DBG_PRINTLN("[TELEM] taskTelemetryIMU created");
  }
  else
  {
    DBG_PRINTLN("[IMU] skip taskTelemetryIMU (telem or imu not OK)");
  }
  DBG_PRINTLN("Setup done");
}

void loop()
{
  // Serial.printf("IMU count: %d, " "LiDAR count: %d, " 
  //                "CTRL count: %d",
  //                imuCount, lidarCount, ctrCount);
  vTaskDelay(pdMS_TO_TICKS(100));
}
