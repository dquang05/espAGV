#include <Arduino.h>
#include "LidarHandler.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/stream_buffer.h"

#define STREAM_BUF_SIZE 4096 // Buffer size in bytes

StreamBufferHandle_t lidarStream = NULL;

// Stats variables
int pointCount = 0;
int recvCount = 0;
int dropCount = 0;
unsigned long lastPrint = 0;
float global_angle = 0;
float global_distance = 0;
float global_quality = 0;
LidarHandler lidar;
LidarPoint point;
// Task to read LIDAR and push data into buffer
void LidarTask(void *pvParameters)
{
    for (;;)
    {
        lidar.run();
        // Serial.println("Looplidar");
        if (lidar.coordinate_take(point))
        {
            // Serial.println("Take point");
            if (point.distance > 0 && point.distance < 7000 &&
                point.angleDeg >= 0 && point.angleDeg < 360)
            {
                // Serial.println("Got point");
                pointCount++;

                uint16_t ang = (uint16_t)round(point.angleDeg * 10); // angle * 0.1 deg
                uint16_t dist = (uint16_t)round(point.distance);

                uint8_t pkt[4];
                pkt[0] = ang & 0xFF;
                pkt[1] = ang >> 8;
                pkt[2] = dist & 0xFF;
                pkt[3] = dist >> 8;

                // Try to push into buffer
                size_t sent = xStreamBufferSend(lidarStream, pkt, sizeof(pkt), 0);
                if (sent < sizeof(pkt))
                {
                    dropCount++;
                }
            }

            vTaskDelay(pdMS_TO_TICKS(1)); // small delay
        }
    }
}
// Task to read from buffer and process data
void ProcessTask(void *pvParameters)
{
    uint8_t pkt[4];
    for (;;)
    {
        // Try to receive a packet (blocking up to 20ms)
        size_t recvd = xStreamBufferReceive(lidarStream, pkt, sizeof(pkt), pdMS_TO_TICKS(20));
        if (recvd == sizeof(pkt))
        {
            recvCount++;
            // Example: reconstruct angle & distance
            uint16_t ang, dist;
            memcpy(&ang, pkt, 2);
            memcpy(&dist, pkt + 2, 2);

            float angle = ang / 10.0f;
            float distance = (float)dist;

            // You can process or send over BLE/WiFi here instead of printing
        }

        // Print statistics every second
        if (millis() - lastPrint >= 1000)
        {
            Serial.print("LIDAR points read: ");
            Serial.print(pointCount);
            Serial.print(" | Pushed to buffer: ");
            Serial.print(recvCount);
            Serial.print(" | Dropped: ");
            Serial.println(dropCount);

            pointCount = 0;
            recvCount = 0;
            dropCount = 0;
            lastPrint = millis();
        }
    }
}

void setup()
{
    Serial.begin(115200);

    Serial.println("Starting LIDAR test with FreeRTOS...");

    // Init LIDAR
    lidar.begin(9, 10, 3); // RX, TX, PWM pins)

    delay(100);
    // Create stream buffer
    lidarStream = xStreamBufferCreate(STREAM_BUF_SIZE, 4);
    if (lidarStream == NULL)
    {
        Serial.println("Failed to create stream buffer!");
        while (1)
            ;
    }

    // Create tasks
    xTaskCreatePinnedToCore(LidarTask, "LidarTask", 4096, NULL, 2, NULL, 1);
    xTaskCreatePinnedToCore(ProcessTask, "ProcessTask", 4096, NULL, 1, NULL, 0);
}

void loop()
{
   Serial.print("LIDAR Point - Angle: ");
   Serial.print(global_angle);
   Serial.print(" | Distance: ");
   Serial.print(global_distance);
   Serial.print(" | Quality: ");
   Serial.println(global_quality);
}
