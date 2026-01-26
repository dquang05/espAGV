#include "LidarHandler.h"

static LidarHandler *instancePtr = nullptr;

LidarHandler::LidarHandler() : lidar(Serial2) {
    instancePtr = this;
}

bool LidarHandler::begin(uint8_t rxPin, uint8_t txPin, uint8_t pwmPin) {
    // PWM điều khiển motor LIDAR
    motorPWM.begin(pwmPin, 5000, 95);
    delay(10);
    motorPWM.set_frequency(10000);
    delay(10);
    motorPWM.set_frequency(50000);

    Serial.begin(115200);
    lidar.init(rxPin, txPin);
    lidar.postParseCallback = dataHandlerTrampoline;

    if(!lidar.connectionCheck()) {
        Serial.println("LIDAR: connection failed");
        return false;
    }

    lidar.startExpressScan(EXPRESS_SCAN_WORKING_MODE_BOOST);
    return true;
}

// static callback → chuyển về method thật
void LidarHandler::dataHandlerTrampoline(RPlidar* lidarPtr, uint16_t dist, uint16_t angle_q6, uint8_t newRotFlag, int8_t quality) {
    if (instancePtr != nullptr) {
        instancePtr->dataHandler(dist, angle_q6, newRotFlag, quality);
    }
}

// Lưu dữ liệu
void LidarHandler::dataHandler(uint16_t dist, uint16_t angle_q6, uint8_t newRotFlag, int8_t quality) {
    lastDistance = dist;
    lastAngle = (angle_q6 * 0.015625f); // q6 -> degrees
}

void LidarHandler::run() {
    lidar.handleData(false, false);
}

bool LidarHandler::coordinate_take(LidarPoint &outPoint) {
    if (lastDistance < 0) return false;

    outPoint.distance = lastDistance;
    outPoint.angleDeg = lastAngle;
    return true;
}
