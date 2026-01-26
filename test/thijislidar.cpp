#define lidarDebugSerial Serial

#include "thijs_rplidar.h"
#include "pwmgen.h"


RPlidar lidar(Serial2);
pwm_gen motorHandler; // PWM on pin 4, 5kHz frequency, 95% max duty cycle

bool keepSpinning = true;
//uint16_t debugPrintCounter = 0;
//const uint16_t debugPrintThreshold = 48; // print data every (this many) datapoints (if you are getting CRC errors, there may be buffer overflow, try setting this to like 48+ (or uncommenting printing entirely))
uint32_t debugPrintTimer;
const uint32_t dubugPrintInterval = 5000; // micros between prints

void dataHandler(RPlidar* lidarPtr, uint16_t dist, uint16_t angle_q6, uint8_t newRotFlag, int8_t quality) {
  float distFloat = dist; // unit is mm directly
  float angleDegreesFloat = angle_q6 * 0.015625; // angle comes in 'q6' format, so divide by (1<<6)=64 (or multiply by 1/64) (or bitshift to the right by 6) to get angle in degrees
  // alternatively, you could use bitshifting to divide the angleDegreesFloat slightly faster. Something like:
//  float angleDegreesFloat = angle_q6;   angleDegreesFloat = (float&)(((uint32_t&)angleDegreesFloat)-=(((uint32_t)6)<<23)); // subtract 6 from the float's exponent, thereby dividing it by 2^6=64
//
//  debugPrintCounter++;
//  if(debugPrintCounter >= debugPrintThreshold) {  // (debugPrintCounter >= (lidarPtr->lidarSerial.available())) {  // dynamic?
//    debugPrintCounter = 0;
  if((micros()-debugPrintTimer) >= dubugPrintInterval) {  // (debugPrintCounter >= (lidarPtr->lidarSerial.available())) {  // dynamic?
    debugPrintTimer = micros();
    //// printing all the data is too slow (there's too much data), so this may cause packet loss (due to buffer overflow).
    //Serial.println(lidarPtr->lidarSerial.available());
    //Serial.print("DH: "); Serial.print(dist); Serial.print("  \t"); Serial.print(angle_q6); Serial.print('\t'); Serial.print(newRotFlag); Serial.print('\t'); Serial.println(quality);
    String dataToPrint = String(millis()) + '\t';
    dataToPrint += String(dist) + "  \t" + String(angle_q6);
    dataToPrint += '\t' + String(lidarPtr->packetCount) + '\t' + String(lidarPtr->rotationCount);
    dataToPrint += '\t' + String(newRotFlag) + '\t' + String(quality);
    dataToPrint += '\t' + String(lidarPtr->rawAnglePerMillisecond()) + '\t' + String(lidarPtr->RPM());
    Serial.println(dataToPrint);
  }
}

void setup() {
  //Setup motor:
  motorHandler.begin(3, 5000, 95); // Set PWM on pin 4, 5kHz frequency, 95% max duty cycle
  vTaskDelay(pdMS_TO_TICKS(10));
  motorHandler.set_frequency(10000); // Set LIDAR speed and frequency
  vTaskDelay(pdMS_TO_TICKS(10));
  motorHandler.set_frequency(50000); // Set LIDAR speed and frequency





  Serial.begin(115200);
  Serial.println();

  lidar.init(9, 10); // RX, TX pins
  lidar.postParseCallback = dataHandler; // set dat handler function

  lidar.printLidarInfo();
//  lidar.printLidarHealth();
//  lidar.printLidarSamplerate();
//  lidar.printLidarConfig();
  Serial.println();

  if(!lidar.connectionCheck()) { Serial.println("connectionCheck() failed"); while(1) {} }

  delay(10);
  // set pwm
  //bool startSuccess = lidar.startStandardScan();
  //bool startSuccess = lidar.startExpressScan(EXPRESS_SCAN_WORKING_MODE_LEGACY);
  bool startSuccess = lidar.startExpressScan(EXPRESS_SCAN_WORKING_MODE_BOOST);
//  Serial.print("startSuccess: "); Serial.println(startSuccess);
}

void loop() {
//  if(Serial.available()) { lidar.lidarSerial.write(Serial.read()); }
//  if(lidar.lidarSerial.available()) { Serial.write(lidar.lidarSerial.read()); }
//  if(millis() >= 5000) { motorHandler.setPWM(0); while(1) {} }
  
  if(keepSpinning) {
    uint32_t extraSpeedTimer = micros();
    int8_t datapointsProcessed = lidar.handleData(false, false); // read lidar data and send it to the callback function. Parameters are: (includeInvalidMeasurements, waitForChecksum)
    // includeInvalidMeasurements means sending data where the measurement failed (out of range or too close or bad surface, etc. it's when distance == 0)
    // waitForChecksum (only applies to express scans) means whether you wait for the whole packet to come, or to process data as it comes in (checksum is still checked when the whole packet is there, but the bad data may have already been sent to the callback)
//    extraSpeedTimer = micros() - extraSpeedTimer;
//    if(extraSpeedTimer > 40) { Serial.println(extraSpeedTimer); }

    if(datapointsProcessed < 0) { keepSpinning = false; lidar.stopScan(); } // handleData() returns -1 if it encounters an error
    //if(lidar.packetCount >= 200) { keepSpinning = false; lidar.stopScan(); }  // stop scanning after a while
  } else {
    // stop lidar
  }
}