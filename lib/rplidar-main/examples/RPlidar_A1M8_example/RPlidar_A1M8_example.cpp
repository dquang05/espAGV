/*

NOTES:
if you're using the included serial adapter, the DTR pin is connected to the motor PWM control.
on boot (and on reset) it prints something like:
"
RP LIDAR System.
Firmware Ver 1.29 - rtm, HW Ver 7
Model: 18
"

some data retrieved from the lidar (model A1M8, bought on Aliexpress in Feb 2022):
from CMD_GET_INFO:
model: 24
firmware: 1.29
hardware: 7
serialNumber (HEX): D8 E0 ED F9 C7 E2 9B D1 A7 E3 9E F2 48 33 43 1B 
serialNumber (32bit): 4193116376 3516654279 4070499239 457388872 
serialNumber (64bit): 15103915123836575960 1964470250864829351 
from CMD_GET_HEALTH:
status: 0
errorCode (HEX): 0
from CMD_GET_SAMPLERATE:
standard: 508
express: 254
from CMD_GET_LIDAR_CONF: (and sub-commands)
GET_CONF_SCAN_MODE_COUNT: 5
scan mode: 0
  name: Standard
  sampletime: 130048 == 508us
  maxDist: 3072      == 12m
  ansType (HEX): 81  == CONF_SCAN_MODE_ANS_TYPE_STANDARD
scan mode: 1
  name: Express
  sampletime: 65024  == 254us
  maxDist: 3072      == 12m
  ansType (HEX): 82  == CONF_SCAN_MODE_ANS_TYPE_EXPRESS (== legacy???)
scan mode: 2
  name: Boost
  sampletime: 32512  == 127us
  maxDist: 3072      == 12m
  ansType (HEX): 84  == CONF_SCAN_MODE_ANS_TYPE_EXPRESS_EXTEND ???
scan mode: 3
  name: Sensitivity
  sampletime: 32512  == 127us
  maxDist: 3072      == 12m
  ansType (HEX): 84  == CONF_SCAN_MODE_ANS_TYPE_EXPRESS_EXTEND ???
scan mode: 4
  name: Stability
  sampletime: 51456  == 201us
  maxDist: 3072      == 12m
  ansType (HEX): 84  == CONF_SCAN_MODE_ANS_TYPE_EXPRESS_EXTEND ???
*/

#define lidarDebugSerial Serial

#include "thijs_rplidar.h"


struct lidarMotorHandler {  // not really needed (and (currently) very ESP32-bound) but somewhat futureproof
  const uint8_t pin;
  const uint32_t freq; //Hz
  //const uint8_t res; //bits (commented because i want to keep this thing simple, and changing variable sizes (templates?) is not
  const uint8_t channel; // an ESP32 ledc specific thing
  const bool activeHigh; // depends on your specific hardware setup (CTRL_MOTO should be driven to the same voltage as 5V_MOTO (which can range from 5 to 9V), i think)
  lidarMotorHandler(const uint8_t pin, const bool activeHigh=true, const uint32_t freq=500, /*const uint8_t res=8,*/ const uint8_t channel=0) : 
                    pin(pin), freq(freq), /*res(res),*/ channel(channel), activeHigh(activeHigh) {}
  void init() {
    ledcSetup(channel, freq, 8);
    ledcAttachPin(pin, channel);
    setPWM(0);
  }
  inline void setPWM(uint8_t newPWMval) {ledcWrite(channel, activeHigh ? newPWMval : (255-newPWMval));}
};

lidarMotorHandler motorHandler(27);
RPlidar lidar(Serial2);

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
  Serial.begin(115200);
  Serial.println();

  motorHandler.init();

  lidar.init(16, 17);
  lidar.postParseCallback = dataHandler; // set dat handler function

  lidar.printLidarInfo();
//  lidar.printLidarHealth();
//  lidar.printLidarSamplerate();
//  lidar.printLidarConfig();
  Serial.println();

  if(!lidar.connectionCheck()) { Serial.println("connectionCheck() failed"); while(1) {} }

  delay(10);
  motorHandler.setPWM(200);
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
    motorHandler.setPWM(0);
  }
}