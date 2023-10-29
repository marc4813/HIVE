#include <WiFi.h> // ESP32 Wi-Fi Library
#include <WebServer.h> // WebServer Library for ESP32
#include <WebSocketsClient.h> // WebSocket Client Library for WebSocket
#include <ArduinoJson.h> // Arduino JSON Library
#include <YDLidar.h>
#include "Mecanum.h"
#include "MinIMU9.h"

#define RXD2 16
#define TXD2 17
#define LIDAR_RX 32
#define LIDAR_TX 33
#define lidar_start 0xA5
#define lidar_scan 0x60
#define lidar_stop 0x65
#define lidar_increase_freq_0_1 0x09
#define lidar_increase_freq_1 0x0B
#define lidar_get_freq 0x0D
#define lidar_restart 0x40
#define num_lidar_points 500

// Define pins for motor controllers
#define m1_pwm 23
#define m1_gpio 19
#define m2_pwm 18
#define m2_gpio 5
#define m3_pwm 17
#define m3_gpio 16
#define m4_pwm 4
#define m4_gpio 0

// Define pins for hall effect sensors (if any)
// GPIO6, 7, and 8 seemingly can't be used as normal GPIO
#define m1_hall_effect 15
#define m2_hall_effect 8
#define m3_hall_effect 7
#define m4_hall_effect 6

// Define number of readings per revolution for hall effect sensors
#define hallReadingsPerRev 8

#define LED_Status 2

const char* ssid = "hive"; // Wi-Fi SSID
const char* password = "password@1"; // Wi-Fi Password
const char* webSocketServerIP = "192.168.137.137"; // IP Address of server (Raspberry Pi)
const int webSocketPort = 80;
WebSocketsClient webSocket; // WebSocket client class instance
const int buffSize = 2 * 1024;// * 32;//1024 * 8;
//StaticJsonDocument<buffSize> lidarDoc;
//StaticJsonDocument<buffSize> IMUDoc;
StaticJsonDocument<buffSize> incomingDoc;
StaticJsonDocument<buffSize> outgoingDoc;
const String WStypes[11] =
{
  "WStype_ERROR",
  "WStype_DISCONNECTED",
  "WStype_CONNECTED",
  "WStype_TEXT",
  "WStype_BIN",
  "WStype_FRAGMENT_TEXT_START",
  "WStype_FRAGMENT_BIN_START",
  "WStype_FRAGMENT",
  "WStype_FRAGMENT_FIN",
  "WStype_PING",
  "WStype_PONG"
};

typedef enum MessageType
{
  _geometry = 1,
  _laserscan = 2,
  _imu = 3,
};
const String lidarFrame = "agent1/laser";
const String IMUFrame = "agent1/imu";

MinIMU9 imu;
YDLidar lidar;
const scanPoint &lidarPoint = lidar.getCurrentScanPoint();
int error = 1;

// Motor(int speedPin, int directionPin, [int hallEffectPin = -1], [bool inverse = false])
Motor m1 = Motor(m1_pwm, m1_gpio, m1_hall_effect);
Motor m2 = Motor(m2_pwm, m2_gpio, m2_hall_effect, true);
Motor m3 = Motor(m3_pwm, m3_gpio, m3_hall_effect);
Motor m4 = Motor(m4_pwm, m4_gpio, m4_hall_effect, true);

// Mecanum(Motor &m1, Motor &m2, Motor &m3, Motor &m4)
Mecanum drivetrain = Mecanum(m1, m2, m3, m4);

typedef union
{
  float number;
  uint8_t bytes[4];
} FLOATUNION_t;

typedef union
{
  uint16_t number;
  uint8_t bytes[2];
} INT16UNION_t;

typedef union
{
  uint32_t number;
  uint8_t bytes[4];
} INT32UNION_t;

typedef 

TaskHandle_t LiDAR_Task;
SET_LOOP_TASK_STACK_SIZE(1024 * 10);

unsigned long lastDriveCmd = millis();

void setup()
{
  drivetrain.stop();
  pinMode(m1_hall_effect, OUTPUT);
  lockSolenoid1();
  Serial.begin(115200);
  lidar.setIntensity(true);
  lidar.setSingleChannel(false);
  lidar.begin(Serial2, 230400, LIDAR_RX, LIDAR_TX);
  if (!lidar.isSingleChannel())
  {
    initLidar();
  }

  Wire.begin();
  WiFi.begin(ssid, password);
  Serial.print("Establishing connection");
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print(".");
    delay(500);
  }
  Serial.println();
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP()); // Print local IP address
  webSocket.begin(webSocketServerIP, webSocketPort, "/"); // address, port, URL path
  webSocket.onEvent(webSocketEvent); // WebSocket event handler
  webSocket.setReconnectInterval(5000); // if connection failed, retry every 5s

  for (int i = 0; i < 1000 && !imu.init(); i++)
  {
    Serial.println("Unable to initialize IMU!");
    delay(1);
  }

  if (lidar.startScan() != RESULT_OK)
  {
    Serial.println("Fail to start!");
  }

  pinMode(LED_Status, OUTPUT);
  xTaskCreatePinnedToCore(
    SendData,
    "SendData",
    1024 * 12,
    NULL,
    tskIDLE_PRIORITY,
    &LiDAR_Task,
    0
  );
}

void loop()
{
  webSocket.loop(); // Keep the socket alive
  if (imu.isOpen()) imu.read(); // get imu data every iteration
  if (lidar.isOpen()) GetLidarData(); // get lidar data every iteration
  
  if (millis() - lastDriveCmd > 1000)
  {
    lastDriveCmd = millis(); // resetting the timer here just so drivetrain.stop() isn't called every following iteration
    drivetrain.stop();
  }
}

void GetLidarData()
{
  // lidar.waitScanDot() will update the values of lidarPoint, if the result is ok
  if (lidar.waitScanDot(10) == RESULT_OK)
  {
    Serial.print("Point angle: ");
    Serial.print(lidarPoint.angle, 3);
    Serial.print("\tdist: ");
    Serial.print(lidarPoint.distance, 3);
    Serial.print("\tintensity: ");
    Serial.println(lidarPoint.quality);
  }
  else
  {
    Serial.println("Fail to get scan point!");
    delay(1);
  }
}

void lockSolenoid1()
{
  digitalWrite(m1_hall_effect, LOW);
}

void unlockSolenoid1()
{
  digitalWrite(m1_hall_effect, HIGH);
}

void initLidar()
{
  device_info di = {0};
  if (lidar.getDeviceInfo(di, 100) == RESULT_OK)
  {
    int _samp_rate = 4;
    String model;
    float freq = 7.0f;
    switch (di.model)
    {
      case 1:
        model = "F4";
        _samp_rate = 4;
        freq = 7.0;
        break;
      case 4:
        model = "S4";
        _samp_rate = 4;
        freq = 7.0;
        break;
      case 5:
        model = "G4";
        _samp_rate = 9;
        freq = 7.0;
        break;
      case 6:
        model = "X4";
        _samp_rate = 5;
        freq = 7.0;
        break;
      default:
        model = "Unknown";
    }

    uint16_t maxv = (uint16_t)(di.firmware_version >> 8);
    uint16_t midv = (uint16_t)(di.firmware_version & 0xff) / 10;
    uint16_t minv = (uint16_t)(di.firmware_version & 0xff) % 10;
    if (midv == 0)
    {
      midv = minv;
      minv = 0;
    }

    Serial.print("Firmware version: ");
    Serial.print(maxv);
    Serial.print(".");
    Serial.print(midv);
    Serial.print(".");
    Serial.println(minv);
    
    Serial.print("Hardware version: ");
    Serial.println(di.hardware_version);
    
    Serial.print("Model: ");
    Serial.println(model);
    
    Serial.print("Serial: ");
    for (int i = 0; i < 16; i++)
    {
      Serial.print(di.serialnum[i] & 0xff);
    }
    Serial.println();

    Serial.print("Sampling Rate: ");
    Serial.print(_samp_rate, DEC);
    Serial.println("K");

    Serial.print("Scan Frequency: ");
    Serial.print(freq, DEC);
    Serial.println("Hz");
  }
  else
  {
    Serial.println("Fail to get device info!");
  }

  delay(100);
  device_health hi = {0};
  if (lidar.getHealth(hi, 100) == RESULT_OK)
  {
    Serial.print("[YDLIDAR INFO] YDLIDAR running correctly! The health status:");
    Serial.println(hi.status == 0 ? "well" : "bad");
  }
  else
  {
    Serial.println("Fail to get health status!");
  }
}

void SendData(void* pvParameters)
{
//  char lidarBuff[buffSize]; // must NOT be static
//  char IMUBuff[buffSize]; // must NOT be static
  char outgoingBuff[buffSize];
  static unsigned long seqID = 0;
  for (;; seqID++)
  {
    // LiDAR
    if (lidar.isOpen())
    {
      outgoingDoc.clear(); // implicitly called by deserializeJson
      outgoingDoc["type"] = _laserscan;
      outgoingDoc["data"]["header"][0] = seqID;
      outgoingDoc["data"]["header"][1] = lidarFrame;
      outgoingDoc["data"]["intensity"] = lidarPoint.quality;
      outgoingDoc["data"]["angle"] = lidarPoint.angle;
      outgoingDoc["data"]["distance"] = lidarPoint.distance;
      size_t len = serializeJson(outgoingDoc, outgoingBuff);
      if (webSocket.isConnected())
        webSocket.sendTXT(outgoingBuff);
    }
    
    // IMU
    if (imu.isOpen())
    {
      outgoingDoc.clear(); // implicitly called by deserializeJson
      outgoingDoc["type"] = _imu;
      outgoingDoc["data"]["header"][0] = seqID;
      outgoingDoc["data"]["header"][1] = IMUFrame;
      outgoingDoc["data"]["a"]["x"] = imu.a.x;
      outgoingDoc["data"]["a"]["y"] = imu.a.y;
      outgoingDoc["data"]["a"]["z"] = imu.a.z;
      outgoingDoc["data"]["g"]["x"] = imu.g.x;
      outgoingDoc["data"]["g"]["y"] = imu.g.y;
      outgoingDoc["data"]["g"]["z"] = imu.g.z;
      outgoingDoc["data"]["m"]["x"] = imu.m.x;
      outgoingDoc["data"]["m"]["y"] = imu.m.y;
      outgoingDoc["data"]["m"]["z"] = imu.m.z;
      size_t len = serializeJson(outgoingDoc, outgoingBuff);
      if (webSocket.isConnected())
        webSocket.sendTXT(outgoingBuff);
    }
  }
}

void webSocketEvent(WStype_t type, uint8_t * payload, size_t length)
{
//  bool lock = false; // temp
  Serial.print("Websocket Event Triggered. ");
  Serial.print("type: ");
  Serial.println(WStypes[type]);
  if (type == WStype_TEXT)
  {
    // Deserialize incoming JSON String
    DeserializationError error = deserializeJson(incomingDoc, payload);
    if (error) // Print error msg if incoming String is not JSON formated
    {
      Serial.print(F("deserializeJson() failed: "));
      Serial.println(error.c_str());
      return;
    }
    MessageType messageType = incomingDoc["type"];
    String _data = incomingDoc["data"];
    Serial.println(_data);
    if (messageType == _geometry)
    {
      float x = incomingDoc["data"]["linear"]["x"];
      float y = incomingDoc["data"]["linear"]["y"];
      float z = incomingDoc["data"]["angular"]["z"];
      drivetrain.drive(x, y, z);
      lastDriveCmd = millis();
//      lock = !(x <= 0.51 && x >= 0.49); // temp
    }
//    if (lock) // temp
//      lockSolenoid1();
//    else
//      unlockSolenoid1();
  }
  else if (type == WStype_DISCONNECTED)
  {
    drivetrain.stop();
//    lockSolenoid1(); // temp
  }
}

// Displays command velocities that are sent to the drivetrain.
// Values should be between -1.0 and 1.0.
void printCommandVelocities()
{
  Serial.print("XVel: ");
  Serial.print(drivetrain.getXVel());
  Serial.print("\tYVel: ");
  Serial.print(drivetrain.getYVel());
  Serial.print("\tZVel: ");
  Serial.println(drivetrain.getZVel());
}

// Displays the raw PWM data sent to each motor.
// Values will be between 0 and 255, where either 0 or 255 can be max speed.
void printRawSpeeds()
{
  Serial.print("M1R: ");
  Serial.print(drivetrain.m1.getRawSpeed());
  Serial.print("\tM2R: ");
  Serial.print(drivetrain.m2.getRawSpeed());
  Serial.print("\tM3R: ");
  Serial.print(drivetrain.m3.getRawSpeed());
  Serial.print("\tM4R: ");
  Serial.println(drivetrain.m4.getRawSpeed());
}

// Displays the desired speeds of each motor relative to the robot.
// 255 is full speed forward, -255 is full speed reverse, and 0 is not moving.
void printObservedSpeeds()
{
  Serial.print("M1O: ");
  Serial.print(drivetrain.m1.getObservedSpeed());
  Serial.print("\tM2O: ");
  Serial.print(drivetrain.m2.getObservedSpeed());
  Serial.print("\tM3O: ");
  Serial.print(drivetrain.m3.getObservedSpeed());
  Serial.print("\tM4O: ");
  Serial.println(drivetrain.m4.getObservedSpeed());
}
