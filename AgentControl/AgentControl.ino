#include <WiFi.h> // ESP32 Wi-Fi Library
#include <WebServer.h> // WebServer Library for ESP32
#include <WebSocketsClient.h> // WebSocket Client Library for WebSocket
#include <ArduinoJson.h> // Arduino JSON Library
#include <YDLidar.h>
#include <esp_timer.h>
#include "Mecanum.h"
#include "MinIMU9.h"
#include <SPI.h>
//#include <mcp2515.h>

// D-
#define MOUSE_DATA 14
// D+
#define MOUSE_CLOCK 27
void writePS2(byte Data);
unsigned long int readPS2();
volatile unsigned long int MouseRDPacket[3]; // 3 11-bit packets: buttons, x movement, y movement

#define RXD2 16
#define TXD2 17
//#define LIDAR_RX 33
//#define LIDAR_TX 32
#define lidar_start 0xA5
#define lidar_scan 0x60
#define lidar_stop 0x65
#define lidar_increase_freq_0_1 0x09
#define lidar_increase_freq_1 0x0B
#define lidar_get_freq 0x0D
#define lidar_restart 0x40
#define num_lidar_points 666

// Define pins for motor controllers
//#define m1_pwm 23
//#define m1_gpio 19
//#define m2_pwm 18
//#define m2_gpio 5
//#define m3_pwm 17
//#define m3_gpio 16
//#define m4_pwm 4
//#define m4_gpio 0

//#define m1_pwm 27
//#define m1_gpio 14
//#define m2_pwm 26
//#define m2_gpio 15
//#define m3_pwm 17
//#define m3_gpio 16
//#define m4_pwm 4
//#define m4_gpio 0

#define m1_pwm 25
#define m1_gpio 26
#define m2_pwm 27
#define m2_gpio 14
#define m3_pwm 32
#define m3_gpio 33
#define m4_pwm 4
#define m4_gpio 0

// Define pins for hall effect sensors (if any)
// GPIO6, 7, and 8 seemingly can't be used as normal GPIO
#define m1_hall_effect 36
#define m2_hall_effect 39
#define m3_hall_effect 34
#define m4_hall_effect -1 /*35*/

#define tempSolenoid 2
//#define tempSolenoidButton 26

// Define number of readings per revolution for hall effect sensors
#define hallReadingsPerRev 8

//#define LED_Status 2

const char* ssid = "hive"; // Wi-Fi SSID
const char* password = "password@1";//"password@1"; // Wi-Fi Password
const char* webSocketServerIP = "192.168.137.68";//"192.168.137.237";//"192.168.137.137"; // IP Address of server (Raspberry Pi)
const int webSocketPort = 80;
WebSocketsClient webSocket; // WebSocket client class instance
const int buffSize = 2 * 1024;
StaticJsonDocument<buffSize> incomingDoc;
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
  _geometry = 0,
  _laserscan,
  _imu,
  _odom,
  _rpm
};
uint64_t lastMouseReading = esp_timer_get_time(); // used for timeout

MinIMU9 imu;
YDLidar lidar;
const scanPoint &lidarPoint = lidar.getCurrentScanPoint();
int error = 1;

int Motor::HallReadingsPerRev = hallReadingsPerRev;

// Motor(int speedPin, int directionPin, [int hallEffectPin = -1], [bool inverse = false])
Motor m1 = Motor(m1_pwm, m1_gpio, m1_hall_effect, true);
Motor m2 = Motor(m2_pwm, m2_gpio, m2_hall_effect);
Motor m3 = Motor(m3_pwm, m3_gpio, m3_hall_effect, true);
Motor m4 = Motor(m4_pwm, m4_gpio, m4_hall_effect);

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

#pragma pack(push, 1)
typedef struct
{
  uint8_t type;
  uint32_t seqID; // 4 bytes
  uint16_t numPoints;
  uint16_t angles[2]; // angles times * 100
  uint16_t ranges[num_lidar_points];
//  uint8_t intensities[num_lidar_points];
} LiDAR_Data_t;
LiDAR_Data_t LiDAR_Data = {.type = (uint8_t)_laserscan, .numPoints = num_lidar_points};

typedef struct
{
  uint8_t type;
  uint32_t seqID; // 4 bytes
  float a[3];
  float g[3];
  float m[3];
} IMU_Data_t;
IMU_Data_t IMU_Data = {.type = (uint8_t)_imu};

typedef struct
{
  uint8_t type;
  uint32_t seqID; // 4 bytes
  float x;
  float y;
} Odom_Data_t;
Odom_Data_t Odom_Data = {.type = (uint8_t)_odom};

typedef struct
{
  uint8_t type;
  uint32_t seqID; // 4 bytes
  int32_t m1;
  int32_t m2;
  int32_t m3;
  int32_t m4;
} RPM_Data_t;
RPM_Data_t RPM_Data = {.type = (uint8_t)_rpm};

typedef struct
{
  uint32_t type;
  float x;
  float y;
  float z;
  uint32_t payload;
} CMD_Vel_Data_t;
CMD_Vel_Data_t* CMD_Vel_Data;
#pragma pack(pop)

TaskHandle_t LiDAR_Task;
TaskHandle_t Mouse_Task;
TaskHandle_t SendData_Task;
SET_LOOP_TASK_STACK_SIZE(1024 * 10);

unsigned long lastDriveCmd = millis();

//struct can_frame canMsg;
//MCP2515 mcp2515(5);

void setup()
{
  drivetrain.stop();
//  mcp2515.reset();
//  mcp2515.setBitrate(CAN_1000KBPS, MCP_8MHZ);
//  mcp2515.setNormalMode();
  pinMode(tempSolenoid, OUTPUT);
//  pinMode(tempSolenoidButton, INPUT);
  lockSolenoid1();
  Serial.begin(115200);
  setCpuFrequencyMhz((uint32_t)240);
  Serial.print("CPU Clock: ");
  Serial.print(getCpuFrequencyMhz());
  Serial.println(" MHz");
//  {
//    static unsigned long int rd[4];
//    pinMode(MOUSE_CLOCK, INPUT);
//    pinMode(MOUSE_DATA, INPUT);
//    writePS2(0xFF);    // reset mouse
//    rd[0] = readPS2(); // acknowledge: should be 0xFA
//    rd[1] = readPS2(); // self test:   should be 0xAA
//    rd[2] = readPS2(); // mouse id:    Should be 0x00
//    
//    writePS2(0xF4);    // Enable mouse
//    rd[3] = readPS2(); // acknowledge: should be 0xFA
//
//    for(int i = 0; i < 4; i ++) {
//      rd[i] = (rd[i] & 0b00111111110) >> 1; // take off pairity, start & stop bits
//    }
//   
//   // print startup sequence
//    Serial.println("FF reset command");
//    Serial.println(rd[0]  , HEX);
//    Serial.println(rd[1]  , HEX);
//    Serial.println(rd[2]  , HEX);
//    Serial.println("F4 Enable");
//    Serial.println(rd[3]  , HEX);
//  }
//  {
////    m4.drive((int)(255 * 1));
//    for (;;)
//    {
//      drivetrain.drive(-0.5, 0, 0);
////      Serial.print(m1.getRPM());
////      Serial.print("\t");
////      Serial.print(m2.getRPM());
////      Serial.print("\t");
////      Serial.print(m3.getRPM());
////      Serial.print("\t");
//      Serial.println(m4.returnRPM());
////      float rpm = m4.getRPM();
////      if (rpm > 0 || !digitalRead(m4_hall_effect))
////      {
////        Serial.print(digitalRead(m4_hall_effect));
////        Serial.print("\t");
////        Serial.println(rpm);
////      }
//    }
//  }
  for (int i = 0; i < LiDAR_Data.numPoints; i++)
  {
//    LiDAR_Data.angles[i] = 0;
    LiDAR_Data.ranges[i] = 0;
//    LiDAR_Data.intensities[i] = 0;
  }
  lidar.setIntensity(true);
  lidar.setSingleChannel(false);
  lidar.begin(Serial2, 230400, RXD2, TXD2);
//  lidar.stop();
  
  for (int i = 0; i < 6; ++i)
  {
    Serial2.write(lidar_start);
    Serial2.write(lidar_increase_freq_1);
    delay(25);
  }

  for (int i = 0; i < 10; ++i)
  {
    Serial2.write(lidar_start);
    Serial2.write(lidar_increase_freq_0_1);
    delay(25);
  }
  
//  lidar.setMotorSpeed(0);
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
  lidar.setMotorSpeed(5);

//  pinMode(LED_Status, OUTPUT);
  xTaskCreatePinnedToCore(
    SendData,
    "SendData",
    1024 * 12,
    NULL,
    tskIDLE_PRIORITY,
    &SendData_Task,
    0
  );
//  xTaskCreatePinnedToCore(
//    continuousLidarData,
//    "continuousLidarData",
//    1024 * 12,
//    NULL,
//    1,
//    &LiDAR_Task,
//    1
//  );
//  xTaskCreatePinnedToCore(
//    GetLidarData,
//    "GetLidarData",
//    1024 * 5,
//    NULL,
//    tskIDLE_PRIORITY,
//    &LiDAR_Task,
//    0
//  );
//  xTaskCreate(
//    GetLidarData,
//    "GetLidarData",
//    1024 * 5,
//    NULL,
//    tskIDLE_PRIORITY,
//    &LiDAR_Task
//  );
//  xTaskCreatePinnedToCore(
//    ReadMouse,
//    "ReadMouse",
//    1024 * 2,
//    NULL,
//    tskIDLE_PRIORITY,
//    &Mouse_Task,
//    0
//  );
}

void loop()
{
//  static bool lock = true;
//  static bool prevButtonState = false;
//  bool butt = digitalRead(tempSolenoidButton);
//  if (butt != prevButtonState)
//  {
//    prevButtonState = butt;
//    if (butt)
//    {
//      if (lock)
//      {
//        unlockSolenoid1();
//      }
//      else
//      {
//        lockSolenoid1();
//      }
//      lock = !lock;
//    }
//  }
//  SendData(NULL);
  drivetrain.update(); // Call every iteration (allows hall effect sensors to work)
//  RPM_Data.m1 = ((int)(1e3 * m1.getRPM()));
//  RPM_Data.m2 = ((int)(1e3 * m2.getRPM()));
//  RPM_Data.m3 = ((int)(1e3 * m3.getRPM()));
//  RPM_Data.m4 = ((int)(1e3 * m4.getRPM()));
//  if (imu.isOpen())
//  {
//    imu.read(); // get imu data every iteration
//    IMU_Data.a[0] = imu.a.x;
//    IMU_Data.a[1] = imu.a.y;
//    IMU_Data.a[2] = imu.a.z;
//    IMU_Data.g[0] = imu.g.x;
//    IMU_Data.g[1] = imu.g.y;
//    IMU_Data.g[2] = imu.g.z;
//    IMU_Data.m[0] = imu.m.x;
//    IMU_Data.m[1] = imu.m.y;
//    IMU_Data.m[2] = imu.m.z;
//  }
  if (lidar.isOpen()) GetLidarData(NULL); // get lidar data every iteration
//  GetLidarCAN();

  if (millis() - lastDriveCmd > 1000)
  {
    lastDriveCmd = millis(); // resetting the timer here just so drivetrain.stop() isn't called every following iteration
    drivetrain.stop();
//    lockSolenoid1();
  }
//
//  // Look into REMOTE vs streaming mode
//  static unsigned long int x;
//  static uint8_t mouseReadings = 0;
//  static uint64_t lastReading = esp_timer_get_time(); // used for timeout
//  static uint64_t velReading = esp_timer_get_time(); // used for velocity calculation
//  int xmvmt = 0, ymvmt = 0;
//  static float xInch = 0, yInch = 0;
//  {
////    while (digitalRead(MOUSE_DATA)){}
////    x = readPS2();
////    while (digitalRead(MOUSE_DATA)){}
////    xmvmt = readPS2();
////    while (digitalRead(MOUSE_DATA)){}
////    ymvmt = readPS2();
//
//    if (esp_timer_get_time() - lastReading > 10*1000)
//    {
//      Odom_Data.x = 0;
//      Odom_Data.y = 0;
//      mouseReadings = 0;
//      lastReading = esp_timer_get_time();
//    }
//    if (mouseReadings == 0)
//    {
//      for (int i = 0; i < 10 && digitalRead(MOUSE_DATA); i++);
//      if (!digitalRead(MOUSE_DATA))
//      {
//        x = readPS2();
//        lastReading = esp_timer_get_time();
//        ++mouseReadings;
//      }
//    }
//    if (mouseReadings == 1)
//    {
//      for (int i = 0; i < 10 && digitalRead(MOUSE_DATA); i++);
//      if (!digitalRead(MOUSE_DATA))
//      {
//        xmvmt = readPS2();
//        lastReading = esp_timer_get_time();
//        ++mouseReadings;
//      }
//    }
//    if (mouseReadings == 2)
//    {
//      for (int i = 0; i < 10 && digitalRead(MOUSE_DATA); i++);
//      if (!digitalRead(MOUSE_DATA))
//      {
//        ymvmt = readPS2();
//        lastReading = esp_timer_get_time();
//        mouseReadings = 0;
//        
//        x = (x & 0b00111111110) >> 1; //take off pairity, start & stop bits
//    
////        if((x & 0b00001000) && (x & 0b00000111)&& !(x & 0b11000000) ) {
////          Serial.print(x, BIN);
////          Serial.println(" mouse pressed");
////        }
////        Serial.print("X:");
//        xmvmt = (xmvmt & 0b00111111110) >> 1;  // take off pairity, start & stop bits
//        if((x & 0b00010000) >> 4) {            // if negative, process 9-bit 2s complement to regular signed int type
//          xmvmt = -((~xmvmt & 0b11111111) + 1);
//        }
//        xInch += xmvmt / 1000.0;
////        Serial.print(xmvmt);
////        Serial.print("\tin:");
////        Serial.print(xInch, 3);
////        Serial.print("\tY:");
//         
//        ymvmt = (ymvmt & 0b00111111110) >> 1; // take off pairity, start & stop bits
//        if((x & 0b00100000) >> 5) {           // if negative, process 9-bit 2s complement to regular signed int type
//          ymvmt = -((~ymvmt &0b11111111) + 1);
//        }
//        float secs = (lastReading - velReading) / 1e6; // microseconds to seconds
//        Odom_Data.x = (xmvmt * 2.54e-5) / secs; // meters per second
//        Odom_Data.y = (ymvmt * 2.54e-5) / secs; // meters per second
//        velReading = lastReading;
////        Serial.print(Odom_Data.x);
////        Serial.print("\t");
////        Serial.println(Odom_Data.y);
//        yInch += ymvmt / 1000.0;
////        Serial.print(ymvmt);
////        Serial.print("\tin:");
////        Serial.print(yInch, 3);
////        Serial.println(" ");
//      }
//    }
//  }
}

//void continuousLidarData(void* pvParameters)
//{
//  for (;;)
//  {
//    GetLidarCAN();
//  }
//}
//
//void GetLidarCAN()
//{
//  static INT16UNION_t INT16UNION;
//  if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK)
//  {
//    if (canMsg.can_id == 0x0F6)
//    {
//      if (canMsg.can_dlc >= 4)
//      {
//        INT16UNION.bytes[0] = canMsg.data[0];
//        INT16UNION.bytes[1] = canMsg.data[1];
//        uint16_t angle = INT16UNION.number;
//        INT16UNION.bytes[0] = canMsg.data[2];
//        INT16UNION.bytes[1] = canMsg.data[3];
//        uint16_t range = INT16UNION.number;
//        uint16_t point_num = (360 - angle/100.0) * (num_lidar_points / 360.0);
//        if (point_num == 0 || point_num == num_lidar_points - 1)
//        {
//          LiDAR_Data.angles[!(bool)point_num] = 360 - angle;
//        }
//        LiDAR_Data.ranges[point_num] = range;
//      }
//      if (canMsg.can_dlc == 8)
//      {
//        INT16UNION.bytes[0] = canMsg.data[4];
//        INT16UNION.bytes[1] = canMsg.data[5];
//        uint16_t angle = INT16UNION.number;
//        INT16UNION.bytes[0] = canMsg.data[6];
//        INT16UNION.bytes[1] = canMsg.data[7];
//        uint16_t range = INT16UNION.number;
//        uint16_t point_num = (360 - angle/100.0) * (num_lidar_points / 360.0);
//        if (point_num == 0 || point_num == num_lidar_points - 1)
//        {
//          LiDAR_Data.angles[!(bool)point_num] = 360 - angle;
//        }
//        LiDAR_Data.ranges[point_num] = range;
//      }
//    }
//  }
//}

void GetLidarData(void* pvParameters)
{
//  for (;;)
  {
    if (lidar.isOpen())
    {
      // LOOK INTO MEMORY ON SECONDARY MCU BOARDS TO SEE IF A BUFFER IS VIABLE
    //  static int i = 0;
      // lidar.waitScanDot() will update the values of lidarPoint, if the result is ok
      if (lidar.waitScanDot() == RESULT_OK)
      {
        uint16_t point_num = (360 - lidarPoint.angle) * (num_lidar_points / 360.0);
        if (point_num == 0 || point_num == num_lidar_points - 1)
        {
          LiDAR_Data.angles[!(bool)point_num] = (uint16_t)((360 - lidarPoint.angle) * 100);
        }
//        LiDAR_Data.angles[point_num] = (uint16_t)((360 - lidarPoint.angle) * 100);
        LiDAR_Data.ranges[point_num] = (uint16_t)(lidarPoint.distance);
//        LiDAR_Data.intensities[point_num] = (uint8_t)(lidarPoint.quality);
    //    Serial.print(i);
    //    Serial.print("\tAngle:");
    //    Serial.print(LiDAR_Data.angles[i]);
    //    Serial.print("\tRange:");
    //    Serial.print(LiDAR_Data.ranges[i]);
    //    Serial.print("\tInten:");
    //    Serial.println(LiDAR_Data.intensities[i]);
    //    Serial.print("Point angle: ");
    //    Serial.print(lidarPoint.angle, 3);
    //    Serial.print("\tdist: ");
    //    Serial.print(lidarPoint.distance, 3);
    //    Serial.print("\tintensity: ");
    //    Serial.println(lidarPoint.quality);
      }
      else
      {
        Serial.println("Fail to get scan point!");
        delay(1);
      }
    }
  }
}

void lockSolenoid1()
{
  digitalWrite(tempSolenoid, LOW);
}

void unlockSolenoid1()
{
  digitalWrite(tempSolenoid, HIGH);
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

void ReadMouse(void* pvParameters)
{
  static unsigned long int x;
  static uint64_t velReading = esp_timer_get_time(); // used for velocity calculation
  for(;;)
  {
    while (digitalRead(MOUSE_DATA)){}
    x = readPS2();
    while (digitalRead(MOUSE_DATA)){}
    int xmvmt = readPS2();
    while (digitalRead(MOUSE_DATA)){}
    int ymvmt = readPS2();
    lastMouseReading = esp_timer_get_time();
    
    x = (x & 0b00111111110) >> 1; //take off pairity, start & stop bits
    xmvmt = (xmvmt & 0b00111111110) >> 1;  // take off pairity, start & stop bits
    if((x & 0b00010000) >> 4) // if negative, process 9-bit 2s complement to regular signed int type
    {
      xmvmt = -((~xmvmt & 0b11111111) + 1);
    }
  
    ymvmt = (ymvmt & 0b00111111110) >> 1; // take off pairity, start & stop bits
    if((x & 0b00100000) >> 5) // if negative, process 9-bit 2s complement to regular signed int type
    {
      ymvmt = -((~ymvmt &0b11111111) + 1);
    }
  
    float secs = (lastMouseReading - velReading) / 1e6; // microseconds to seconds
    Odom_Data.x = (xmvmt * 2.54e-5) / secs; // meters per second
    Odom_Data.y = (ymvmt * 2.54e-5) / secs; // meters per second
    velReading = lastMouseReading;
  }
}

void SendData(void* pvParameters)
{
//  char lidarBuff[buffSize]; // must NOT be static
//  char IMUBuff[buffSize]; // must NOT be static
  static uint32_t seqID = 0;
  for (;; seqID++)
  {
//    seqID++;
//    GetLidarCAN();
    webSocket.loop(); // Keep the socket alive
//    if (lidar.isOpen()) GetLidarData(); // get lidar data every iteration
//     LiDAR
    if (lidar.isOpen())
    {
////      GetLidarData();
      LiDAR_Data.seqID = seqID;
      if (webSocket.isConnected())
        webSocket.sendBIN((uint8_t*)&LiDAR_Data, sizeof(LiDAR_Data));
    }
    
    // IMU
    if (imu.isOpen())
    {
      IMU_Data.seqID = seqID;
      if (webSocket.isConnected())
        webSocket.sendBIN((uint8_t*)&IMU_Data, sizeof(IMU_Data));
    }

    // Mouse Optical Sensor
//    {
//      if (esp_timer_get_time() - lastMouseReading > 150 * 1000)
//      {
//        lastMouseReading = esp_timer_get_time();
//        Odom_Data.x = Odom_Data.y = 0;
//      }
//      Odom_Data.seqID = seqID;
////      Serial.print(Odom_Data.x, 4);
////      Serial.print("\t");
////      Serial.println(Odom_Data.y, 4);
//      if (webSocket.isConnected())
//        webSocket.sendBIN((uint8_t*)&Odom_Data, sizeof(Odom_Data));
//    }

    // RPM
//    {
////      Serial.print(RPM_Data.m1);
////      Serial.print("\t");
////      Serial.print(RPM_Data.m2);
////      Serial.print("\t");
////      Serial.print(RPM_Data.m3);
////      Serial.print("\t");
////      Serial.println(RPM_Data.m4);
//      RPM_Data.seqID = seqID;
//      if (webSocket.isConnected())
//        webSocket.sendBIN((uint8_t*)&RPM_Data, sizeof(RPM_Data));
//    }
  }
}

void webSocketEvent(WStype_t type, uint8_t * payload, size_t length)
{
//  bool lock = false; // temp
//  Serial.print("Websocket Event Triggered. ");
//  Serial.print("type: ");
//  Serial.println(WStypes[type]);
  /*if (type == WStype_TEXT)
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
    if ((MessageType)incomingDoc["type"] == _frame)
    {
//      strncpy(LiDAR_Data.frameID, _data, sizeof(LiDAR_Data.frameID));
//      strcat(LiDAR_Data.frameID, "/laser");
//
//      strncpy(IMU_Data.frameID, _data, sizeof(IMU_Data.frameID));
//      strcat(IMU_Data.frameID, "/imu");
//
//      strncpy(Odom_Data.frameID, _data, sizeof(Odom_Data.frameID));
//      strcat(Odom_Data.frameID, "/odom");
    }
    else if ((MessageType)incomingDoc["type"] == _geometry)
    {
      float x = (float)incomingDoc["data"]["linear"]["x"];
      float y = (float)incomingDoc["data"]["linear"]["y"];
      float z = (float)incomingDoc["data"]["angular"]["z"];
      drivetrain.drive(x, y, z);
      lastDriveCmd = millis();
//      lock = !(x <= 0.51 && x >= 0.49); // temp
    }
//    if (lock) // temp
//      lockSolenoid1();
//    else
//      unlockSolenoid1();
  }
  else*/ if (type == WStype_BIN)
  {
    CMD_Vel_Data = (CMD_Vel_Data_t*)payload;
    drivetrain.drive(CMD_Vel_Data->x, CMD_Vel_Data->y, CMD_Vel_Data->z);
    lastDriveCmd = millis();
    if (CMD_Vel_Data->payload)
    {
      unlockSolenoid1();
    }
    else
    {
      lockSolenoid1();
    }
  }
  else if (type == WStype_DISCONNECTED)
  {
    Serial.println("Websocket Event Triggered. type: WStype_DISCONNECTED");
    drivetrain.stop();
    lockSolenoid1(); // temp
  }
  else if (type == WStype_CONNECTED)
  {
    Serial.println("Websocket Event Triggered. type: WStype_CONNECTED");
  }
  incomingDoc.clear();
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

// write a byte to the mouse
void writePS2( byte Data) {

  // bring the clock low to stop mouse from communicating
  pinMode(MOUSE_CLOCK, OUTPUT); 
  digitalWrite(MOUSE_CLOCK, LOW);
  delayMicroseconds(200); // wait for mouse

  // bring data low to tell mouse that the host wants to communicate
  pinMode(MOUSE_DATA, OUTPUT);
  digitalWrite(MOUSE_DATA, LOW);
  delayMicroseconds(50);
  
  // release control of clock by putting it back to a input
  pinMode(MOUSE_CLOCK, INPUT_PULLUP);

  // now clk is high because of pullup
  
  while(digitalRead(MOUSE_CLOCK)){} // wait for clock to go low again
  
  // when the mouse brings clock low again, it is ready to communicate

  // shift out byte from LSB to MSB
  int pairck = 0;                               // track # of 1s in byte for pairity
  for(int i = 0; i < 8; i++) {
    digitalWrite(MOUSE_DATA, (Data & (1 << i)) >> i); // shift out a bit when clock is low
    pairck += ((Data & (1 << i)) >> i);         // count if 1 for pairity check later
    while(digitalRead(MOUSE_CLOCK) == LOW){}            // wait for clock to go high
    while(digitalRead(MOUSE_CLOCK) == HIGH){}           // wait for clock to go low
  }

  // add pairity bit if needed so that the number of 1s in the byte shifted out plus the pairity bit is always odd
  if(pairck %2 == 0) {
    digitalWrite(MOUSE_DATA, HIGH);
  }
  else {
    digitalWrite(MOUSE_DATA, LOW);
  }
  while(digitalRead(MOUSE_CLOCK) == LOW){} // wait for clock to go high
  while(digitalRead(MOUSE_CLOCK) == HIGH){} // wait for clock to go low
  

  pinMode(MOUSE_DATA, INPUT_PULLUP);       // release control of data
  while(digitalRead(MOUSE_DATA) == HIGH){} // wait for data to go low
  while(digitalRead(MOUSE_CLOCK)  == HIGH){} // wait for clk to go low
  
  while(digitalRead(MOUSE_DATA) == LOW){} // wait for data to go high again
  while(digitalRead(MOUSE_CLOCK) == LOW){}  // wait for clk to go high again
  
}

// read 11 byte packet from the mouse
unsigned long int readPS2 () {
  unsigned long int b;

  // shift in 11 bits from MSB to LSB
  for(int i = 0; i < 11; i++) {
    while(digitalRead(MOUSE_CLOCK) == HIGH){} // wait for the clock to go LOW
    b += (digitalRead(MOUSE_DATA) << i);    // shift in a bit 
    while(digitalRead(MOUSE_CLOCK) == LOW){}  // wait here while the clock is still low
   }
   return b;
}
