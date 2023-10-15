#include <WiFi.h> // ESP32 Wi-Fi Library
#include <WebServer.h> // WebServer Library for ESP32
#include <WebSocketsClient.h> // WebSocket Client Library for WebSocket
#include <ArduinoJson.h> // Arduino JSON Library
#include <HardwareSerial.h>
#include "Mecanum.h"
#include "MinIMU9.h"

#define RXD2 16
#define TXD2 17
#define lidar_start 0xA5
#define lidar_scan 0x60
#define lidar_stop 0x65
#define lidar_increase_freq_0_1 0x09
#define lidar_increase_freq_1 0x0B
#define lidar_get_freq 0x0D
#define lidar_restart 0x40
#define num_lidar_points 500

// PS2 Controller Arduino Library: https://github.com/madsci1016/Arduino-PS2X
//#include <PS2X_lib.h>

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

// Define pins for PS2 controller breakout board
//#define ps2_data -1
//#define ps2_cmd -1
//#define ps2_att -1
//#define ps2_clk -1

#define LED_Status 2

const char* ssid = "hive"; // Wi-Fi SSID
const char* password = "password@1"; // Wi-Fi Password
const char* webSocketServerIP = "192.168.137.133"; // IP Address of server (Raspberry Pi)
const int webSocketPort = 80;
WebSocketsClient webSocket; // WebSocket client class instance
const int buffSize = 1024 * 32;//1024 * 8;
StaticJsonDocument<buffSize> doc; // Allocate a static JSON document
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
  _status = 3,
};
const String frame = "agent1/laser";
//const uint16_t headerSize = (32 / 8) + (frame.length() * sizeof(uint8_t));
//const uint16_t numDataPoints = 500;
//const uint16_t dataSize = 2 * (32 / 8) + headerSize + numDataPoints * (32 / 8);
char* lidarData = new char[num_lidar_points * 3];
float* lidarAngles = new float[num_lidar_points];

//PS2X ps2x;
MinIMU9 imu;
HardwareSerial LiDAR_UART(2); // use UART2
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

TaskHandle_t LiDAR_Task;
SET_LOOP_TASK_STACK_SIZE(36 * 1024);

void setup()
{
  Serial.begin(115200);
  LiDAR_UART.begin(230400, SERIAL_8N1, RXD2, TXD2);
  LiDAR_UART.write(lidar_start);
  LiDAR_UART.write(lidar_stop);
  
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

  for (int i = 0; i < num_lidar_points; i++)
  {
    lidarAngles[i] = -1;
    for (int j = 0; j < 3; j++)
    {
      lidarData[3*i + j] = 0;
    }
  }

//  while (!imu.init())
//  {
//    Serial.println("Unable to initialize IMU!");
//  }

  // Attempts to connect to PS2 controller a maximum of 50 times
  //  for (int i = 0; i < 50 && error; i++) {
  //    // config_gamepad(clock, command, attention, data, pressures?, rumble?)
  //    error = ps2x.config_gamepad(ps2_clk, ps2_cmd, ps2_att, ps2_data, false, false);
  //    delay(500);
  //  }

//  pinMode(LED_Status, OUTPUT);
//  xTaskCreatePinnedToCore(
//    TransmitLiDAR, // Task function
//    "TransmitLiDAR", // Task name
//    1024 * 32, // Task stack size (in words)
//    NULL, // Task parameters
//    tskIDLE_PRIORITY, // Task priority
//    &LiDAR_Task, // Task handle
//    0 // Core
//  );
  xTaskCreatePinnedToCore(
    SendData,
    "SendData",
    1024 * 36,
    NULL,
    tskIDLE_PRIORITY,
    &LiDAR_Task,
    0
  );
  TransmitLiDAR({});
}

void printFail(int i, char c)
{
  Serial.print("Fail");
  Serial.print(i);
  Serial.print(":\t");
  Serial.println(c, HEX);
}

void SendData(void* pvParameters)
{
  for (;;)
  {
    webSocket.loop(); // Keep the socket alive TEMPPPPPPPPPPPPPP
    doc.clear(); // implicitly called by deserializeJson
    static uint32_t seqID = 0;
    static INT32UNION_t seqIDUnion;
    seqIDUnion.number = seqID++;
    for (int i = 0; i < num_lidar_points; i++)
    {
      doc["data"]["intensity"][i] = (byte)lidarData[3*i];
      doc["data"]["distance"][i] = (uint16_t)(lidarData[3*i + 2] << 6) + (lidarData[3*i + 1] >> 2);
      doc["data"]["flag"][i] = (byte)(lidarData[3*i + 1] & 0x03);
      doc["data"]["angles"][i] = lidarAngles[i];
    }
    doc["data"]["header"][0] = seqID;
    doc["data"]["header"][1] = frame;
    doc["data"]["header"][2] = num_lidar_points;
    doc["type"] = _laserscan;
    char buff[buffSize];
    size_t len = serializeJson(doc, buff);
    webSocket.sendTXT(buff); // Send acknowledgement
  }
}

void TransmitLiDAR(void* pvParameters)
{
  delay(500);
  LiDAR_UART.write(lidar_start);
  LiDAR_UART.write(lidar_scan);
  for(;;)
  {
//    static uint32_t seqID = 0;
//    static INT32UNION_t seqIDUnion;
//    seqIDUnion.number = seqID++;
    static uint16_t start_angle = 0;
    static uint16_t end_angle = 0;
    static uint16_t check_code = 0;
    static uint16_t sample_quantity = 0;
    static INT16UNION_t int16union;
    static char data_rcvd;
  
    if (LiDAR_UART.peek() >= 0)
    {
      data_rcvd = LiDAR_UART.read();
      if (data_rcvd == 0xAA) // packet header
      {
        data_rcvd = LiDAR_UART.read();
        if (data_rcvd == 0x55) // packet header
        {
          data_rcvd = LiDAR_UART.read();
          if (true) // package type
          {
            Serial.print("Package type:\t");
            Serial.println(data_rcvd, HEX);
            data_rcvd = LiDAR_UART.read();
            sample_quantity = data_rcvd;
            if (sample_quantity) // sample quantity
            {
              Serial.print("Sample quantity:\t");
              Serial.println(data_rcvd, HEX);
              
              data_rcvd = LiDAR_UART.read();
              int16union.bytes[0] = data_rcvd;
              data_rcvd = LiDAR_UART.read();
              int16union.bytes[1] = data_rcvd;
              start_angle = int16union.number;
              start_angle = (start_angle>>1) / 64;
              Serial.print("Start angle:\t");
              Serial.println(start_angle);
  
              data_rcvd = LiDAR_UART.read();
              int16union.bytes[0] = data_rcvd;
              data_rcvd = LiDAR_UART.read();
              int16union.bytes[1] = data_rcvd;
              end_angle = int16union.number;
              end_angle = (end_angle>>1) / 64;
              Serial.print("End angle:\t");
              Serial.println(end_angle);
  
              data_rcvd = LiDAR_UART.read();
              int16union.bytes[0] = data_rcvd;
              data_rcvd = LiDAR_UART.read();
              int16union.bytes[1] = data_rcvd;
              check_code = int16union.number;
              Serial.print("Check code:\t");
              Serial.println(check_code, BIN);
  
              Serial.println("LiDAR data:");
              for (int i = 0; i < sample_quantity; i++)
              {
                float angle;
                if (end_angle == start_angle || sample_quantity == 1)
                {
                  angle = start_angle;
                }
                else if (end_angle > start_angle)
                {
                  angle = ((end_angle - start_angle) / (float)(sample_quantity - 1)) * i + start_angle;
                }
                else
                {
                  angle = ((end_angle + 360 - start_angle) / (float)(sample_quantity - 1)) * i + start_angle;
                }
                if (angle >= 360) angle -= 360;
                uint16_t point_num = angle * (num_lidar_points / 360.0);
                lidarAngles[point_num] = angle;
                for (int j = 0; j < 3; j++)
                {
                  data_rcvd = LiDAR_UART.read();
                  lidarData[3*point_num + j] = data_rcvd;
                }
              }
              for (int i = 0; i < num_lidar_points; i++)
              {
                Serial.print(lidarAngles[i]);
                Serial.print("\t");
                Serial.print(lidarData[3*i], DEC);
                Serial.print("\t");
                Serial.print((lidarData[3*i + 2] << 6) + (lidarData[3*i + 1] >> 2), DEC);
                Serial.print("\t");
                Serial.println(lidarData[3*i + 1] & 0x03);
              }
            }
            else
            {
              printFail(4, data_rcvd);
            }
          }
          else
          {
            printFail(3, data_rcvd);
          }
        }
        else
        {
          printFail(2, data_rcvd);
        }
      }
      else
      {
        printFail(1, data_rcvd);
      }
    }
    else
    {
      Serial.println("LiDAR_UART is unavailable!");
    }
//    doc["type"] = _laserscan;
//    doc["data"]["header"][0] = seqID;
//    doc["data"]["header"][1] = frame;
//    doc["data"]["header"][2] = num_lidar_points;
//    for (int i = 0; i < num_lidar_points; i++)
//    {
//      doc["data"]["intensity"][i] = (byte)lidarData[3*i];
//      doc["data"]["distance"][i] = (uint16_t)(lidarData[3*i + 2] << 6) + (lidarData[3*i + 1] >> 2);
//      doc["data"]["flag"][i] = (byte)(lidarData[3*i + 1] & 0x03);
//      doc["data"]["angles"][i] = lidarAngles[i];
//    }
//    char buff[buffSize];
//    size_t len = serializeJson(doc, buff);
//    webSocket.sendTXT(buff); // Send acknowledgement
  }
}

void loop()
{
  imu.read();
  webSocket.loop(); // Keep the socket alive
  static unsigned long startTime = millis();
  //  if (millis() - startTime < 2000)
  //  {
  //    drivetrain.drive(0.5, 0, 0);
  //  }
  //  else if (millis() - startTime < 4000)
  //  {
  //    drivetrain.drive(-0.5, 0, 0);
  //  }
  //  else if (millis() - startTime < 6000)
  //  {
  //    drivetrain.drive(0, 0.5, 0);
  //  }
  //  else if (millis() - startTime < 8000)
  //  {
  //    drivetrain.drive(0, -0.5, 0);
  //  }
  //  else if (millis() - startTime < 10000)
  //  {
  //    drivetrain.drive(0, 0, 0.5);
  //  }
  //  else if (millis() - startTime < 12000)
  //  {
  //    drivetrain.drive(0, 0, -0.5);
  //  }
  //  else
  //  {
  //    startTime = millis();
  //  }

  if (false)//millis() - startTime > 100)
  {
    Serial.print("\nA:\t");
    Serial.print(imu.a.x);
    Serial.print("\t");
    Serial.print(imu.a.y);
    Serial.print("\t");
    Serial.println(imu.a.z);

    Serial.print("\nG:\t");
    Serial.print(imu.g.x);
    Serial.print("\t");
    Serial.print(imu.g.y);
    Serial.print("\t");
    Serial.println(imu.g.z);

    Serial.print("\nM:\t");
    Serial.print(imu.m.x);
    Serial.print("\t");
    Serial.print(imu.m.y);
    Serial.print("\t");
    Serial.println(imu.m.z);

    Serial.print("\nAngles:\t");
    Serial.print(imu.angles.x);
    Serial.print("\t");
    Serial.print(imu.angles.y);
    Serial.print("\t");
    Serial.println(imu.angles.z);

    Serial.print("\nRAW A:\t");
    Serial.print(imu.imu.a.x);
    Serial.print("\t");
    Serial.print(imu.imu.a.y);
    Serial.print("\t");
    Serial.println(imu.imu.a.z);

    Serial.print("\nRAW G:\t");
    Serial.print(imu.imu.g.x);
    Serial.print("\t");
    Serial.print(imu.imu.g.y);
    Serial.print("\t");
    Serial.println(imu.imu.g.z);

    Serial.print("\nRAW M:\t");
    Serial.print(imu.mag.m.x);
    Serial.print("\t");
    Serial.print(imu.mag.m.y);
    Serial.print("\t");
    Serial.println(imu.mag.m.z);

    startTime = millis();
  }

  //  if (error == 1) {
  //    Serial.println("Error, no controller");
  //    return; // Skip if no controller
  //  }
  //  digitalWrite(LED_Status, HIGH);
  //  float* accel = getAccel();
  //  float* gyro = getGyro();

  //  updateAngles(angles, accelAngles, gyroAngles); // runs best with no delay
  //  ps2x.read_gamepad(); // Must be called frequently to read updated values

  //  static unsigned long currentTime, printTime;
  //  currentTime = millis();
  //
  //  if (currentTime - printTime >= 250) {
  ////    printIMU(accelAngles, gyroAngles, angles);
  //    printTime = currentTime;
  //  }

  //  float lx = 2 * (ps2x.Analog(PSS_LX) / 255.0) - 1; // Left stick x-axis
  //  float ly = -(2 * (ps2x.Analog(PSS_LY) / 255.0) - 1); // Left stick y-axis
  //  float rx = 2 * (ps2x.Analog(PSS_RX) / 255.0) - 1; // Right stick x-axis
  //  float ry = -(2 * (ps2x.Analog(PSS_RY) / 255.0) - 1); // Right stick y-axis
  // Above values are adjusted to be between -1.0 and 1.0

  // Use of the D-Pad on the PS2 controller will override the left stick.
  // This is to test with digital power (max on or off), while the stick allows for analog power.
  //  float pad_x = 0.0;
  //  float pad_y = 0.0;
  //  if (ps2x.Button(PSB_PAD_UP) || ps2x.Button(PSB_PAD_DOWN)) {
  //    pad_y = ps2x.Button(PSB_PAD_UP) ? 1.0 : -1.0;
  //  } else if (ps2x.Button(PSB_PAD_RIGHT) || ps2x.Button(PSB_PAD_LEFT)) {
  //    pad_x = ps2x.Button(PSB_PAD_RIGHT) ? 1.0 : -1.0;
  //  }

  // Reset Gyro Z Angle
  //  if (ps2x.Button(PSB_SELECT)) {
  //    gyroAngles[2] = 0;
  //  }

  //  float turn = 0;
  //  if (ps2x.Button(PSB_R1)) {
  //    if (abs(rx) > 0.2 || abs(ry) > 0.2) {
  //      turn = getTurnCmd(getAngleFromJoystick(rx, ry));
  //      turn = map(turn*10, -150, 150, -100, 100) / 100.0;
  //      turn = constrain(turn, -1, 1);
  //    }
  //  } else {
  //    turn = rx;
  //  }

  //  float a = getAngleFromJoystick(rx, ry);
  //  Serial.print("Des: ");
  //  Serial.print(a);
  //  Serial.print("\tCur: ");
  //  Serial.print(angles[2]);
  //  Serial.print("\tTurn: ");
  //  Serial.println(turn);

  // Controls the drivetrain based on either the D-Pad or the left stick.
  // Right stick still works for turning in either case.
  // drivetrain.drive(float x, float y, float z)
  //  if (pad_x || pad_y) {
  //      drivetrain.drive(pad_y, pad_x, turn); // x and y intentionally swapped
  //    } else {
  //      drivetrain.drive(ly, lx, turn); // x and y intentionally swapped
  //    }

  // For debugging purposes via the Serial Monitor:
  //  printCommandVelocities();
  //  printRawSpeeds();
  //  printObservedSpeeds();

  //  delay(50);

  // END OF LOOP --------------------------------------------------------------------------
}

void webSocketEvent(WStype_t type, uint8_t * payload, size_t length)
{
  Serial.print("Websocket Event Triggered. ");
  Serial.print("type: ");
  Serial.println(WStypes[type]);
  if (type == WStype_TEXT)
  {
    // Deserialize incoming JSON String
    DeserializationError error = deserializeJson(doc, payload);
    if (error) // Print error msg if incoming String is not JSON formated
    {
      Serial.print(F("deserializeJson() failed: "));
      Serial.println(error.c_str());
      return;
    }
    MessageType messageType = doc["type"];
    String _data = doc["data"];
    Serial.println(_data);
    if (messageType == _geometry)
    {
      float x = doc["data"]["linear"]["x"], y = doc["data"]["linear"]["y"], z = doc["data"]["angular"]["z"];
      drivetrain.drive(x, y, z);
    }
//    setLidarData();
//    char buff[buffSize];
//    size_t len = serializeJson(doc, buff);
//    Serial.println(len);
//    webSocket.sendTXT(buff); // Send acknowledgement
  }
  else if (type == WStype_DISCONNECTED)
  {
    drivetrain.drive(0,0,0); // should make a stop function in Mecanum class
  }
}

//void setLidarData()
//{
//  static uint32_t seqID = 0;
//  static INT32UNION_t seqIDUnion;
//  seqIDUnion.number = seqID++;
//  FLOATUNION_t floatUnion;
//  INT16UNION_t dataSizeUnion;
//  INT16UNION_t headerSizeUnion;
//  floatUnion.number = 4.5;
//  dataSizeUnion.number = dataSize;
//  headerSizeUnion.number = headerSize;
//
//  for (int i = 0; i < dataSize; i++)
//  {
//    if (i < 2) // Data Size
//    {
//      lidarData[i] = dataSizeUnion.bytes[i];
//    }
//    else if (i < 4) // Header Size
//    {
//      lidarData[i] = headerSizeUnion.bytes[i - 2];
//    }
//    else if (i < 8) // Sequence ID (part of header)
//    {
//      lidarData[i] = seqIDUnion.bytes[i - 4];
//    }
//    else if (i < 4 + headerSize) // Frame ID (part of header)
//    {
//      lidarData[i] = frame[i - 8];
//    }
//    else
//    {
//      lidarData[i] = floatUnion.bytes[(i - 4 - headerSize) % 4];
//    }
//  }
//  doc["type"] = _laserscan;
//  doc["data"][0] = seqID;
//  doc["data"][1]= frame;
//  doc["data"][2] = numDataPoints;
//  for (int i = 0; i < numDataPoints; i++)
//  {
//    for (int j = 0; j < 4; j++)
//    {
//      floatUnion.bytes[j] = lidarData[4*i + j + 4 + headerSize];
//    }
//    doc["data"][i + 3] = floatUnion.number;
//  }
//  Serial.println("setLidarData Finished.");
//}

float getAngleFromJoystick(float x, float y)
{
  float angle = -atan(x / y) * (180 / PI);
  if (y < 0)
  {
    if (x < 0)
    {
      angle += 180;
    }
    else
    {
      angle -= 180;
    }
  }
  return angle;
}

//float getTurnCmd(float desiredAngle) {
//  float error = angles[2] - desiredAngle;
//  if (error < -180) {
//    error += 360;
//  } else if (error > 180) {
//    error -= 360;
//  }
//  return error;
//}

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

//void updateAngles(float* angles, float* accelAngles, float* gyroAngles) {
//  updateAccelAngles(accelAngles);
//  updateGyroAngles(gyroAngles);
//  float alpha = 0.9;
//  for (int i = 0; i < 3; i++) {
//    // Complementary filter to combine accelerometer and gyroscope angle values
//    if (i < 2) { // Accelerometer readings is only X and Y
//      if (abs(accelAngles[i]) < 0.5) gyroAngles[i] = 0; // reset gyro if accelerometer reads close to 0 degrees
//      angles[i] = alpha*gyroAngles[i] + (1-alpha)*accelAngles[i];
//    } else {
//      angles[i] = gyroAngles[i];
//    }
//  }
//}

//void updateAccelAngles(float* accelAngles) {
//  float* accel = getAccel();
//  accelAngles[0] = acos(accel[2] / sqrt(accel[1]*accel[1] + accel[2]*accel[2])) * (180 / PI) * (accel[1] < 0 ? -1 : 1);
//  accelAngles[1] = acos(accel[2] / sqrt(accel[0]*accel[0] + accel[2]*accel[2])) * (180 / PI) * (accel[0] > 0 ? -1 : 1);
//  for (int i = 0; i < 2; i++) {
//    accelAngles[i] = contScaleToBounded(accelAngles[i]);
//  }
//}

//void updateGyroAngles(float* gyroAngles) {
//  float* gyro = getGyro();
//  static unsigned long previousTime, currentTime, elapsedTime;
//  previousTime = currentTime;
//  currentTime = millis();
//  elapsedTime = currentTime - previousTime;
//  for (int i = 0; i < 3; i++) {
//    gyroAngles[i] += (gyro[i] - gyroErrors[i]) * (elapsedTime / 1000.0); // divide by 1000 to convert to seconds
//    gyroAngles[i] = contScaleToBounded(gyroAngles[i]);
//  }
//}

// Converts a number from a continuous scale to a scale bounded from -180 to 180
float contScaleToBounded(float x)
{
  while (x > 180)
    x -= 360;
  while (x < -180)
    x += 360;
  return x;
}

// returns {x,y,z} acceleration in m/s^2
//float* getAccel() {
//  return getAccel(0);
//}

// returns {x,y,z} acceleration in m/s^2
//float* getAccel(int cal) {
//  static float accel[3];
//  mpu.read_acc();
//  scales sensorScale = scale_2g; // scale_2g is default. Can also be 4g, 8g, or 16g.
//  accel[0] = processAccel(mpu.ax, sensorScale);
//  accel[1] = processAccel(mpu.ay, sensorScale);
//  accel[2] = processAccel(mpu.az, sensorScale);
//  if (!cal) {
//    for (int i = 0; i < 3; i++) {
//      accel[i] -= accelErrors[i];
//    }
//  }
//  return accel;
//}

// returns {x,y,z} angular velocity in degrees/sec
//float* getGyro() {
//  static float gyro[3];
//  mpu.read_gyro();
//  scales sensorScale = scale_250dps; // scale_250dps is default. Can also be 500, 1000, or 2000.
//  gyro[0] = processAngVel(mpu.gx, sensorScale);
//  gyro[1] = processAngVel(mpu.gy, sensorScale);
//  gyro[2] = processAngVel(mpu.gz, sensorScale);
//  return gyro;
//}

// input = raw reading, sensorScale = selected sensor scale
// returns: acceleration in m/s^2
// example call: processAccel(mpu.ax, scale_2g);
//float processAccel(int input, scales sensorScale) {
//  float output = input;
//  switch(sensorScale) {
//    case scale_2g: // default
//      output /= 16384;
//      break;
//    case scale_4g:
//      output /= 8192;
//      break;
//    case scale_8g:
//      output /= 4096;
//      break;
//    case scale_16g:
//      output /= 2048;
//      break;
//    default:
//      output = 0;
//  }
//  return output * g;
//}

// input = raw reading, sensorScale = selected sensor scale
// returns: angular velocity in degrees per second
// example call: processAngVel(mpu.gx, scale_250dps);
//float processAngVel(int16_t input, scales sensorScale) {
//  switch(sensorScale) {
//    case scale_250dps: // default
//      return input / 131;
//    case scale_500dps:
//      return input / 65.5;
//    case scale_1000dps:
//      return input / 32.8;
//    case scale_2000dps:
//      return input / 16.4;
//  }
//  return 0;
//}

//void printIMU(float* accelAngles, float* gyroAngles, float* angles) {
//  Serial.println();
//
//  Serial.print("A:\t");
//  Serial.print(accelAngles[0]);
//  Serial.print("\t");
//  Serial.println(accelAngles[1]);
////  Serial.print("\t");
////  Serial.println(accel[2]);
//
//  Serial.print("G:\t");
//  Serial.print(gyroAngles[0]);
//  Serial.print("\t");
//  Serial.print(gyroAngles[1]);
//  Serial.print("\t");
//  Serial.println(gyroAngles[2]);
//
//  Serial.print("Angles:\t");
//  Serial.print(angles[0]);
//  Serial.print("\t");
//  Serial.print(angles[1]);
//  Serial.print("\t");
//  Serial.println(angles[2]);
//}

//void calculateIMUError() {
//  float* accel;
//  float* gyro;
//  const int iterations = 250*5;
//
//  Serial.println("Calibration is about to start.");
//  Serial.println("Make sure IMU is stationary with Z-axis parallel to gravity.");
//  delay(3000);
//  Serial.print("Calibration starting in ");
//  for (int i = 3; i > 0; i--) {
//    Serial.print(i);
//    Serial.print("... ");
//    delay(1000);
//  }
//  Serial.println();
//  for (int i = 0; i < 3; i++) {
//    gyroAngles[i] = 0;
//  }
//  for (int i = 0; i < iterations; i++) {
//    accel = getAccel(1);
//    gyro = getGyro();
//
//    for (int j = 0; j < 3; j++) {
//      gyroErrors[j] += gyro[j];
//      accelErrors[j] += accel[j];
//      if (j == 2)
//        accelErrors[j] -= g; // Z acceleration should be experiencing 1g
//    }
//    delay(4);
//  }
//  for (int i = 0; i < 3; i++) {
//    accelErrors[i] /= iterations;
//    gyroErrors[i] /= iterations;
//  }
//  Serial.println("Calibration finished.");
//  Serial.print("accelErrorX: ");
//  Serial.println(accelErrors[0]);
//  Serial.print("accelErrorY: ");
//  Serial.println(accelErrors[1]);
//  Serial.print("accelErrorZ: ");
//  Serial.println(accelErrors[2]);
//  Serial.print("gyroErrorX: ");
//  Serial.println(gyroErrors[0]);
//  Serial.print("gyroErrorY: ");
//  Serial.println(gyroErrors[1]);
//  Serial.print("gyroErrorZ: ");
//  Serial.println(gyroErrors[2]);
//  // Small delay to ensure errors remain visible for a short time
//  delay(1500);
//}
