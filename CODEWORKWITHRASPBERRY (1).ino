#include <Wire.h>
#include <Servo.h>
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>

const char *ssid = "Drone";
const char *password = "nevergiveup";
const IPAddress remoteIP(192, 168, 1, 100); // Change to the IP address of the remote receiver
const int remotePort = 8888;                // Change to the desired remote port

WiFiUDP udp;

Servo myservo1; /* Motors numbering:             4  1   */
Servo myservo2; /*                                \ /    */
Servo myservo3; /*                                / \    */
Servo myservo4; /*                               3   2   */

//=================== PID Rate Controller variables===================
float PRateRoll = 0.6;
float PRatePitch = 9;
float PRateYaw = 0;
float IRateRoll = 3.5;
float IRatePitch = 3.5;
float IRateYaw = 0;
float DRateRoll = 0.03;
float DRatePitch = 5;
float DRateYaw = 0;
float Rr_pid_i, Rp_pid_i, Ry_pid_i;
float Rr_pid_p;
float Rp_pid_p;
float Ry_pid_p;
float Rp_pid;
float Rr_pid;
float Ry_pid;
float ErrorRateRoll;
float ErrorRatePitch;
float ErrorRateYaw;
float PrevErrorRateRoll, PrevErrorRatePitch, PrevErrorRateYaw;
//=============================================================
float InputRoll;
float InputPitch;
float InputYaw;
float w1, w2, w3, w4;
//=========== PID angle controller=============================
float r_pid, r_error, pre_rerror; //
float r_pid_p = 0;                //
float r_pid_i = 0;                // ROLL
float r_pid_d = 0;
double r_kp = 0;
double r_ki = 0;
double r_kd = 0;
float dr = 0;
float p_pid, p_error, pre_perror;
float p_pid_p = 0;
float p_pid_i = 0; // PITCH
float p_pid_d = 0;
double p_kp = 2;
double p_ki = 0;
double p_kd = 0;
float dp = 0;
float y_pid, y_error, pre_yerror;
float y_pid_p = 0;
float y_pid_i = 0;
float y_pid_d = 0; // YAW
double y_kp = 0;
double y_ki = 0;
double y_kd = 0;
float dy = 0;
int uz = 1100;

float RateCalibrationNumber, RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYaw;

float RateRoll, RatePitch, RateYaw;
float AccX, AccY, AccZ;
float AngleRoll, AnglePitch, AnglaYaw;
float GyroAngleRoll, GyroAnglePitch, GyroAngleYaw;
float CompAngleRoll, CompAnglePitch;
float LoopTimer;
unsigned long timer;
unsigned long timer1, timer2;
const float alpha = 0.8; // Complementary filter coefficient

void gyro_signals(void)
{
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 6);
  int16_t AccXLSB = Wire.read() << 8 | Wire.read();
  int16_t AccYLSB = Wire.read() << 8 | Wire.read();
  int16_t AccZLSB = Wire.read() << 8 | Wire.read();
  Wire.beginTransmission(0x68);
  Wire.write(0x1B);
  Wire.write(0x8);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 6);
  int16_t GyroX = Wire.read() << 8 | Wire.read();
  int16_t GyroY = Wire.read() << 8 | Wire.read();
  int16_t GyroZ = Wire.read() << 8 | Wire.read();
  RateRoll = (float)GyroX / 65.5;
  RatePitch = (float)GyroY / 65.5;
  RateYaw = (float)GyroZ / 65.5;
  AccX = (float)AccXLSB / 4095.98;
  AccY = (float)AccYLSB / 4096.02;
  AccZ = (float)AccZLSB / 4095.88;

  // Calculate roll angle from accelerometer data
  AngleRoll = atan(AccY / sqrt(AccX * AccX + AccZ * AccZ)) / 0.017455;
  AnglePitch = -atan(AccX / sqrt(AccY * AccY + AccZ * AccZ)) / 0.017455;

  // Integrate gyro data for roll angle
  float dt = (millis() - timer) / 1000.0; // time since last loop
  timer = millis();

  GyroAngleRoll += RateRoll * dt;
  GyroAnglePitch += RatePitch * dt;
  GyroAngleYaw += RateYaw * dt;

  // Apply complementary filter to combine accelerometer and gyro data
  CompAngleRoll = 0.8 * (CompAngleRoll + RateRoll * dt) + 0.2 * AngleRoll;
  CompAnglePitch = 0.8 * (CompAnglePitch + RatePitch * dt) + 0.2 * AnglePitch;
}

void handlePing()
{
  server.send(200, "text/plain", "Pong"); // Respond with "Pong" to indicate successful ping
}

void handleUz()
{
  if (server.hasArg("uz"))
  {
    uz = server.arg("uz").toInt();
    Serial.print("Uz changed to ");
    Serial.println(uz);
  }
  server.send(200, "text/plain", String(uz));
}
void handlePRatePitch()
{
  if (server.hasArg("PRatePitch"))
  {
    PRatePitch = server.arg("PRatePitch").toInt();
    Serial.print("PRatePitch changed to ");
    Serial.println(PRatePitch);
  }
  server.send(200, "text/plain", String(PRatePitch));
}
void handleIRatePitch()
{
  if (server.hasArg("IRatePitch"))
  {
    IRatePitch = server.arg("IRatePitch").toInt();
    Serial.print("IRatePitch changed to ");
    Serial.println(IRatePitch);
  }
  server.send(200, "text/plain", String(IRatePitch));
}
void handleDRatePitch()
{
  if (server.hasArg("DRatePitch"))
  {
    DRatePitch = server.arg("DRatePitch").toInt();
    Serial.print("DRatePitch changed to ");
    Serial.println(DRatePitch);
  }
  server.send(200, "text/plain", String(DRatePitch));
}

void handlep_kp()
{
  if (server.hasArg("p_kp"))
  {
    p_kp = server.arg("p_kp").toInt();
    Serial.print("p_kp changed to ");
    Serial.println(p_kp);
  }
  server.send(200, "text/plain", String(p_kp));
}

void setSpeed(int w1, int w2, int w3, int w4)
{
  if (w1 > 2000)
    w1 = 1999;
  if (w2 > 2000)
    w2 = 1999;
  if (w3 > 2000)
    w3 = 1999;
  if (w4 > 2000)
    w4 = 1999;
  int ThrottleIdle = 1100;
  if (w1 < ThrottleIdle)
    w1 = ThrottleIdle;
  if (w2 < ThrottleIdle)
    w2 = ThrottleIdle;
  if (w3 < ThrottleIdle)
    w3 = ThrottleIdle;
  if (w4 < ThrottleIdle)
    w4 = ThrottleIdle;

  myservo1.writeMicroseconds(w1);
  myservo2.writeMicroseconds(w2);
  myservo3.writeMicroseconds(w3);
  myservo4.writeMicroseconds(w4);
}

void calibrateESCs()
{
  // Send a MIN signal to all ESCs to calibrate them
  myservo1.writeMicroseconds(1000);
  myservo2.writeMicroseconds(1000);
  myservo3.writeMicroseconds(1000);
  myservo4.writeMicroseconds(1000);

  // Wait for 1 seconds for ESCs to be calibrated
  delay(1000);
}

void sendControlUDP()
{
  String message = String(w1) + "," + String(w2) + "," + String(w3) + "," + String(w4);
  udp.beginPacket(remoteIP, remotePort);
  udp.write(message.c_str(), message.length());
  udp.endPacket();
}

void setup()
{

  Serial.begin(57600);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  server.on("/ping", HTTP_GET, handlePing); // Handle the ping request
  server.on("/uz", handleUz);
  server.on("/PRatePitch", handlePRatePitch);
  server.on("/IRatePitch", handleIRatePitch);
  server.on("/DRatePitch", handleDRatePitch);
  server.on("/p_kp", handlep_kp);

  server.begin();
  Serial.println("Web server started");

  myservo1.attach(16);
  myservo2.attach(17);
  myservo3.attach(18);
  myservo4.attach(19); // Attach motors to pins 16-19

  calibrateESCs();
  delay(2000);
  //   setSpeed(0,0,0,0);//
  Wire.setClock(400000);
  Wire.begin();
  delay(500);
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
  timer = millis();
  for (RateCalibrationNumber = 0;
       RateCalibrationNumber < 2000; // FOR loop to sum the avreage of 2000 Measuremnts
       RateCalibrationNumber++)
  {
    gyro_signals();
    RateCalibrationRoll += RateRoll;
    RateCalibrationPitch += RatePitch;
    RateCalibrationYaw += RateYaw;
    delay(1);
  }

  RateCalibrationRoll /= 2000;
  RateCalibrationPitch /= 2000; // get the avreage value of the
  RateCalibrationYaw /= 2000;

  LoopTimer = micros();
}
void loop()
{

  gyro_signals();

  RateRoll -= RateCalibrationRoll;
  RatePitch -= RateCalibrationPitch;
  RateYaw -= RateCalibrationYaw;

  float dt1 = (millis() - timer1) / 1000.0; // time since last loop
  timer1 = millis();
  r_error = dr - CompAngleRoll;
  p_error = dp - CompAnglePitch;
  y_error = dy - GyroAngleYaw;

  r_pid_p = r_kp * r_error;
  p_pid_p = p_kp * p_error;
  y_pid_p = y_kp * y_error;

  r_pid_d = r_kd * ((r_error - pre_rerror) / dt1);
  p_pid_d = p_kd * ((p_error - pre_perror) / dt1);
  y_pid_d = y_kd * ((y_error - pre_yerror) / dt1);

  r_pid = r_pid_p + r_pid_d;
  p_pid = p_pid_p + p_pid_d;
  y_pid = y_pid_p + y_pid_d;

  if (p_pid < -400)
  {
    p_pid = -400;
  }
  if (p_pid > 400)
  {
    p_pid = 400;
  }
  if (y_pid < -400)
  {
    y_pid = -400;
  }
  if (y_pid > 400)
  {
    y_pid = 400;
  }
  if (r_pid < -400)
  {
    r_pid = -400;
  }
  if (r_pid > 400)
  {
    r_pid = 400;
  }

  float ErrorRateRoll = r_pid - RateRoll;
  float ErrorRatePitch = p_pid - RatePitch;
  float ErrorRateYaw = y_pid - RateYaw;

  Rr_pid_p = PRateRoll * ErrorRateRoll;
  Rp_pid_p = PRatePitch * ErrorRatePitch;
  Ry_pid_p = PRateYaw * ErrorRateYaw;

  float Rr_pid_d = DRateRoll * ((ErrorRateRoll - PrevErrorRateRoll) / dt1);
  float Rp_pid_d = DRatePitch * ((ErrorRatePitch - PrevErrorRatePitch) / dt1);
  float Ry_pid_d = DRateYaw * ((ErrorRateYaw - PrevErrorRateYaw) / dt1);

  Rr_pid_i = Rr_pid_i + (IRateRoll * ErrorRateRoll);
  Rp_pid_i = Rp_pid_i + (IRateRoll * ErrorRatePitch);
  Ry_pid_i = Ry_pid_i + (IRateRoll * ErrorRateYaw);

  InputRoll = Rr_pid_p + Rr_pid_d + Rr_pid_i;
  InputPitch = Rp_pid_p + Rp_pid_d + Rp_pid_i;
  InputYaw = Ry_pid_p + Ry_pid_d + Ry_pid_i;

  if (InputPitch < -400)
  {
    InputPitch = -400;
  }
  if (InputPitch > 400)
  {
    InputPitch = 400;
  }
  if (InputYaw < -400)
  {
    InputYaw = -400;
  }
  if (InputYaw > 400)
  {
    InputYaw = 400;
  }
  if (InputRoll < -400)
  {
    InputRoll = -400;
  }
  if (InputRoll > 400)
  {
    InputRoll = 400;
  }

  float up = InputPitch;
  float ur = InputRoll;
  float uy = InputYaw;

  w1 = uz - up - ur - uy;
  w2 = uz + up - ur + uy;
  w3 = uz + up + ur - uy;
  w4 = uz - up + ur + uy;

  pre_rerror = r_error;
  pre_perror = p_error;

  float PrevErrorRateRoll = ErrorRateRoll;
  float PrevErrorRatePitch = ErrorRatePitch;

  setSpeed(w1, w2, w3, w4);
  while (micros() - LoopTimer < 2000)
    ;
  LoopTimer = micros();

  sendControlUDP();

  Serial.print("CompAngleRoll = ");
  Serial.print(CompAngleRoll);
  Serial.print(", ");
  Serial.print("CompAnglePitch = ");
  Serial.print(CompAnglePitch);
  // Serial.print(LoopTimer);
  // Serial.print(", ");
  // Serial.print("w1 = ");
  // Serial.print (w1);
  // Serial.print(", ");
  // Serial.print("W2 = ");
  // Serial.print (w2);
  // Serial.print(", ");
  // Serial.print("w3 = ");
  // Serial.print (w3);
  // Serial.print(", ");
  // Serial.print("w4 = ");
  // Serial.print (w4)
}
