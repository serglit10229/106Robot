#include <Wire.h>
#include <LIS3MDL.h>
#include <LSM6.h>
#include <SimpleTimer.h>
#include <EEPROM.h>
#include <BasicEncoder.h>
#include <TimerOne.h>
#include <ServoTimer2.h>

// Calibration mode
int buttonPin = 3;
bool Recalibrate = false;

// Compass settings
float heading = 0.0;
const float alpha = 0.0;

// Servo settings
const int servoPin = 9;
int targetHeading = 320; //320 ,160

// Solenoid settings
int solenoidPin = 2;
float crankAngle = 0.0f;
float solenoidAdvance = 0.0f;
float currentRPS = 0.0f;
float currentSpeed = 0.0f;

unsigned int externalTimerCounter = 1; 

#pragma region Compass_Internal
  LSM6 imu;
  LIS3MDL mag;
  LIS3MDL::vector<int16_t> mag_min = {32767, 32767, 32767};
  LIS3MDL::vector<int16_t> mag_max = {-32768, -32768, -32768};
  int gyro_offset = 0;
  const int compassUpdateFrequency = 100; //ms
  const int gyroSampleFrequency = 2; //ms 
  long tempGyroSum = 0;
  int gyroSampleCounter = 0;
  const int CalibrationDuration = 10000;
#pragma endregion

#pragma region Servo_Internal
  ServoTimer2 servo;
  SimpleTimer steerDelayTimer;
  SimpleTimer bonusDelayTimer;
  const int servoInitOffset = 1500;
  int servoLimit = 200;
  int currentServoAngle = servoInitOffset;
  float kp = 0.03f;
  float kd = 0.0000f;
  float prevError = 0.0f;
  const int servoUpdateInterval = 10; //ms
#pragma endregion

#pragma region Solenoid_Internal
  SimpleTimer driveTimer;
  bool solenoidActive = false;
  BasicEncoder encoder(11, 12);
  float encoderResolution = 20;
  int prevEncoderPos = 0;
  unsigned long prevEncoderTime; 
  float wheelRadius = 0.065f;
#pragma endregion

void setup() {
  Serial.begin(9600);
  Wire.begin();
  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(LED_BUILTIN, OUTPUT);

  Recalibrate = !digitalRead(buttonPin);

  ServoInit();
  SolenoidInit();
  CompassInit();

  delay(11000);

  prevEncoderTime = millis();
  DriveForSeconds(26.0f, 23.0f); //26,23
  SteerAfterSeconds(3.8f); //4.2

  Timer1.initialize(1000);
  Timer1.attachInterrupt(ExternalTimer);
}

void ExternalTimer() {
  encoder.service();

  if((externalTimerCounter % servoUpdateInterval) == 0)
  {
    if(!driveTimer.isReady())
    {
      SteerToHeading(targetHeading);
    }
  }

  if((externalTimerCounter % gyroSampleFrequency) == 0)
  {
    ReadGyro();
  }

  if((externalTimerCounter % compassUpdateFrequency) == 0)
  {
    ReadCompass();
  }

  externalTimerCounter++;
}

void loop() {
  mag.read();
  imu.readGyro();
  ReadEncoder();
  
  if(steerDelayTimer.isReady()) {
    targetHeading = 40;
    if(bonusDelayTimer.isReady())
      targetHeading = 100;
  }

  if(!driveTimer.isReady())
  {
    if(fmod(crankAngle+solenoidAdvance, 360.0) < (180.0f - 25.0f))
    {
      digitalWrite(solenoidPin,HIGH);
      solenoidActive = true;
    }
    else
    {
      digitalWrite(solenoidPin,LOW);
      solenoidActive = false;
    }
  }
  else
  {
    SteerToValue(0.0f);
    digitalWrite(solenoidPin,LOW);
    solenoidActive = false;
  }
  Serial.println(String(heading) + "   " + String(currentServoAngle) + "   " + String(crankAngle));
}

void ServoInit() {
  servo.attach(servoPin);
  SteerToValue(0.0f);
  delay(500);
  SteerToValue(1.0f);
  delay(500);
  SteerToValue(-1.0f);
  delay(500);
  SteerToValue(0.0f);
}

void SolenoidInit() {
  pinMode(solenoidPin, OUTPUT);
  digitalWrite(solenoidPin, LOW);
  // delay(1000);
  // digitalWrite(solenoidPin, HIGH);
  // delay(1000);
  // digitalWrite(solenoidPin, LOW);
}

void CompassInit() {
  // Compass init
  if (!mag.init() || !imu.init())
  {
    Serial.println("Failed to find compass sensor.");
    while (true)
    {
      digitalWrite(LED_BUILTIN, HIGH);
      delay(250);
      digitalWrite(LED_BUILTIN, LOW);
      delay(250);
    }
  }
  mag.enableDefault();
  imu.enableDefault();
  digitalWrite(LED_BUILTIN, HIGH);
  CalibrateMagnetometer();
  CalibrateGyroscope();
  digitalWrite(LED_BUILTIN, LOW);
  ReadCompass();
}

void DriveForSeconds(float seconds, float bonusStartSeconds) {
  driveTimer.reset();
  driveTimer.setInterval(1000 * seconds);
  bonusDelayTimer.reset();
  bonusDelayTimer.setInterval(1000 * bonusStartSeconds);
}

void SteerAfterSeconds(float seconds) {
  steerDelayTimer.reset();
  steerDelayTimer.setInterval(1000 * seconds);
}

void SteerToValue(float setValue) {
  currentServoAngle = servoInitOffset + constrain(-setValue * servoLimit, -servoLimit, servoLimit);
  servo.write(currentServoAngle);
}

void SteerToHeading(int setHeading) {
  //PD controller
  float error = (setHeading - heading);
  if(abs(setHeading - heading) > 179)
  {
    error = 360 - abs(setHeading - heading);
  }

  //float error = (setHeading - heading);
  float output = (kp * error) + kd * ((error - prevError) / (servoUpdateInterval / 1000.0f));

  //Serial.println("heading: " + String(heading) + " error: " + String(error));  

  prevError = error;
  SteerToValue(-output);
}

void ReadEncoder() {
  int tempEncoderPos = -(encoder.get_count());
  //Serial.println(tempEncoderPos);
  unsigned long tempTime = millis();
  crankAngle = (tempEncoderPos / encoderResolution) * 360.0f;

  int deltaEncoderPos = tempEncoderPos - prevEncoderPos;
  int deltaEncoderTime = tempTime - prevEncoderTime;
  if(deltaEncoderTime == 0) return;
  
  currentRPS = (deltaEncoderPos / encoderResolution) / (deltaEncoderTime / 1000);
  currentSpeed = wheelRadius * ((deltaEncoderPos / encoderResolution) * 6.2832f) / (deltaEncoderTime / 1000);
  if(encoder.get_change())
  {
    prevEncoderPos = tempEncoderPos;
    prevEncoderTime = tempTime;
  }
}

void ReadCompass() {
  float mx = Mapfloat(mag.m.x, mag_min.x, mag_max.x, -1.0f, 1.0f);
  float my = Mapfloat(mag.m.y, mag_min.y, mag_max.y, -1.0f, 1.0f);

  int gz = 0;
  if(gyroSampleCounter > 0)
  {
    gz = tempGyroSum / gyroSampleCounter;
    if(abs(gz) < 200)
      gz = 0;
    tempGyroSum = 0;
    gyroSampleCounter = 0;
  }
  SensorFusionUpdate(gz, mx, my, compassUpdateFrequency/1000.0f);
}

void ReadGyro() {
  tempGyroSum += (imu.g.z - gyro_offset);
  gyroSampleCounter++;
}

void SensorFusionUpdate(float gz, float mx, float my, float dt) {
  float magAngle = atan2(my, mx) * (180 / M_PI);
  heading = alpha * (heading + (-0.01 * gz * dt)) + (1 - alpha) * magAngle;
  //Serial.println("gz: " + String((-0.01 * gz * dt)) + " heading: " + String(heading));

  heading = fmod(heading, 360.0);
  if(heading < 0) 
    heading += 360;
}

void CalibrateMagnetometer() {
  if (Recalibrate)
  {
    SimpleTimer calibrationTimer(CalibrationDuration);
    Serial.println("Calibrating magnetometer: " + String(CalibrationDuration/1000) + " seconds. Rotate around Z.");
    while(!calibrationTimer.isReady())
    {

      mag.read();
      mag_min.x = min(mag_min.x, mag.m.x);
      mag_min.y = min(mag_min.y, mag.m.y);
      mag_min.z = min(mag_min.z, mag.m.z);
      mag_max.x = max(mag_max.x, mag.m.x);
      mag_max.y = max(mag_max.y, mag.m.y);
      mag_max.z = max(mag_max.z, mag.m.z);

      delay(10);
    }
    Serial.println("Magnetometer calibrated");
    EEPROM.put(0, mag_min);
    EEPROM.put(sizeof(LIS3MDL::vector<int16_t>), mag_max);
  }
  else
  {
    EEPROM.get(0, mag_min);
    EEPROM.get(sizeof(LIS3MDL::vector<int16_t>), mag_max);
    Serial.println(String(mag_min.z) + " " + String(mag_max.z));
    delay(1000);
  }
}

void CalibrateGyroscope() {
  if (Recalibrate)
  {
    SimpleTimer calibrationTimer(CalibrationDuration/2);
    Serial.println("Calibrating gyroscope: " + String(CalibrationDuration/2000) + " seconds. Hold still.");
    long tempSum = 0;
    int i = 0;
    while(!calibrationTimer.isReady())
    {

      imu.readGyro();
      tempSum += imu.g.z;
      i++;

      delay(2);
    }
    gyro_offset = tempSum/i;
    Serial.println("Magnetometer calibrated");
    EEPROM.put(2 * sizeof(LIS3MDL::vector<int16_t>), gyro_offset);
  }
  else
  {
    EEPROM.get(2 * sizeof(LIS3MDL::vector<int16_t>), gyro_offset);
    Serial.println(gyro_offset);
    delay(1000);
  }
}

float Mapfloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float GetOdometer() {
  return wheelRadius * (crankAngle / 6.28318531);
}

int DegToMicroseconds(int deg) {
  return (deg * 5.55555555556f) + 1000;
}