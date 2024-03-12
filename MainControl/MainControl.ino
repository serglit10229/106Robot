#include <Wire.h>
#include <LIS3MDL.h>
#include <LSM6.h>
#include <SimpleTimer.h>
#include <EEPROM.h>
#include <Servo.h>

// Calibration mode
bool Recalibrate = false;
int buttonPin = 12;

// Compass settings
const int CompassSampleFrequency = 10;
int gyroscopeSampleMultiplier = 50;
float heading = 0.0;
const float alpha = 0.1;

// Servo settings
Servo servo;
const int servoPin = 9;
int servoLimit = 25;
int targetHeading = 0;

// Solenoid settings
int solenoidPin = 2;
int timingSwitchPin = 7;

#pragma region Compass_Internal
  LSM6 imu;
  LIS3MDL mag;
  float invSampleFreq;
  LIS3MDL::vector<int16_t> mag_min = {32767, 32767, 32767};
  LIS3MDL::vector<int16_t> mag_max = {-32768, -32768, -32768};
  int gyro_offset = 0;
  SimpleTimer CompassUpdateTimer;
  const int CalibrationDuration = 10000;
#pragma endregion

#pragma region Servo_Internal
  const int servoInitOffset = 68;
  int currentServoAngle = servoInitOffset;
  float kp = 0.05f;
  float kd = 0.0000f;
  float prevError = 0.0f;
  const float servoUpdateInterval = 0.001f;
  SimpleTimer ServoUpdateTimer;
#pragma endregion

#pragma region Solenoid_Internal
  SimpleTimer driveTimer;
  SimpleTimer tempTimer(3);
  bool prevTimingPos = true;
  int timingCounter = 0;
  bool solenoidActive = false;
#pragma endregion

void setup() {
  Serial.begin(9600);
  Wire.begin();
  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(LED_BUILTIN, OUTPUT);

  Recalibrate = !digitalRead(buttonPin);

  // Servo init
  servo.attach(servoPin);
  servo.write(servoInitOffset); // (+/-25)

  // Solenoid init
  pinMode(solenoidPin, OUTPUT);
  pinMode(timingSwitchPin, INPUT_PULLUP);
  digitalWrite(solenoidPin, LOW);
  Serial.println("hello");
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
  Serial.println("hello2");
  digitalWrite(LED_BUILTIN, HIGH);
  CalibrateMagnetometer();
  CalibrateGyroscope();
  ReadCompass();
  targetHeading = heading;
  digitalWrite(LED_BUILTIN, LOW);

  invSampleFreq = (1.0f / CompassSampleFrequency); 
  CompassUpdateTimer.setInterval(1000 * invSampleFreq);
  ServoUpdateTimer.setInterval(1000 * servoUpdateInterval);
  DriveForSeconds(100.0f);
}

void loop() {
  if(CompassUpdateTimer.isReady())
  {
    ReadCompass();
    CompassUpdateTimer.reset();
  }

  if(ServoUpdateTimer.isReady())
  {
    SteerToHeading(targetHeading);
    ServoUpdateTimer.reset();
  }

  // if(tempTimer.isReady()) 
  // {
  //   bool tempPos = digitalRead(timingSwitchPin);
  //   if(prevTimingPos == false && tempPos == true)
  //   {
  //     timingCounter += 1;
  //     //Serial.println(timingCounter);
  //   }
  //   prevTimingPos = tempPos;

  //   if((timingCounter % 2) == 0)
  //   {
  //     digitalWrite(solenoidPin,HIGH);
  //     solenoidActive = true;
  //   }
  //   else
  //   {
  //     digitalWrite(solenoidPin,LOW);
  //     solenoidActive = false;
  //   }
  //   tempTimer.reset();
  // }

  Serial.println(String(heading) + "   " + String(currentServoAngle));
}


void DriveForSeconds(float seconds) {
  driveTimer.reset();
  driveTimer.setInterval(1000 * seconds);
}

void SteerToHeading(int setHeading) {
  //PD controller
  float error = setHeading - heading;
  float output = (kp * error) + kd * ((error - prevError) / servoUpdateInterval);
  prevError = error;
  currentServoAngle = servoInitOffset + constrain(output * servoLimit, -servoLimit, servoLimit);
  servo.write(currentServoAngle);
}

void ReadCompass() {
  float tempGyroSum = 0.0f;
  int i = 0;
  while(i < gyroscopeSampleMultiplier)
  {
    imu.readGyro();
    tempGyroSum = (imu.g.z - gyro_offset);
    i++;
  }
  float gz = tempGyroSum / gyroscopeSampleMultiplier;
  
  mag.read();
  float mx = mapfloat(mag.m.x, mag_min.x, mag_max.x, -1.0f, 1.0f);
  float my = mapfloat(mag.m.y, mag_min.y, mag_max.y, -1.0f, 1.0f);

  SensorFusionUpdate(gz, mx, my, invSampleFreq);
}

void SensorFusionUpdate(float gz, float mx, float my, float dt) {
  float magAngle = atan2(my, mx) * (180 / M_PI);
  heading = alpha * (heading + -gz * dt) + (1 - alpha) * magAngle;

  heading = fmod(heading, 360.0);
  if(heading < 0) 
    heading += 360;
}

void CalibrateMagnetometer() {
  if (Recalibrate)
  {
    SimpleTimer calibrationTimer(CalibrationDuration);
    Serial.println("Calibrating magnetometer: " + String(CalibrationDuration/1000) + " seconds. Rotate around Z.");
    while(true)
    {
      if (calibrationTimer.isReady()) { break; }

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
    float tempSum = 0.0f;
    int i = 0;
    while(true)
    {
      if (calibrationTimer.isReady()) { break; }

      imu.readGyro();
      tempSum += imu.g.z;
      i++;

      delay(10);
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

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
