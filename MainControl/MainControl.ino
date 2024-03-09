#include <Wire.h>
#include <LIS3MDL.h>
#include <LSM6.h>
#include <SimpleTimer.h>
#include <EEPROM.h>
#include <Servo.h>

// Compass settings
bool Recalibrate = false;
const int CalibrationDuration = 10000;
const int CompassSampleFrequency = 10;
int gyroscopeSampleMultiplier = 50;
float heading = 0.0;
const float alpha = 0.1;

// Servo settings
Servo servo;
int servoPin = 9;
int servoInitOffset = 78;

// Solenoid settings
int solenoidPin = 2;

#pragma region Compass_Internal
  LSM6 imu;
  LIS3MDL mag;
  float invSampleFreq;
  LIS3MDL::vector<int16_t> mag_min = {32767, 32767, 32767}, mag_max = {-32768, -32768, -32768};
  int gyro_offset = 0;
  SimpleTimer CompassUpdateTimer;
#pragma endregion

void setup() {
  Serial.begin(115200);
  Wire.begin();

  // Compass init
  if (!mag.init() || !imu.init())
  {
    Serial.println("Failed to compass sensor.");
    while (1);
  }
  mag.enableDefault();
  imu.enableDefault();
  CalibrateMagnetometer();
  CalibrateGyroscope();
  invSampleFreq = (1.0f / CompassSampleFrequency); 
  CompassUpdateTimer.setInterval(1000 * invSampleFreq);

  // Servo init
  servo.attach(servoPin);
  servo.write(servoInitOffset); // (+/-25)

  // Solenoid init
  pinMode(solenoidPin, OUTPUT);
  digitalWrite(solenoidPin,LOW);
}

void loop() {
  if(CompassUpdateTimer.isReady())
  {
    //ReadCompass();
    CompassUpdateTimer.reset();
  }

  if (Serial.available()) {
    int number = Serial.parseInt();
    Serial.println(String(number));
    
    if(number == 1)
      digitalWrite(solenoidPin,HIGH);
    if(number == 0)
      digitalWrite(solenoidPin,LOW);
  }
  while (Serial.available() > 0) {
      Serial.read();
  }
}

void CalibrateMagnetometer() {
  if (Recalibrate)
  {
    SimpleTimer CalibrationTimer(CalibrationDuration);
    Serial.println("Calibrating magnetometer: " + String(CalibrationDuration/1000) + " seconds. Rotate around Z.");
    while(true)
    {
      if (CalibrationTimer.isReady()) { break; }

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
    SimpleTimer CalibrationTimer(CalibrationDuration/2);
    Serial.println("Calibrating gyroscope: " + String(CalibrationDuration/2000) + " seconds. Hold still.");
    float tempSum = 0.0f;
    int i = 0;
    while(true)
    {
      if (CalibrationTimer.isReady()) { break; }

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
  Serial.println(String(heading));
}

void SensorFusionUpdate(float gz, float mx, float my, float dt) {
  float magAngle = atan2(my, mx) * (180 / M_PI);
  heading = alpha * (heading + -gz * dt) + (1 - alpha) * magAngle;

  heading = fmod(heading, 360.0);
}

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
