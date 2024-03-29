#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <Wire.h>

float RateRoll, RatePitch, RateYaw;

float AccX,AccY,AccZ;
float AngleRoll, AnglePitch;

float LoopTimer;
static const int RXPin = 4, TXPin = 5;
static const uint32_t GPSBaud = 9600;
long LateLat, LateLn;

// The TinyGPS++ object
TinyGPSPlus gps;

// The serial connection to the GPS device
SoftwareSerial ss(RXPin, TXPin);

void setup() {
  Serial.begin(9600);
  ss.begin(GPSBaud);

  Wire.setClock(400000);
  Wire.begin(12, 13);
  delay(250);
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
}

void gyro_signals(void) {
  // EN lowpass filter
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission(); 
  //config accel.meter output
  Wire.beginTransmission(0x68);
  Wire.write(0x1C); 
  Wire.write(0x10); 
  Wire.endTransmission(); 
  // get accel. value
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(0x68,6);
  int16_t AccXLSB=Wire.read()<<8 | Wire.read();
  int16_t AccYLSB=Wire.read()<<8 | Wire.read();
  int16_t AccZLSB=Wire.read()<<8 | Wire.read();
  // config gyro output
  Wire.beginTransmission(0x68);
  Wire.write(0x1B);
  Wire.write(0x8);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(0x68,6);
  int16_t GyroX=Wire.read()<<8 | Wire.read();
  int16_t GyroY=Wire.read()<<8 | Wire.read();
  int16_t GyroZ=Wire.read()<<8 | Wire.read();
  RateRoll=(float)GyroX/4096;
  RateRoll=(float)GyroY/4096;
  RateRoll=(float)GyroZ/4096;
  // convert to g. + calibration offset
  AccX=(float)AccXLSB/4096 - 0.03;
  AccY=(float)AccYLSB/4096 - 0.01;
  AccZ=(float)AccZLSB/4096 + 0.09;
  // Angle Measurement
  AngleRoll=atan( AccY / sqrt(AccX*AccX + AccZ*AccZ))*1 / (3.142/180);
  AnglePitch=atan( AccX / sqrt(AccY*AccY + AccZ*AccZ))*1 / (3.142/180);
}

void readGPSData() {
  // This function reads GPS data once
  int updated = 0;
  while (updated != 1) {
    while (ss.available() > 0) {
      gps.encode(ss.read());
      if (gps.location.isUpdated()) {
        LateLat = gps.location.lat();
        LateLn = gps.location.lng();
        updated = 1;
      }
    }
  }
}

void loop() {
  // Call the function to read GPS data
  readGPSData();
  gyro_signals();

  Serial.print(" Roll [°] = ");
  Serial.print(" Pitch [°] = ");
}
