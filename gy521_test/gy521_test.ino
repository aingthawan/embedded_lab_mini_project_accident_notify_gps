#include <Wire.h>

float RateRoll, RatePitch, RateYaw;

float AccX,AccY,AccZ;
float AngleRoll, AnglePitch;

float LoopTimer;

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
  // Around x
  Serial.print("Roll [°] = ");
  Serial.print(AngleRoll);
  // Around y
  Serial.print(" Pitch [°] = ");
  Serial.println(AnglePitch);
}

void setup() {
  Serial.begin(57600);
  Wire.setClock(400000);
  Wire.begin(12, 13);
  delay(250);
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
}

void loop() {
  gyro_signals();
  // Serial.print("Acc X [g]= ");
  // Serial.print(AccX);
  // Serial.print(" Acc Y [g] = ");
  // Serial.print(AccY);
  // Serial.print(" Acc Z [g] = ");
  // Serial.println(AccZ);

  // Around x
  Serial.print("Roll [°] = ");
  Serial.print(AngleRoll);
  // Around y
  Serial.print(" Pitch [°] = ");
  Serial.println(AnglePitch);
  

  // Check if orientation is unsafe
  if (abs(AngleRoll) > 45 || abs(AnglePitch) > 45 || AccZ < -0.7) {
    Serial.println("Orientation Unsafe!"); 
    // tone(15, 1000, 5);
  }

  delay(50);
}


