#include <Wire.h>

float RateRoll, RatePitch, RateYaw;

float AccX,AccY,AccZ;
float AngleRoll, AnglePitch;

float PrevAccX, PrevAccY, PrevAccZ; // Variables to store previous accelerometer readings

float LoopTimer;

float RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYaw;
int RateCalibrationNumber;
float KalmanAngleRoll=0, KalmanUncertaintyAngleRoll=2*2;
float KalmanAnglePitch=0, KalmanUncertaintyAnglePitch=2*2;
float Kalman1DOutput[]={0,0};
void kalman_1d(float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurement) {
  KalmanState=KalmanState+0.004*KalmanInput;
  KalmanUncertainty=KalmanUncertainty + 0.004 * 0.004 * 4 * 4;
  float KalmanGain=KalmanUncertainty * 1/(1*KalmanUncertainty + 3 * 3);
  KalmanState=KalmanState+KalmanGain * (KalmanMeasurement-KalmanState);
  KalmanUncertainty=(1-KalmanGain) * KalmanUncertainty;
  Kalman1DOutput[0]=KalmanState; 
  Kalman1DOutput[1]=KalmanUncertainty;
}

void gyro_update(void) {
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
  // Serial.print("Roll [째] = "); // Around X
  // Serial.print(AngleRoll);
  // Serial.print(" Pitch [째] = "); // Around Y
  // Serial.println(AnglePitch);
  
  // Store current accelerometer readings for next iteration
  PrevAccX = AccX;
  PrevAccY = AccY;
  PrevAccZ = AccZ;
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

  for (RateCalibrationNumber=0; RateCalibrationNumber<2000; RateCalibrationNumber ++) {
    gyro_update();
    RateCalibrationRoll+=RateRoll;
    RateCalibrationPitch+=RatePitch;
    RateCalibrationYaw+=RateYaw;
    delay(1);
  }
  RateCalibrationRoll/=2000;
  RateCalibrationPitch/=2000;
  RateCalibrationYaw/=2000;
  LoopTimer=micros();
}

void loop() {
  gyro_update();
  
  // // Check if orientation is unsafe
  // if (abs(AngleRoll) > 45 || abs(AnglePitch) > 45 || AccZ < -0.7) {
  //   Serial.println("Orientation Unsafe!"); 
  // }

  // // Detect collision
  // float accChangeX = abs(AccX - PrevAccX);
  // float accChangeY = abs(AccY - PrevAccY);
  // float accChangeZ = abs(AccZ - PrevAccZ);
  // float collisionThreshold = 2.0; // Adjust threshold
  // if (accChangeX > collisionThreshold || accChangeY > collisionThreshold || accChangeZ > collisionThreshold) {
  //   Serial.println("Collision Detected!");
  // }

  // delay(50);

  RateRoll-=RateCalibrationRoll;
  RatePitch-=RateCalibrationPitch;
  RateYaw-=RateCalibrationYaw;
  kalman_1d(KalmanAngleRoll, KalmanUncertaintyAngleRoll, RateRoll, AngleRoll);
  KalmanAngleRoll=Kalman1DOutput[0]; 
  KalmanUncertaintyAngleRoll=Kalman1DOutput[1];
  kalman_1d(KalmanAnglePitch, KalmanUncertaintyAnglePitch, RatePitch, AnglePitch);
  KalmanAnglePitch=Kalman1DOutput[0]; 
  KalmanUncertaintyAnglePitch=Kalman1DOutput[1];
  Serial.print("Roll Angle : ");
  Serial.print(KalmanAngleRoll);
  Serial.print(" Pitch Angle : ");
  Serial.println(KalmanAnglePitch);
  while (micros() - LoopTimer < 4000);
  LoopTimer=micros();
}
