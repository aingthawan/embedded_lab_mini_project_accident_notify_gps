#include <TinyGPS++.h>
#include <SoftwareSerial.h>

static const int RXPin = 4, TXPin = 5;
static const uint32_t GPSBaud = 9600;

// The TinyGPS++ object
TinyGPSPlus gps;

// The serial connection to the GPS device
SoftwareSerial ss(RXPin, TXPin);

void setup() {
  Serial.begin(9600);
  ss.begin(GPSBaud);
}

void readGPSData() {
  // This function reads GPS data once
  int updated = 0;
  while (updated != 1) {
    while (ss.available() > 0) {
      gps.encode(ss.read());
      if (gps.location.isUpdated()) {
        // Latitude in degrees (double)
        Serial.print("Latitude= ");
        Serial.print(gps.location.lat(), 6);
        // Longitude in degrees (double)
        Serial.print(" Longitude= ");
        Serial.print(gps.location.lng(), 6);

        // Raw date in DDMMYY format (u32)
        Serial.print(" Raw date DDMMYY = ");
        Serial.print(gps.date.value());

        // Raw time in HHMMSSCC format (u32)
        Serial.print(" Raw time in HHMMSSCC = ");
        Serial.print(gps.time.value());

        // Number of satellites in use (u32)
        Serial.print(" Number of satellites in use = ");
        Serial.print(gps.satellites.value());

        // Horizontal Dim. of Precision (100ths-i32)
        Serial.print(" HDOP = ");
        Serial.println(gps.hdop.value());
        
        updated = 1;
      }
    }
  }
}

void loop() {
  // Call the function to read GPS data
  readGPSData();
  delay(10000);
}
