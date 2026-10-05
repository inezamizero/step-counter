// 01: I2C scanner - find which addresses answer on the bus.
// Result on my board: one device at 0x19 (the LIS3DH, address jumper open).
#include <Wire.h>

// The ESP32-S3 can route I2C to almost any GPIO. IO16/IO17 are the pins
// LilyGO uses for I2C in their T-QT SensorBNO080 example.
const int PIN_SDA = 16;
const int PIN_SCL = 17;

void setup() {
  Serial.begin(115200);
  delay(1500);                    // give the Mac time to open the USB serial port

  Wire.begin(PIN_SDA, PIN_SCL);   // start I2C as the controller on our pins

  Serial.println("Scanning I2C bus...");
  int found = 0;

  // 7-bit addresses go from 0 to 127; 0 and 127 are reserved, so scan 1..126
  for (int addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);          // queue the address
    int result = Wire.endTransmission();   // send it; 0 means a device replied (ACK)
    if (result == 0) {
      Serial.printf("Found device at 0x%02X\n", addr);
      found++;
    }
  }

  if (found == 0) Serial.println("No devices found. Check 3V, GND, SDA->IO16, SCL->IO17.");
  Serial.println("Done.");
}

void loop() {}
