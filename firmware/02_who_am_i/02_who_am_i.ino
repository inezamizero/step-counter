// 02: read the LIS3DH WHO_AM_I register (0x0F). A real LIS3DH always returns 0x33.
// Result on my board: WHO_AM_I = 0x33, read took ~551 us at the default 100 kHz.
#include <Wire.h>

const int PIN_SDA = 16;
const int PIN_SCL = 17;
const uint8_t LIS3DH_ADDR  = 0x19;   // from the scan
const uint8_t REG_WHO_AM_I = 0x0F;   // datasheet register map

// Read one byte from a register. Returns -1 if the chip didn't respond.
int readRegister(uint8_t reg) {
  Wire.beginTransmission(LIS3DH_ADDR);   // 1) address the chip...
  Wire.write(reg);                       //    ...and tell it which register
  int err = Wire.endTransmission(false); // false = repeated start (keep the bus)
  if (err != 0) {
    Serial.printf("Write failed, error %d\n", err);
    return -1;
  }
  Wire.requestFrom(LIS3DH_ADDR, (uint8_t)1);  // 2) ask for 1 byte back
  if (Wire.available() < 1) return -1;
  return Wire.read();                         // 3) the register's contents
}

void setup() {
  Serial.begin(115200);
  delay(1500);
  Wire.begin(PIN_SDA, PIN_SCL);

  unsigned long t0 = micros();
  int id = readRegister(REG_WHO_AM_I);
  unsigned long t1 = micros();

  Serial.printf("WHO_AM_I = 0x%02X  (expected 0x33)\n", id);
  Serial.printf("Read took %lu microseconds\n", t1 - t0);
  if (id == 0x33) Serial.println("It's a LIS3DH!");
  else            Serial.println("Unexpected ID. Check wiring/address.");
}

void loop() {}
