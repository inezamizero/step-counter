// 03: configure the LIS3DH and print X/Y/Z acceleration in g.
// Flat and still, my board reads about x 0.01, y 0.00, z 0.985 g (gravity on Z).
#include <Wire.h>

const int PIN_SDA = 16;
const int PIN_SCL = 17;
const uint8_t LIS3DH_ADDR = 0x19;

// Register addresses (LIS3DH datasheet, register map)
const uint8_t REG_WHO_AM_I  = 0x0F;
const uint8_t REG_CTRL_REG1 = 0x20;
const uint8_t REG_CTRL_REG4 = 0x23;
const uint8_t REG_OUT_X_L   = 0x28;

void writeRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(LIS3DH_ADDR);
  Wire.write(reg);     // which register
  Wire.write(value);   // what to put in it
  Wire.endTransmission();
}

int readRegister(uint8_t reg) {
  Wire.beginTransmission(LIS3DH_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return -1;
  Wire.requestFrom(LIS3DH_ADDR, (uint8_t)1);
  return Wire.available() ? Wire.read() : -1;
}

void setup() {
  Serial.begin(115200);
  delay(1500);
  Wire.begin(PIN_SDA, PIN_SCL, 400000);   // 400 kHz fast mode

  if (readRegister(REG_WHO_AM_I) != 0x33) {
    Serial.println("LIS3DH not found!");
    while (true) delay(1000);
  }

  // CTRL_REG1 = 0100 0111: ODR=0100 (50 Hz), LPen=0 (normal), Z/Y/X enabled
  writeRegister(REG_CTRL_REG1, 0x47);
  // CTRL_REG4 = 1001 1000: BDU=1, FS=01 (+/-4 g), HR=1 (12-bit high resolution)
  writeRegister(REG_CTRL_REG4, 0x98);
  Serial.println("LIS3DH configured.");
}

void loop() {
  uint8_t raw[6];

  // Read all 6 output bytes in one go (bit 7 of the address = auto-increment)
  Wire.beginTransmission(LIS3DH_ADDR);
  Wire.write(REG_OUT_X_L | 0x80);
  Wire.endTransmission(false);
  Wire.requestFrom(LIS3DH_ADDR, (uint8_t)6);
  for (int i = 0; i < 6; i++) raw[i] = Wire.read();

  // low + high byte -> signed 16-bit, then drop the 4 unused bits (12-bit, left-justified)
  int16_t x = (int16_t)(raw[0] | (raw[1] << 8)) >> 4;
  int16_t y = (int16_t)(raw[2] | (raw[3] << 8)) >> 4;
  int16_t z = (int16_t)(raw[4] | (raw[5] << 8)) >> 4;

  // 2 mg per count at +/-4 g in high-resolution mode
  Serial.printf("x:%.3f,y:%.3f,z:%.3f\n", x * 0.002f, y * 0.002f, z * 0.002f);
  delay(100);
}
