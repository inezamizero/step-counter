// step_counter_v5: v4 + Bluetooth (phone reads total steps, sets the clock) + "today" with midnight rollover.
// The step algorithm lives in step_detector.h (no Arduino code, so it is tested on a computer too).
#include <Wire.h>
#include <Preferences.h>  // ESP32 key-value storage in flash (NVS)
#include <sys/time.h>     // settimeofday() / time() for the real clock
#include <BLEDevice.h>    // Bluetooth Low Energy (built into the ESP32 Arduino core)
#include <BLEServer.h>
#include <BLE2902.h>
#include <math.h>
#include "step_detector.h"
#include <TFT_eSPI.h>   // LilyGO's patched copy (pins + panel setup live in its User_Setup)

// ---------- Hardware ----------
const int PIN_SDA = 16;
const int PIN_SCL = 17;
const uint8_t LIS3DH_ADDR = 0x19;

const uint8_t REG_WHO_AM_I  = 0x0F;
const uint8_t REG_CTRL_REG1 = 0x20;
const uint8_t REG_CTRL_REG4 = 0x23;
const uint8_t REG_STATUS    = 0x27;
const uint8_t REG_OUT_X_L   = 0x28;

// Total steps ever counted (declared up here because many sections use it)
uint32_t stepCount = 0;

StepDetector detector;   // all thresholds and timing rules: see StepConfig in step_detector.h

// ---------- Saving the count ----------
// Flash wears out after many writes, so we don't save on every step.
// We save at most every 30 s, and only if the count changed.
Preferences prefs;
const uint32_t SAVE_EVERY_MS = 30000;
uint32_t lastSaveMs = 0;
uint32_t savedCount = 0;

// Left button (IO0) held for 2 s resets the count. Buttons read LOW when pressed.
const int PIN_BTN_LEFT = 0;
const uint32_t RESET_HOLD_MS = 2000;
uint32_t btnDownSince = 0;

// ---------- Bluetooth ----------
// Our own GATT "service" with two "characteristics" (think: a tiny API with two endpoints).
// UUIDs are just unique IDs we made up; the iPhone app will look for these exact ones.
#define SERVICE_UUID    "7e1a0001-5f3b-4c4e-9a3e-2b8f6c1d0a01"
#define STEPS_CHAR_UUID "7e1a0002-5f3b-4c4e-9a3e-2b8f6c1d0a01"  // read + notify: total steps, uint32 little-endian
#define TIME_CHAR_UUID  "7e1a0003-5f3b-4c4e-9a3e-2b8f6c1d0a01"  // write: 4 bytes UTC seconds + 4 bytes timezone offset

BLECharacteristic *stepsChar = nullptr;
volatile bool bleConnected = false;
uint32_t lastNotifyMs = 0;
uint32_t notifiedCount = 0xFFFFFFFF;

// The phone's time arrives in the Bluetooth task; we copy it here and apply it in loop()
volatile bool timeReceived = false;
volatile uint32_t rxUtc = 0;
volatile int32_t rxOffset = 0;

class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *server) override { bleConnected = true; }
  void onDisconnect(BLEServer *server) override {
    bleConnected = false;
    server->getAdvertising()->start();   // advertise again so the phone can reconnect later
  }
};

class TimeCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *c) override {
    std::string v = c->getValue();
    if (v.size() < 8) return;
    const uint8_t *b = (const uint8_t *)v.data();
    rxUtc    = b[0] | (b[1] << 8) | (b[2] << 16) | ((uint32_t)b[3] << 24);
    rxOffset = (int32_t)(b[4] | (b[5] << 8) | (b[6] << 16) | ((uint32_t)b[7] << 24));
    timeReceived = true;
  }
};

void setupBluetooth() {
  BLEDevice::init("QT-Steps");                      // the name you'll see when scanning
  BLEServer *server = BLEDevice::createServer();
  server->setCallbacks(new ServerCallbacks());
  BLEService *service = server->createService(SERVICE_UUID);

  stepsChar = service->createCharacteristic(STEPS_CHAR_UUID,
      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  stepsChar->addDescriptor(new BLE2902());          // lets the phone switch notifications on

  BLECharacteristic *timeChar = service->createCharacteristic(TIME_CHAR_UUID,
      BLECharacteristic::PROPERTY_WRITE);
  timeChar->setCallbacks(new TimeCallbacks());

  service->start();
  BLEAdvertising *adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(SERVICE_UUID);                // phones can scan for just our service
  adv->setScanResponse(true);
  BLEDevice::startAdvertising();
}

void publishSteps(uint32_t total) {
  uint8_t le[4] = { (uint8_t)total, (uint8_t)(total >> 8), (uint8_t)(total >> 16), (uint8_t)(total >> 24) };
  stepsChar->setValue(le, 4);
  if (bleConnected) stepsChar->notify();
}

// ---------- Clock and "today" ----------
// The device keeps TWO numbers:
//   stepCount  = total steps, never reset (the phone syncs this to Apple Health)
//   today      = stepCount - dayStartCount (what the screen shows; rolls over at midnight)
bool clockSet = false;
int32_t tzOffset = 0;             // seconds east of UTC (New York in October = -4 h = -14400)
uint32_t dayStartCount = 0;       // total at the start of today
int32_t dayNumber = -1;           // days since 1970 in local time (changes at local midnight)

uint32_t localNow() { return (uint32_t)time(nullptr) + tzOffset; }

void checkMidnight() {
  if (!clockSet) return;                       // can't know the date until the phone tells us
  int32_t d = localNow() / 86400;
  if (d != dayNumber) {                        // new day (or first time we learn the date)
    if (dayNumber != -1) dayStartCount = stepCount;   // midnight passed: today starts from here
    dayNumber = d;
    prefs.putUInt("dayStart", dayStartCount);
    prefs.putInt("dayNum", dayNumber);
    Serial.printf("New day %ld, today starts at total %lu\n", (long)dayNumber, (unsigned long)dayStartCount);
  }
}

// ---------- Display ----------
// The sprite is an off-screen 128x128 image in RAM (32 KB). We draw into it, then
// copy the finished frame to the screen in one go, so the number never flickers.
TFT_eSPI tft;
TFT_eSprite frame = TFT_eSprite(&tft);

uint32_t shownToday = 0xFFFFFFFF;   // what's on screen now (forces the first draw)
bool shownWalking = false, shownBle = false;
int shownMinute = -1;

void drawScreen(uint32_t today, bool isWalking) {
  frame.fillSprite(TFT_BLACK);
  frame.setTextDatum(MC_DATUM);              // text is centred on the x,y we give

  // clock at the top (only once the phone has set it)
  if (clockSet) {
    uint32_t t = localNow();
    char clk[6];
    snprintf(clk, sizeof(clk), "%02lu:%02lu", (unsigned long)((t / 3600) % 24), (unsigned long)((t / 60) % 60));
    frame.setTextColor(TFT_DARKGREY, TFT_BLACK);
    frame.drawString(clk, 64, 12, 2);
  }

  char buf[12];
  snprintf(buf, sizeof(buf), "%lu", (unsigned long)today);
  frame.setTextColor(TFT_WHITE, TFT_BLACK);
  int font = 7;                              // 48 px 7-segment digits
  if (frame.textWidth(buf, 7) > 120) font = 4;   // too wide (5+ digits)? use 26 px font
  frame.drawString(buf, 64, 60, font);

  frame.setTextColor(TFT_DARKGREY, TFT_BLACK);
  frame.drawString("steps today", 64, 98, 2);

  // dots: cyan = walking, blue = phone connected
  frame.fillCircle(56, 118, 4, isWalking ? TFT_CYAN : 0x2104);
  frame.fillCircle(72, 118, 4, bleConnected ? TFT_BLUE : 0x2104);

  frame.pushSprite(0, 0);                    // send the finished frame to the screen
}

// ---------- I2C helpers ----------
void writeRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(LIS3DH_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

int readRegister(uint8_t reg) {
  Wire.beginTransmission(LIS3DH_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return -1;
  Wire.requestFrom(LIS3DH_ADDR, (uint8_t)1);
  return Wire.available() ? Wire.read() : -1;
}

// ---------- Setup & loop ----------
void setup() {
  Serial.begin(115200);
  delay(1500);

  // start the screen first, so errors can be shown on it
  tft.init();                    // also turns the backlight on (IO10 LOW)
  tft.setRotation(0);
  tft.fillScreen(TFT_BLACK);
  frame.createSprite(128, 128);

  Wire.begin(PIN_SDA, PIN_SCL, 400000);

  if (readRegister(REG_WHO_AM_I) != 0x33) {
    Serial.println("LIS3DH not found!");
    frame.fillSprite(TFT_BLACK);
    frame.setTextDatum(MC_DATUM);
    frame.setTextColor(TFT_RED, TFT_BLACK);
    frame.drawString("No sensor", 64, 54, 2);
    frame.setTextColor(TFT_WHITE, TFT_BLACK);
    frame.drawString("check wiring", 64, 76, 2);
    frame.pushSprite(0, 0);
    while (true) delay(1000);
  }
  writeRegister(REG_CTRL_REG1, 0x47);  // 50 Hz, X/Y/Z on
  writeRegister(REG_CTRL_REG4, 0x98);  // BDU, ±4 g, high resolution

  // load the last saved count ("steps" namespace, key "total"; 0 if never saved)
  prefs.begin("steps", false);
  stepCount = prefs.getUInt("total", 0);
  detector.setSteps(stepCount);
  savedCount = stepCount;
  dayStartCount = prefs.getUInt("dayStart", 0);
  dayNumber = prefs.getInt("dayNum", -1);      
  if (dayStartCount > stepCount) dayStartCount = stepCount;

  setupBluetooth();                            
  publishSteps(stepCount);
  pinMode(PIN_BTN_LEFT, INPUT_PULLUP);

  Serial.printf("Step counter ready. Restored %lu steps.\n", (unsigned long)stepCount);
}

void loop() {
  int status = readRegister(REG_STATUS);
  if (status < 0 || !(status & 0x08)) return;   // wait for new data (ZYXDA)

  uint8_t raw[6];
  Wire.beginTransmission(LIS3DH_ADDR);
  Wire.write(REG_OUT_X_L | 0x80);
  Wire.endTransmission(false);
  Wire.requestFrom(LIS3DH_ADDR, (uint8_t)6);
  for (int i = 0; i < 6; i++) raw[i] = Wire.read();

  float gx = ((int16_t)(raw[0] | (raw[1] << 8)) >> 4) * 0.002f;
  float gy = ((int16_t)(raw[2] | (raw[3] << 8)) >> 4) * 0.002f;
  float gz = ((int16_t)(raw[4] | (raw[5] << 8)) >> 4) * 0.002f;

  if (detector.update(sqrtf(gx * gx + gy * gy + gz * gz))) {
    stepCount = detector.steps();
    Serial.printf("STEPS: %lu   (gap %.2f s, step size %.2f g)\n",
                  (unsigned long)stepCount, detector.lastGap(), detector.typicalStep());
  }
  bool walking = detector.walking();

  // save to flash every 30 s if the count changed
  if (stepCount != savedCount && millis() - lastSaveMs > SAVE_EVERY_MS) {
    prefs.putUInt("total", stepCount);
    savedCount = stepCount;
    lastSaveMs = millis();
    Serial.println("(saved)");
  }

  // hold the left button 2 s to reset to 0 (non-blocking: sampling keeps running while held)
  static bool resetDone = false;
  if (digitalRead(PIN_BTN_LEFT) == LOW) {
    if (btnDownSince == 0) btnDownSince = millis();
    if (!resetDone && millis() - btnDownSince > RESET_HOLD_MS) {
      detector.setSteps(0);
      stepCount = 0;
      dayStartCount = 0;
      prefs.putUInt("total", 0);
      prefs.putUInt("dayStart", 0);
      savedCount = 0;
      resetDone = true;              // only once per press
      Serial.println("Count reset.");
    }
  } else {
    btnDownSince = 0;
    resetDone = false;
  }

  // apply the time the phone sent
  if (timeReceived) {
    timeReceived = false;
    struct timeval tv = { (time_t)rxUtc, 0 };
    settimeofday(&tv, nullptr);
    tzOffset = rxOffset;
    clockSet = true;
    Serial.printf("Clock set by phone: UTC %lu, offset %ld s\n", (unsigned long)rxUtc, (long)rxOffset);
  }
  checkMidnight();

  // tell the phone about new steps, at most once per second
  if (stepCount != notifiedCount && millis() - lastNotifyMs > 1000) {
    publishSteps(stepCount);
    notifiedCount = stepCount;
    lastNotifyMs = millis();
  }

  // Redraw only when something changed. A redraw takes a few ms, well
  // inside the 20 ms between samples, so no sensor data is missed.
  uint32_t today = stepCount - dayStartCount;
  int minute = clockSet ? (int)((localNow() / 60) % 60) : -1;
  if (today != shownToday || walking != shownWalking || bleConnected != shownBle || minute != shownMinute) {
    drawScreen(today, walking);
    shownToday = today;
    shownWalking = walking;
    shownBle = bleConnected;
    shownMinute = minute;
  }
}
