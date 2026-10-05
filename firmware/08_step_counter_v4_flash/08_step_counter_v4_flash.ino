// step_counter_v4: v3 + count saved in flash (survives unplugging) + reset button
#include <Wire.h>
#include <Preferences.h>  // NEW: ESP32 key-value storage in flash (NVS)
#include <math.h>
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

// ---------- Step rules (chosen from the 20-step recording) ----------
const float SAMPLE_HZ      = 50.0;   // matches CTRL_REG1 = 0x47
const float LOWPASS_HZ     = 4.0;    // keep walking (1-3 Hz), drop jitter
const float BASELINE_TAU_S = 2.0;    // how slowly the "resting level" follows the signal
const float ARM_FRACTION   = 0.4;    // arm when the signal rises 40% of a typical step above baseline
const float ARM_FLOOR      = 0.03;   // g: never arm below this (keeps standing-still noise out)
const float PEAK_SMOOTHING = 0.25;   // how fast "typical step size" adapts (0..1)
const float MIN_GAP_S      = 0.30;   // closer than this = double bump, ignore
const float MAX_GAP_S      = 2.0;    // longer than this = you stopped walking
const int   STEPS_TO_START = 4;      // regular steps in a row before counting
const float RHYTHM_RATIO   = 1.4;    // each gap must be within 1.4x of the previous

// ---------- NEW: Saving the count ----------
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

// ---------- Display ----------
// The sprite is an off-screen 128x128 image in RAM (32 KB). We draw into it, then
// copy the finished frame to the screen in one go, so the number never flickers.
TFT_eSPI tft;
TFT_eSprite frame = TFT_eSprite(&tft);

uint32_t shownCount = 0xFFFFFFFF;   // what's on screen now (forces the first draw)
bool shownWalking = false;

void drawScreen(uint32_t steps, bool isWalking) {
  frame.fillSprite(TFT_BLACK);
  frame.setTextDatum(MC_DATUM);              // text is centred on the x,y we give

  char buf[12];
  snprintf(buf, sizeof(buf), "%lu", (unsigned long)steps);
  frame.setTextColor(TFT_WHITE, TFT_BLACK);
  int font = 7;                              // 48 px 7-segment digits
  if (frame.textWidth(buf, 7) > 120) font = 4;   // too wide (5+ digits)? use 26 px font
  frame.drawString(buf, 64, 56, font);

  frame.setTextColor(TFT_DARKGREY, TFT_BLACK);
  frame.drawString("steps", 64, 98, 2);

  // small dot: cyan while a walk is confirmed, dark grey otherwise
  frame.fillCircle(64, 118, 4, isWalking ? TFT_CYAN : 0x2104);

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

// ---------- Low-pass filter (2nd-order Butterworth "biquad") ----------
// Coefficients from the RBJ Audio EQ Cookbook low-pass formula.
// It smooths the signal like a moving average, but blocks fast vibration much better.
float b0, b1, b2, a1, a2;
// filter memory (previous inputs and outputs), start at 1 g = resting.
// (Named in1/out1 because y1 is already a function in math.h.)
float in1 = 1, in2 = 1, out1 = 1, out2 = 1;

void setupLowpass(float cutoffHz, float sampleHz) {
  float w0 = 2 * PI * cutoffHz / sampleHz;
  float alpha = sin(w0) / (2 * 0.7071);   // Q = 0.7071 -> Butterworth (no overshoot)
  float a0 = 1 + alpha;
  b0 = (1 - cos(w0)) / 2 / a0;
  b1 = (1 - cos(w0)) / a0;
  b2 = b0;
  a1 = -2 * cos(w0) / a0;
  a2 = (1 - alpha) / a0;
}

float lowpass(float x) {
  float y = b0 * x + b1 * in1 + b2 * in2 - a1 * out1 - a2 * out2;
  in2 = in1; in1 = x;
  out2 = out1; out1 = y;
  return y;
}

// ---------- Step detector state ----------
uint32_t stepCount = 0;
uint32_t sampleNum = 0;          // counts samples; time = sampleNum / 50
bool armed = false;              // true after the signal rose above the arm level
bool haveLastStep = false;
float lastStepTime = 0;          // seconds
float prevGap = 0;               // seconds between the previous two steps
int pending = 0;                 // candidate steps waiting to be confirmed
bool walking = false;            // true once a bout is confirmed

// Called once per candidate step (one up-and-down swing). Applies the timing rules.
void onCandidateStep(float now) {
  if (!haveLastStep) {                       // first swing of a possible bout
    haveLastStep = true; lastStepTime = now; prevGap = 0; pending = 1;
    return;
  }
  float gap = now - lastStepTime;
  if (gap < MIN_GAP_S) return;               // rule 2: double bump, ignore it entirely
  lastStepTime = now;

  bool offRhythm = prevGap > 0 && (gap > prevGap * RHYTHM_RATIO || gap < prevGap / RHYTHM_RATIO);
  prevGap = gap;

  if (walking) {                             // already confirmed: count it, even if one gap is a bit off
    stepCount++;
  } else if (offRhythm) {                    // rule 4b: while confirming, rhythm must be steady
    pending = 1;
  } else if (++pending >= STEPS_TO_START) {  // rule 4a: confirm bout, credit buffered steps
    stepCount += pending;
    pending = 0;
    walking = true;
  }
}

// Adaptive threshold state
float baseline = 1.0;      // slow-moving average of the filtered signal (~1 g at rest)
float typicalPeak = 0.10;  // g above baseline; how big recent steps have been
float swingMax = 0;        // highest point of the current swing

void processSample(float mag) {
  sampleNum++;
  float now = sampleNum / SAMPLE_HZ;
  float f = lowpass(mag);

  // Remove the resting level. Works whatever the tilt or sensor offset (0.985 vs 1.000).
  baseline += (f - baseline) / (BASELINE_TAU_S * SAMPLE_HZ);
  float s = f - baseline;

  // rule 3: no step for too long -> the bout is over, forget unconfirmed steps
  if (haveLastStep && now - lastStepTime > MAX_GAP_S) {
    haveLastStep = false; walking = false; pending = 0;
  }

  // rule 1 (adaptive): the arm level scales with how big YOUR recent steps are,
  // so soft steps (pocket, bag, gentle walking) still count.
  float armLevel = max(ARM_FLOOR, ARM_FRACTION * typicalPeak);

  if (!armed && s > armLevel) {
    armed = true;
    swingMax = s;
  } else if (armed) {
    if (s > swingMax) swingMax = s;
    if (s < 0) {                                   // fell back below baseline = one full swing
      armed = false;
      typicalPeak += PEAK_SMOOTHING * (swingMax - typicalPeak);   // learn step size
      uint32_t before = stepCount;
      onCandidateStep(now);
      if (stepCount != before) {
        Serial.printf("STEPS: %lu   (gap %.2f s, step size %.2f g)\n", stepCount, prevGap, typicalPeak);
      }
    }
  }
}

// ---------- Setup & loop ----------
void setup() {
  Serial.begin(115200);
  delay(1500);

  // NEW: start the screen first, so errors can be shown on it
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

  setupLowpass(LOWPASS_HZ, SAMPLE_HZ);

  // NEW: load the last saved count ("steps" namespace, key "total"; 0 if never saved)
  prefs.begin("steps", false);
  stepCount = prefs.getUInt("total", 0);
  savedCount = stepCount;
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

  processSample(sqrtf(gx * gx + gy * gy + gz * gz));

  // NEW: save to flash every 30 s if the count changed
  if (stepCount != savedCount && millis() - lastSaveMs > SAVE_EVERY_MS) {
    prefs.putUInt("total", stepCount);
    savedCount = stepCount;
    lastSaveMs = millis();
    Serial.println("(saved)");
  }

  // NEW: hold the left button 2 s to reset to 0
  if (digitalRead(PIN_BTN_LEFT) == LOW) {
    if (btnDownSince == 0) btnDownSince = millis();
    if (millis() - btnDownSince > RESET_HOLD_MS) {
      stepCount = 0;
      prefs.putUInt("total", 0);
      savedCount = 0;
      btnDownSince = 0;
      Serial.println("Count reset.");
      while (digitalRead(PIN_BTN_LEFT) == LOW) delay(10);   // wait for release
    }
  } else {
    btnDownSince = 0;
  }

  // Redraw only when something changed. A redraw takes a few ms, well
  // inside the 20 ms between samples, so no sensor data is missed.
  if (stepCount != shownCount || walking != shownWalking) {
    drawScreen(stepCount, walking);
    shownCount = stepCount;
    shownWalking = walking;
  }
}
