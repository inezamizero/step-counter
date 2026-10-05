// step_counter_v2: LIS3DH step counter with an adaptive threshold
#include <Wire.h>
#include <math.h>

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
  Wire.begin(PIN_SDA, PIN_SCL, 400000);

  if (readRegister(REG_WHO_AM_I) != 0x33) {
    Serial.println("LIS3DH not found!");
    while (true) delay(1000);
  }
  writeRegister(REG_CTRL_REG1, 0x47);  // 50 Hz, X/Y/Z on
  writeRegister(REG_CTRL_REG4, 0x98);  // BDU, ±4 g, high resolution

  setupLowpass(LOWPASS_HZ, SAMPLE_HZ);
  Serial.println("Step counter ready. Start walking!");
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
}
