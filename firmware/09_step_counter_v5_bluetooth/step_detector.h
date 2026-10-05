// step_detector.h - the step-counting algorithm, kept free of Arduino code so the
// exact same file runs on the ESP32 and in the desktop test (tests/test_step_detector.cpp).
//
// Pipeline, once per sample (50 Hz):
//   |a| -> 4 Hz low-pass -> subtract slow baseline -> adaptive hysteresis -> timing rules
#pragma once
#include <math.h>
#include <stdint.h>

struct StepConfig {
  float sampleHz      = 50.0f;  // matches CTRL_REG1 = 0x47
  float lowpassHz     = 4.0f;   // keep walking (1-3 Hz), drop jitter
  float baselineTauS  = 2.0f;   // how slowly the "resting level" follows the signal
  float armFraction   = 0.4f;   // arm at 40% of a typical step above baseline
  float armFloor      = 0.03f;  // g: never arm below this (keeps standing-still noise out)
  float peakSmoothing = 0.25f;  // how fast "typical step size" adapts (0..1)
  float minGapS       = 0.30f;  // closer than this = double bump, ignore
  float maxGapS       = 2.0f;   // longer than this = the walk has stopped
  int   stepsToStart  = 4;      // regular steps in a row before counting
  float rhythmRatio   = 1.4f;   // while confirming, each gap must be within 1.4x of the previous
};

class StepDetector {
public:
  explicit StepDetector(const StepConfig &c = StepConfig()) : cfg(c) { setupLowpass(); }

  // Feed one sample of total acceleration in g. Returns true if the step count went up.
  bool update(float mag) {
    sampleNum++;
    float now = sampleNum / cfg.sampleHz;
    float f = lowpass(mag);

    // Remove the resting level (gravity + sensor offset), whatever the tilt.
    baseline += (f - baseline) / (cfg.baselineTauS * cfg.sampleHz);
    float s = f - baseline;

    // No step for too long: the walk is over, forget unconfirmed steps.
    if (haveLastStep && now - lastStepTime > cfg.maxGapS) {
      haveLastStep = false; walkingNow = false; pending = 0;
    }

    // Adaptive hysteresis: arm when the signal rises 40% of a typical step above
    // baseline, count one candidate when it falls back below baseline.
    float armLevel = cfg.armFraction * typical;
    if (armLevel < cfg.armFloor) armLevel = cfg.armFloor;

    if (!armed && s > armLevel) {
      armed = true;
      swingMax = s;
    } else if (armed) {
      if (s > swingMax) swingMax = s;
      if (s < 0) {
        armed = false;
        typical += cfg.peakSmoothing * (swingMax - typical);   // learn step size
        uint32_t before = count;
        onCandidate(now);
        return count != before;
      }
    }
    return false;
  }

  uint32_t steps() const { return count; }
  void setSteps(uint32_t n) { count = n; }
  bool walking() const { return walkingNow; }
  float lastGap() const { return prevGap; }
  float typicalStep() const { return typical; }

private:
  // Timing rules applied to each candidate step.
  void onCandidate(float now) {
    if (!haveLastStep) {                       // first swing of a possible walk
      haveLastStep = true; lastStepTime = now; prevGap = 0; pending = 1;
      return;
    }
    float gap = now - lastStepTime;
    if (gap < cfg.minGapS) return;             // double bump: ignore it entirely
    lastStepTime = now;

    bool offRhythm = prevGap > 0 && (gap > prevGap * cfg.rhythmRatio || gap < prevGap / cfg.rhythmRatio);
    prevGap = gap;

    if (walkingNow) {                          // already confirmed: count it
      count++;
    } else if (offRhythm) {                    // while confirming, rhythm must be steady
      pending = 1;
    } else if (++pending >= cfg.stepsToStart) {// walk confirmed: credit the buffered steps
      count += pending;
      pending = 0;
      walkingNow = true;
    }
  }

  // 2nd-order Butterworth low-pass (RBJ Audio EQ Cookbook coefficients).
  void setupLowpass() {
    const float PI_F = 3.14159265f;
    float w0 = 2 * PI_F * cfg.lowpassHz / cfg.sampleHz;
    float alpha = sinf(w0) / (2 * 0.7071f);    // Q = 0.7071 -> Butterworth (no overshoot)
    float a0 = 1 + alpha;
    b0 = (1 - cosf(w0)) / 2 / a0;
    b1 = (1 - cosf(w0)) / a0;
    b2 = b0;
    a1 = -2 * cosf(w0) / a0;
    a2 = (1 - alpha) / a0;
  }

  float lowpass(float x) {
    float y = b0 * x + b1 * in1 + b2 * in2 - a1 * out1 - a2 * out2;
    in2 = in1; in1 = x;
    out2 = out1; out1 = y;
    return y;
  }

  StepConfig cfg;
  float b0 = 0, b1 = 0, b2 = 0, a1 = 0, a2 = 0;
  float in1 = 1, in2 = 1, out1 = 1, out2 = 1;   // filter memory, starts at rest (1 g)

  float baseline = 1.0f;    // slow average of the filtered signal (~1 g at rest)
  float typical = 0.10f;    // g above baseline: how big recent steps have been
  float swingMax = 0;       // highest point of the current swing
  bool armed = false;

  uint32_t count = 0;
  uint32_t sampleNum = 0;
  bool haveLastStep = false;
  float lastStepTime = 0;
  float prevGap = 0;
  int pending = 0;
  bool walkingNow = false;
};
