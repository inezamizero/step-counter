// Desktop test: runs the EXACT firmware algorithm (firmware/09_.../step_detector.h)
// over my recorded walk and checks it counts 20 steps.
//
//   g++ -std=c++17 -O2 tests/test_step_detector.cpp -o test_step_detector && ./test_step_detector
#include <cstdio>
#include <cstdlib>
#include "../firmware/09_step_counter_v5_bluetooth/step_detector.h"

int main() {
  FILE *f = fopen("data/walk_20_steps.csv", "r");
  if (!f) { printf("Run from the repository root (data/walk_20_steps.csv not found)\n"); return 2; }
  char header[128];
  if (!fgets(header, sizeof header, f)) return 2;   // skip the CSV header

  StepDetector detector;
  double t, x, y, z, mag;
  int samples = 0;
  while (fscanf(f, "%lf,%lf,%lf,%lf,%lf", &t, &x, &y, &z, &mag) == 5) {
    detector.update((float)mag);
    samples++;
  }
  fclose(f);

  const unsigned expected = 20;
  printf("%d samples, counted %u steps (expected %u): %s\n", samples, detector.steps(), expected,
         detector.steps() == expected ? "PASS" : "FAIL");
  return detector.steps() == expected ? 0 : 1;
}
