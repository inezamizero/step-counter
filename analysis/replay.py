"""Replay the recorded walk through the same step-detection logic as the firmware.

    python3 analysis/replay.py

The recording (data/walk_20_steps.csv) is 59 s at 50 Hz: board on the table,
picked up, moved to my hip, exactly 20 steps, then standing. A correct detector
must report 20 over the WHOLE file, not just the walking part.
"""
import csv, math, os

HERE = os.path.dirname(os.path.abspath(__file__))
FS = 50.0

def load():
    with open(os.path.join(HERE, '..', 'data', 'walk_20_steps.csv')) as f:
        return [float(r['mag_g']) for r in csv.DictReader(f)]

class Lowpass:
    """2nd-order Butterworth low-pass (RBJ Audio EQ Cookbook), same as the firmware."""
    def __init__(self, fc=4.0, fs=FS):
        w0 = 2 * math.pi * fc / fs
        cw, alpha = math.cos(w0), math.sin(w0) / (2 * 0.7071)
        a0 = 1 + alpha
        self.b = ((1 - cw) / 2 / a0, (1 - cw) / a0, (1 - cw) / 2 / a0)
        self.a = (-2 * cw / a0, (1 - alpha) / a0)
        self.x1 = self.x2 = self.y1 = self.y2 = 1.0
    def __call__(self, x):
        y = self.b[0]*x + self.b[1]*self.x1 + self.b[2]*self.x2 - self.a[0]*self.y1 - self.a[1]*self.y2
        self.x2, self.x1, self.y2, self.y1 = self.x1, x, self.y1, y
        return y

class Gate:
    """Timing rules applied to each candidate step."""
    def __init__(self, min_gap=0.3, max_gap=2.0, steps_to_start=4, rhythm=1.4, tolerant=True):
        self.min_gap, self.max_gap, self.need, self.rhythm, self.tolerant = min_gap, max_gap, steps_to_start, rhythm, tolerant
        self.last = None; self.prev = 0; self.pending = 0; self.walking = False; self.steps = 0
    def tick(self, t):
        if self.last is not None and t - self.last > self.max_gap:
            self.last = None; self.walking = False; self.pending = 0
    def candidate(self, t):
        if self.last is None:
            self.last, self.prev, self.pending = t, 0, 1; return
        gap = t - self.last
        if gap < self.min_gap: return
        self.last = t
        off = self.rhythm and self.prev > 0 and (gap > self.prev * self.rhythm or gap < self.prev / self.rhythm)
        self.prev = gap
        if self.walking:
            if off and not self.tolerant: self.walking = False; self.pending = 1; return
            self.steps += 1
        elif off:
            self.pending = 1
        else:
            self.pending += 1
            if self.pending >= self.need:
                self.steps += self.pending; self.pending = 0; self.walking = True

def fixed_threshold(mag, arm=1.06, fire=1.00, **gate):
    """v1: fixed levels."""
    lp, g, armed = Lowpass(), Gate(**gate), False
    for i, m in enumerate(mag):
        t = (i + 1) / FS; f = lp(m); g.tick(t)
        if not armed and f > arm: armed = True
        elif armed and f < fire: armed = False; g.candidate(t)
    return g.steps

def adaptive(mag, frac=0.4, floor=0.03, tau=2.0, smooth=0.25, **gate):
    """v2: arm level = 40% of the typical recent step height above a moving baseline."""
    lp, g, armed = Lowpass(), Gate(**gate), False
    base, typical, swing = 1.0, 0.10, 0.0
    for i, m in enumerate(mag):
        t = (i + 1) / FS; f = lp(m); g.tick(t)
        base += (f - base) / (tau * FS); s = f - base
        level = max(floor, frac * typical)
        if not armed and s > level: armed, swing = True, s
        elif armed:
            swing = max(swing, s)
            if s < 0:
                armed = False; typical += smooth * (swing - typical); g.candidate(t)
    return g.steps

if __name__ == '__main__':
    mag = load()
    print("Effect of each rule (fixed threshold, true count = 20)")
    print("  threshold only            ", fixed_threshold(mag, min_gap=0, max_gap=2.0, steps_to_start=1, rhythm=None))
    print("  + 0.3 s minimum gap       ", fixed_threshold(mag, steps_to_start=1, rhythm=None))
    print("  + 4 steps in a row        ", fixed_threshold(mag, rhythm=None))
    print("  + steady rhythm (1.4x)    ", fixed_threshold(mag, tolerant=False))
    print()
    print("Softer steps (signal scaled toward 1 g)   v1 fixed   v2 adaptive")
    for k in (1.0, 0.6, 0.4):
        m = [1 + (x - 1) * k for x in mag]
        print(f"  {int(k*100):3d}% step strength                 {fixed_threshold(m, tolerant=False):6d}   {adaptive(m):9d}")
