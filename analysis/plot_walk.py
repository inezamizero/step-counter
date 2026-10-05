"""Plot the recorded walk: raw and low-pass-filtered |a|.   python3 analysis/plot_walk.py
Needs matplotlib (pip install matplotlib). Writes media/walk_plot.png."""
import csv, os
import matplotlib; matplotlib.use('Agg')
import matplotlib.pyplot as plt
from replay import Lowpass

HERE = os.path.dirname(os.path.abspath(__file__))
with open(os.path.join(HERE, '..', 'data', 'walk_20_steps.csv')) as f:
    rows = list(csv.DictReader(f))
t0 = float(rows[0]['t_ms'])
t = [(float(r['t_ms']) - t0) / 1000 for r in rows]
mag = [float(r['mag_g']) for r in rows]
lp = Lowpass(); filt = [lp(m) for m in mag]

fig, ax = plt.subplots(2, 1, figsize=(11, 6.5))
ax[0].plot(t, mag, lw=1, color='#2a78d6')
for s, e, label in [(0, 15.5, 'on table'), (16, 23.5, 'picked up'), (24, 28, 'move to hip'),
                    (31.5, 49.5, 'walking (20 steps)'), (50, 59, 'standing')]:
    ax[0].axvspan(s, e, color='#f0efec', zorder=0)
    ax[0].text((s + e) / 2, 1.33, label, ha='center', fontsize=9, color='#52514e')
ax[0].set_ylim(0.78, 1.38); ax[0].set_ylabel('|a| (g)'); ax[0].set_title('Whole recording', loc='left')
w = [i for i, x in enumerate(t) if 31 < x < 50]
ax[1].plot([t[i] for i in w], [mag[i] for i in w], lw=1, color='#b9b8b3', label='raw |a|')
ax[1].plot([t[i] for i in w], [filt[i] for i in w], lw=2, color='#2a78d6', label='low-pass 4 Hz (as on device)')
ax[1].set_xlabel('time (s)'); ax[1].set_ylabel('|a| (g)'); ax[1].set_title('Zoom on the walk', loc='left')
ax[1].legend(frameon=False, loc='upper right', ncol=2, bbox_to_anchor=(1, 1.14))
for a in ax:
    a.spines['top'].set_visible(False); a.spines['right'].set_visible(False)
plt.tight_layout()
plt.savefig(os.path.join(HERE, '..', 'media', 'walk_plot.png'), dpi=130)
print('wrote media/walk_plot.png')
