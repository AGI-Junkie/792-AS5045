"""Interval estimates for the thesis (Section 6.1 'Statistical treatment'): E3 spread, E4-A and E8 speed trends, E5 uncertainty.
Reads thesis/figdata (exported from the data workbook)."""
import csv, os
import numpy as np
from scipy import stats

D = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'figdata')
rd = lambda n: list(csv.DictReader(open(os.path.join(D, n), encoding='utf-8')))
LSB1, LSB2 = 360 / (4096 * 40), 360 / (4096 * 51.06)

def sd_ci(x, conf=0.95):
    n = len(x); s = np.std(x, ddof=1); a = (1 - conf) / 2
    return s, s * np.sqrt((n - 1) / stats.chi2.ppf(1 - a, n - 1)), s * np.sqrt((n - 1) / stats.chi2.ppf(a, n - 1))

def slope_ci(x, y, conf=0.95):
    r = stats.linregress(x, y); t = stats.t.ppf(1 - (1 - conf) / 2, len(x) - 2)
    return r.slope, r.slope - t * r.stderr, r.slope + t * r.stderr, r.pvalue

# E3: standard deviation of the homing origin with a chi-square interval (n = 10, and the settled last 5)
e3 = rd('e3_homing.csv')
for arm, lsb in (('arm1_raw', LSB1), ('arm2_raw', LSB2)):
    x = np.array([float(r[arm]) for r in e3])
    for label, sel in (('all 10', x), ('last 5', x[5:])):
        s, lo, hi = sd_ci(sel)
        print(f'E3 {arm} {label}: SD {s:.1f} LSB, 95% CI [{lo:.1f}, {hi:.1f}] LSB = [{lo*lsb:.4f}, {hi*lsb:.4f}] deg')

# E4-A: does RMS error depend on speed?  Regression over the 12 runs; slope per 1000 microsteps/s of planned peak rate
e4 = rd('e4a_speed.csv')
x = np.array([float(r['peak1_usteps']) for r in e4]) / 1000
for col in ('q1_rmse', 'elbow_rmse', 'q1_peak'):
    y = np.array([float(r[col]) for r in e4]); b, lo, hi, p = slope_ci(x, y)
    print(f'E4-A {col}: slope {b*1000:.2f} mdeg per 1000 usteps/s, 95% CI [{lo*1000:.2f}, {hi*1000:.2f}], p = {p:.3f}, n = {len(y)}')
for lev in sorted({r['level'] for r in e4}):
    y = np.array([float(r['q1_rmse']) for r in e4 if r['level'] == lev]); t = stats.t.ppf(0.975, len(y) - 1)
    print(f'E4-A L{lev} q1 RMS mean {y.mean():.4f} deg, 95% CI +-{t*y.std(ddof=1)/np.sqrt(len(y)):.4f} (n = {len(y)})')

# E8: peak following error against requested rate (30 moves)
e8 = rd('e8_ladder.csv')
x = np.array([float(r['rate_req']) for r in e8]) / 1000
back = np.array([r['move'].startswith('B-A') for r in e8], float)         # direction dominates the spread: add it as a term
X = np.column_stack([np.ones_like(x), x, back])
for col in ('q1_peak', 'elbow_peak'):
    y = np.array([float(r[col]) for r in e8]); c, *_ = np.linalg.lstsq(X, y, rcond=None)
    res = y - X @ c; dof = len(y) - 3; cov = np.linalg.inv(X.T @ X) * (res @ res) / dof
    se = np.sqrt(cov[1, 1]); t = stats.t.ppf(0.975, dof); p = 2 * stats.t.sf(abs(c[1] / se), dof)
    print(f'E8 {col}: slope {c[1]*1000:.2f} mdeg per 1000 usteps/s, 95% CI [{(c[1]-t*se)*1000:.2f}, {(c[1]+t*se)*1000:.2f}], '
          f'p = {p:.2g}; B->A offset {c[2]*1000:.1f} mdeg; n = {len(y)}')
for col in ('err1_lsb', 'err2_lsb'):
    y = np.array([float(r[col]) for r in e8]); print(f'E8 {col}: final error values {sorted(set(y.astype(int)))}')

# E5: Type A scatter of the four directions per circle, and a combined uncertainty for one centre-line diameter.
#   Type B (assumptions, stated in the thesis): caliper resolution 0.01 mm -> rectangular, u = 0.01/sqrt(12) per reading;
#   placing a jaw on the edge of a 0.4-0.6 mm graphite line: +-0.05 mm rectangular per reading (operator judgement).
#   Centre value = mean of an outer and an inner reading, so u_c = sqrt(2) * u_reading / 2.
u_res, u_edge = 0.01 / np.sqrt(12), 0.05 / np.sqrt(3)
u_reading = np.hypot(u_res, u_edge); u_centre = np.sqrt(2) * u_reading / 2
print(f'E5 per reading u = {u_reading:.4f} mm, centre-line value u = {u_centre:.4f} mm, expanded U(k=2) = {2*u_centre:.3f} mm')
e5 = rd('e5_circles.csv')
for d in sorted({r['design_mm'] for r in e5}, key=float):
    y = np.array([float(r['paper_mm']) for r in e5 if r['design_mm'] == d])
    print(f'E5 sheet 4 d={d}: mean {y.mean():.3f}, SD of 4 directions {y.std(ddof=1):.4f} mm, u(mean) incl. type B {np.hypot(y.std(ddof=1)/2, u_centre/2):.3f} mm')
print('E5 sheet-to-sheet difference of the 100 mm circle: +0.42 (sheet 3) vs -1.43 mm (sheet 4) = 1.85 mm, i.e. '
      f'{1.85 / (2 * np.sqrt(2) * u_centre):.0f} times the expanded uncertainty of a difference')

# E5 repeat (sheets 5-9, fineliner): edges are sharp, so instead of the graphite edge estimate use what was observed -
# five re-placed readings identical, four directions within one digit - plus the resolution of the two readings.
r = rd('e5_redo.csv')
u_res_c = np.sqrt(2) * (0.01 / np.sqrt(12)) / 2
for d in ('50', '100', '150'):
    m = np.array([np.mean([float(x['paper_mm']) for x in r if x['design_mm'] == d and x['sheet'] == str(k)]) for k in range(5, 10)])
    print(f'E5 repeat d={d}: SD of the five sheet means {m.std(ddof=1):.4f} mm; u(sheet mean) from resolution {u_res_c/2:.4f} mm')
sd = np.array([np.std([float(x['paper_mm']) for x in r if x['design_mm'] == d and x['sheet'] == str(k)], ddof=1)
               for d in ('50', '100', '150') for k in range(5, 10)])
u_rep = np.hypot(u_res_c, np.sqrt(np.mean(sd ** 2)))
print(f'E5 repeat: pooled SD of 4 directions {np.sqrt(np.mean(sd**2)):.4f} mm; one centre value u = {u_rep:.4f} mm, U(k=2) = {2*u_rep:.3f} mm')
