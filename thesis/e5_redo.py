"""E5 repeat of 2026-10-23 (thesis Section 6.7): sheets 5-9, release firmware, one homing per sheet.
Encoder diameter of every circle run = least-squares circle fitted to the forward kinematics of its encoder telemetry
(drawing phase). Paper diameters = (outer + inner) / 2 from the user's caliper form, copied unchanged.
Outputs: figdata/e5_redo_encoder.csv (per sheet and radius), figdata/e5_redo.csv (every paper reading with its encoder value).
"""
import csv, glob, json, os
from datetime import datetime
import numpy as np
from openpyxl import load_workbook

ROOT = os.path.expandvars(r'%LOCALAPPDATA%\TwoLinkCommissioning')
HERE = os.path.dirname(os.path.abspath(__file__)); OUT = os.path.join(HERE, 'figdata')
FORM = os.path.join(HERE, '..', 'outputs', '01a1670e-20f9-7330-9f59-488b91fa542f', '2026-10-23-B笔尖实测填写表.xlsx')

# homings of the release-firmware session start the sheets 5..9 (the first homing is the post-flash check)
log = glob.glob(os.path.join(ROOT, 'Logs', 'stage1-20261023-192829-*.log'))[0]
homes = [datetime.strptime('20261023' + l[1:9], '%Y%m%d%H:%M:%S') for l in open(log, encoding='utf-8', errors='replace') if 'HOME OK' in l]
sheet_start = dict(zip(range(5, 10), homes[1:6]))

def sheet_of(t):
    return max((s for s, h in sheet_start.items() if h <= t), default=None)

enc = {}
for d in sorted(glob.glob(os.path.join(ROOT, 'Experiments', '20261023-*-trajectory-*'))):
    t = datetime.strptime(os.path.basename(d)[:15], '%Y%m%d-%H%M%S')
    if t < homes[1]: continue
    st = dict(kv.split('=', 1) for kv in json.load(open(os.path.join(d, 'metadata.json'), encoding='utf-8-sig'))['settings'].split('; ') if '=' in kv)
    assert st['shape'] == '3' and st['x'] == '230' and st['y'] == '0'
    rows = [r for r in csv.DictReader(open(os.path.join(d, 'feedback.csv'), encoding='utf-8-sig')) if r['phase'] == 'drawing' and r['x_hybrid_estimate_mm']]
    x = np.array([float(r['x_hybrid_estimate_mm']) for r in rows]); y = np.array([float(r['y_hybrid_estimate_mm']) for r in rows])
    a, b, c = np.linalg.lstsq(np.column_stack([x, y, np.ones_like(x)]), x * x + y * y, rcond=None)[0]
    enc.setdefault((sheet_of(t), int(float(st['a']))), []).append(2 * np.sqrt(c + a * a / 4 + b * b / 4))
assert sorted(len(v) for v in enc.values()) == [1] * 10 + [4] * 5, 'expected 4 passes of r=50 and one of r=25/75 per sheet'

form = load_workbook(FORM, data_only=True)['B笔尖测量']
paper = [(int(r[0]), int(r[1]), int(r[2]), (float(r[4]) + float(r[5])) / 2, (float(r[4]) - float(r[5])) / 2)
         for r in form.iter_rows(min_row=8, max_row=67, values_only=True)]

os.makedirs(OUT, exist_ok=True)
with open(os.path.join(OUT, 'e5_redo_encoder.csv'), 'w', newline='') as fh:
    w = csv.writer(fh); w.writerow(['sheet', 'radius', 'runs', 'encoder_mm'])
    for (s, r), v in sorted(enc.items()): w.writerow([s, r, len(v), f'{np.mean(v):.4f}'])
with open(os.path.join(OUT, 'e5_redo.csv'), 'w', newline='') as fh:
    w = csv.writer(fh); w.writerow(['sheet', 'design_mm', 'direction', 'paper_mm', 'line_mm', 'encoder_mm'])
    for s, r, ang, dc, lw in paper: w.writerow([s, 2 * r, ang, f'{dc:.3f}', f'{lw:.3f}', f'{np.mean(enc[(s, r)]):.4f}'])

for r in (25, 50, 75):
    p = np.array([[dc for s, rr, a, dc, lw in paper if rr == r and s == k] for k in range(5, 10)])   # sheet x direction
    e = np.array([np.mean(enc[(k, r)]) for k in range(5, 10)])
    print(f'd={2*r}: paper {p.mean():.3f} (sheet means {", ".join(f"{m:.3f}" for m in p.mean(1))}; SD between sheets {p.mean(1).std(ddof=1):.4f}, '
          f'max roundness {np.ptp(p, 1).max():.3f}); encoder {e.mean():.3f} (range {e.min():.3f}-{e.max():.3f}); paper - encoder {p.mean() - e.mean():+.3f} mm')
D = np.array([2 * r for s, r, a, dc, lw in paper]); P = np.array([dc for s, r, a, dc, lw in paper])
k, c = np.polyfit(D, P, 1); print(f'fit paper = (1+k) d + c: k = {100*(k-1):+.3f} %, c = {c:+.3f} mm, residual {np.std(P - np.polyval([k, c], D), ddof=2):.4f} mm')
print(f'line width (outer-inner)/2: mean {np.mean([lw for *_, lw in paper]):.3f} mm')
cl = {s: np.mean([dc - np.mean(enc[(s, r)]) for s, r, a, dc, lw in paper if s == s0]) for s0 in range(5, 10) for s in [s0]}
print('paper - encoder per sheet (all circles):', {s: round(v, 3) for s, v in cl.items()})

# Chord check: the planned nodes lie on the circle, but the motors move in a straight line in joint space between them
# (Section 4.5). Diameter of that interpolated path, sampled uniformly in time like the telemetry, with the same link model.
L1, L2 = 180.0, 176.38
def fk(q1, q2):
    return L1 * np.cos(np.radians(q1)) + L2 * np.cos(np.radians(q2)), L1 * np.sin(np.radians(q1)) + L2 * np.sin(np.radians(q2))
def fit_d(x, y):
    a, b, c = np.linalg.lstsq(np.column_stack([x, y, np.ones_like(x)]), x * x + y * y, rcond=None)[0]
    return 2 * np.sqrt(c + a * a / 4 + b * b / 4)
chord = {}
for d in sorted(glob.glob(os.path.join(ROOT, 'Experiments', '20261023-*-trajectory-*'))):
    if datetime.strptime(os.path.basename(d)[:15], '%Y%m%d-%H%M%S') < homes[1]: continue
    r = int(float(dict(kv.split('=', 1) for kv in json.load(open(os.path.join(d, 'metadata.json'), encoding='utf-8-sig'))['settings'].split('; ') if '=' in kv)['a']))
    P = list(csv.DictReader(open(os.path.join(d, 'plan.csv'), encoding='utf-8-sig')))
    t, q1, q2 = (np.array([float(p[k]) for p in P]) for k in ('time_ms', 'q1_target_deg', 'q2_target_deg'))
    ts = np.arange(t[0], t[-1], 10.0)
    chord.setdefault(r, []).append((fit_d(*fk(q1, q2)), fit_d(*fk(np.interp(ts, t, q1), np.interp(ts, t, q2)))))
with open(os.path.join(OUT, 'e5_chord.csv'), 'w', newline='') as fh:
    w = csv.writer(fh); w.writerow(['design_mm', 'runs', 'nodes_mm', 'joint_path_mm', 'encoder_mm', 'paper_mm'])
    for r, v in sorted(chord.items()):
        v = np.mean(v, 0); e = np.mean([np.mean(enc[(k, r)]) for k in range(5, 10)]); pp = np.mean([dc for s, rr, a, dc, lw in paper if rr == r])
        w.writerow([2 * r, len(chord[r]), f'{v[0]:.4f}', f'{v[1]:.4f}', f'{e:.4f}', f'{pp:.4f}'])
        print(f'chord d={2*r}: nodes {v[0]-2*r:+.3f}, joint-space path {v[1]-2*r:+.3f}, encoders {e-2*r:+.3f}, paper {pp-2*r:+.3f} mm')
