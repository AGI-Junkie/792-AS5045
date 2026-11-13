"""Offline study of the per-revolution encoder error (thesis Sections 6.4-6.5).

Model: the encoder reading is the true motor angle plus a periodic error h(phi) of the RAW single-turn sensor angle phi.
The firmware zeroes each axis at its home sensor, so the logged following error is  err = h(phi) - h(phi_origin)  (motor LSB).
Tests: (1) one harmonic model in the raw frame fits every homing session, (2) the per-session constant is predicted by
-h(origin) with no fitting, (3) a model fitted on the 10-14 circles alone removes the error from every 10-15 run,
(4) what that does to the smallest step loss the following-error check can catch (synthetic step-loss injection).
Inputs: experiment records and connection logs of 10-14/10-15 (%LOCALAPPDATA%/TwoLinkCommissioning). Outputs: figdata/*.csv.
"""
import csv, glob, json, os, re
from datetime import datetime, timedelta
import numpy as np

ROOT = os.path.expandvars(r'%LOCALAPPDATA%\TwoLinkCommissioning')
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'figdata')
LSB_PER_REV, G1, G2 = 4096, 40.0, 51.06
K1, K2 = G1 * LSB_PER_REV / 360, G2 * LSB_PER_REV / 360          # motor LSB per output degree
FULL_STEP = LSB_PER_REV / 200                                     # 20.48 LSB
FW_FAIL = 328                                                     # SLIP_FAIL_LSB in main.c

# --- homing origins (raw single-turn LSB), local time --------------------------------------------------------------
homes = {1: [], 2: []}
for log in glob.glob(os.path.join(ROOT, 'Logs', 'stage1-2026101[45]-*.log')):
    day = datetime.strptime(os.path.basename(log)[7:15], '%Y%m%d')
    for line in open(log, encoding='utf-8', errors='replace'):
        m = re.match(r'^\[(\d\d):(\d\d):(\d\d)\.(\d+)\] HOME arm([12]) averaged origin: samples=\d+ raw=(\d+)', line)
        if m:
            t = day + timedelta(hours=int(m[1]), minutes=int(m[2]), seconds=int(m[3]))
            homes[int(m[5])].append((t, int(m[6])))
for k in homes: homes[k].sort()

def origin_before(axis, t):
    prior = [h for h in homes[axis] if h[0] <= t]
    return prior[-1] if prior else None

# --- records ---------------------------------------------------------------------------------------------------------
runs = []
for d in sorted(glob.glob(os.path.join(ROOT, 'Experiments', '2026101[45]-*'))):
    try: meta = json.load(open(os.path.join(d, 'metadata.json'), encoding='utf-8-sig'))
    except Exception: continue
    if meta.get('reference') != '开关回零': continue      # manual "arm straight" references are not zeroed at the home sensor
    start = datetime.strptime(os.path.basename(d)[:15], '%Y%m%d-%H%M%S')       # folder name is local time, like the logs
    h1, h2 = origin_before(1, start), origin_before(2, start)
    if not h1 or not h2: continue
    rows = [r for r in csv.DictReader(open(os.path.join(d, 'feedback.csv'), encoding='utf-8-sig'))
            if r['slip_motor_lsb'] and r['slip2_motor_lsb'] and r['elbow_encoder_deg']]
    if len(rows) < 5: continue
    f = lambda k: np.array([float(r[k]) for r in rows])
    runs.append(dict(name=os.path.basename(d), kind=meta['kind'], start=start, settings=meta.get('settings', ''),
                     session=(h1[0], h2[0]), o1=h1[1], o2=h2[1], q1=f('q1_encoder_deg'), el=f('elbow_encoder_deg'),
                     e1=f('slip_motor_lsb'), e2=f('slip2_motor_lsb'), moving=np.array([r['phase'] in ('drawing', 'point') for r in rows])))
print(len(runs), 'records,', len({r['session'] for r in runs}), 'homing sessions')

def basis(phi):                                                   # phi in raw LSB
    a = 2 * np.pi * phi / LSB_PER_REV
    return np.column_stack([np.cos(a), np.sin(a), np.cos(2 * a), np.sin(2 * a)])

def raw_angle(r, axis, sign, offset):
    if axis == 1: return np.mod(r['o1'] + sign * r['q1'] * K1, LSB_PER_REV)          # switch is 126 deg = 14 motor turns away
    return np.mod(r['o2'] + sign * (r['el'] - offset) * K2, LSB_PER_REV)

def fit_pooled(sel, axis, sign, offset):
    """common harmonics + one constant per session; returns coefficients, constants and residual RMS"""
    sessions = sorted({r['session'] for r in sel})
    X, y = [], []
    for r in sel:
        B = basis(raw_angle(r, axis, sign, offset)); S = np.zeros((len(B), len(sessions))); S[:, sessions.index(r['session'])] = 1
        X.append(np.hstack([B, S])); y.append(r['e1'] if axis == 1 else r['e2'])
    X, y = np.vstack(X), np.concatenate(y)
    c, *_ = np.linalg.lstsq(X, y, rcond=None)
    return c[:4], dict(zip(sessions, c[4:])), float(np.sqrt(np.mean((y - X @ c) ** 2)))

# --- sensor-frame conventions: one sign for arm 1; sign and switch side (+-162 deg) for arm 2, chosen by pooled fit ---
h = lambda c, phi: basis(np.atleast_1d(phi)) @ c
best = {}
for axis, options in ((1, [(s, 0) for s in (1, -1)]), (2, [(s, o) for s in (1, -1) for o in (162.0, -162.0)])):
    scores = []
    for s, o in options:
        c, consts, rms = fit_pooled(runs, axis, s, o)
        # the switch side shifts every session's raw angle equally, so only the predicted constants -h(origin) can tell it
        origin = {r['session']: r['o1'] if axis == 1 else r['o2'] for r in runs}
        const_err = float(np.sqrt(np.mean([(consts[k] + h(c, origin[k])[0]) ** 2 for k in consts])))
        scores.append((round(rms, 2), round(const_err, 2), s, o))
    print('axis', axis, '(residual RMS, constant-prediction RMS, sign, switch side):', scores)
    best[axis] = min(scores)[2:]

# --- test 1: independent fit per session (raw frame) -> amplitude and phase of the first harmonic ---------------------
rows = []
for axis in (1, 2):
    s, o = best[axis]
    for sess in sorted({r['session'] for r in runs}):
        sel = [r for r in runs if r['session'] == sess]
        n = sum(len(r['e1']) for r in sel)
        phi = np.concatenate([raw_angle(r, axis, s, o) for r in sel]); y = np.concatenate([r['e1' if axis == 1 else 'e2'] for r in sel])
        cover = len(np.unique((phi // 64).astype(int))) / 64                                   # fraction of the revolution seen
        if cover < 0.9 or n < 300: continue
        c, *_ = np.linalg.lstsq(np.hstack([basis(phi), np.ones((len(phi), 1))]), y, rcond=None)
        origin = sel[0]['o1'] if axis == 1 else sel[0]['o2']
        rows.append([axis, sess[0].strftime('%m-%d %H:%M'), origin, n, len(sel),
                     np.hypot(c[0], c[1]), np.degrees(np.arctan2(c[1], c[0])), np.hypot(c[2], c[3]), c[4], -h(c[:4], origin)[0]])
with open(os.path.join(OUT, 'enc_sessions.csv'), 'w', newline='') as fh:
    w = csv.writer(fh); w.writerow(['axis', 'homing', 'origin_lsb', 'samples', 'runs', 'h1_amp_lsb', 'h1_phase_deg', 'h2_amp_lsb', 'const_lsb', 'const_predicted_lsb']); w.writerows(rows)
for r in rows: print('  axis %d  homed %s  origin %4d  n=%5d runs=%3d  H1 %.1f LSB @ %6.1f deg  H2 %.1f LSB  const %.1f (predicted -h(origin) %.1f)' % tuple(r))

# --- test 2/3: model from the 10-14 E4-A circles only, applied with NO per-session fitting to every other run --------
train = [r for r in runs if r['start'].day == 14 and 'shape=3' in r['settings'] and 'x=220' in r['settings'] and r['kind'] == 'trajectory']
model = {}
for axis in (1, 2):
    s, o = best[axis]; c, consts, rms = fit_pooled(train, axis, s, o)
    model[axis] = c
    print(f'axis {axis}: trained on {len(train)} circle runs, H1 {np.hypot(c[0], c[1]):.1f} LSB, H2 {np.hypot(c[2], c[3]):.1f} LSB, residual {rms:.2f} LSB')

per_run, const_rows = [], []
for r in runs:
    out = dict(name=r['name'], kind=r['kind'], day=r['start'].day, train=r in train)
    for axis in (1, 2):
        s, o = best[axis]; c = model[axis]
        phi = raw_angle(r, axis, s, o); y = r['e1'] if axis == 1 else r['e2']
        origin = r['o1'] if axis == 1 else r['o2']
        comp = y - (h(c, phi) - h(c, origin)[0])                    # what the firmware would see with the model applied
        out[f'raw_rms{axis}'] = float(np.sqrt(np.mean(y ** 2))); out[f'raw_max{axis}'] = float(np.max(np.abs(y)))
        out[f'comp_rms{axis}'] = float(np.sqrt(np.mean(comp ** 2))); out[f'comp_max{axis}'] = float(np.max(np.abs(comp)))
        out[f'comp{axis}'] = comp; out[f'raw{axis}'] = y
    per_run.append(out)

test = [p for p in per_run if not p['train']]
for axis in (1, 2):
    K = K1 if axis == 1 else K2
    for label, sel in (('train', [p for p in per_run if p['train']]), ('test', test)):
        raw = np.concatenate([p[f'raw{axis}'] for p in sel]); comp = np.concatenate([p[f'comp{axis}'] for p in sel])
        print(f'axis {axis} {label:5}: {len(sel):3} runs  raw RMS {np.sqrt(np.mean(raw**2)):5.1f} max {np.max(np.abs(raw)):5.0f} LSB'
              f'  ->  compensated RMS {np.sqrt(np.mean(comp**2)):4.1f} max {np.max(np.abs(comp)):4.0f} LSB'
              f'  ({np.sqrt(np.mean(comp**2))/K:.4f} deg RMS at the output)')

with open(os.path.join(OUT, 'enc_runs.csv'), 'w', newline='') as fh:
    w = csv.writer(fh); keys = ['name', 'kind', 'day', 'train', 'raw_rms1', 'raw_max1', 'comp_rms1', 'comp_max1', 'raw_rms2', 'raw_max2', 'comp_rms2', 'comp_max2']
    w.writerow(keys); w.writerows([[p[k] for k in keys] for p in per_run])

# --- test 4: synthetic step loss on the 10-15 test runs --------------------------------------------------------------
# A lost step leaves the encoder N full steps behind the count for the rest of the run. Detection = some later sample
# beyond the threshold. Thresholds: firmware 328 LSB; tightest no-false-alarm threshold on raw error; same on compensated.
rng = np.random.default_rng(1)
moving = [r for r, p in zip(runs, per_run) if not p['train'] and r['moving'].sum() >= 20]
pmap = {p['name']: p for p in per_run}
thr = {}
for axis in (1, 2):
    raw_max = max(p[f'raw_max{axis}'] for p in per_run); comp_max = max(p[f'comp_max{axis}'] for p in per_run)
    thr[axis] = dict(firmware=FW_FAIL, raw_tight=np.ceil(raw_max * 1.2), comp_tight=np.ceil(comp_max * 1.2))
    print(f'axis {axis}: largest |error| over all runs raw {raw_max:.0f} LSB, compensated {comp_max:.0f} LSB -> thresholds {thr[axis]}')
steps = np.array([0.25, 0.5, 0.75, 1, 1.5, 2, 3, 4, 6, 8, 10, 12, 14, 16, 18, 20])
det_rows = []
for axis in (1, 2):
    for n in steps:
        hits = {k: 0 for k in thr[axis]}; trials = 0
        for r in moving:
            p = pmap[r['name']]; idx = np.flatnonzero(r['moving'])
            for _ in range(20):
                i0 = rng.choice(idx[: max(1, int(len(idx) * 0.7))]); sgn = rng.choice([-1, 1]); trials += 1
                for k, T in thr[axis].items():
                    sig = p[f'comp{axis}'] if k == 'comp_tight' else p[f'raw{axis}']
                    hits[k] += bool(np.any(np.abs(sig[i0:] + sgn * n * FULL_STEP) > T))
        det_rows.append([axis, n] + [hits[k] / trials for k in thr[axis]])
with open(os.path.join(OUT, 'enc_detection.csv'), 'w', newline='') as fh:
    w = csv.writer(fh); w.writerow(['axis', 'full_steps', 'firmware_328', 'raw_tight', 'comp_tight']); w.writerows(det_rows)
for axis in (1, 2):
    for k, j in (('firmware', 2), ('raw_tight', 3), ('comp_tight', 4)):
        sure = [r[1] for r in det_rows if r[0] == axis and r[j] >= 0.99]
        print(f'axis {axis} {k:10} threshold {thr[axis][k]:4.0f} LSB: detects >=99% from {min(sure) if sure else ">20"} full steps')
json.dump({str(a): {k: float(v) for k, v in thr[a].items()} for a in thr} | {'model': {str(a): [float(x) for x in model[a]] for a in model},
          'convention': {str(a): list(best[a]) for a in best}}, open(os.path.join(OUT, 'enc_model.json'), 'w'), indent=1)

# --- scatter data for the figure: 10-15 samples against raw angle, before/after -----------------------------------------
with open(os.path.join(OUT, 'enc_rawframe.csv'), 'w', newline='') as fh:
    w = csv.writer(fh); w.writerow(['axis', 'day', 'phi_deg', 'err_lsb', 'model_lsb', 'comp_lsb'])
    for r, p in zip(runs, per_run):
        for axis in (1, 2):
            s, o = best[axis]; phi = raw_angle(r, axis, s, o); y = r['e1'] if axis == 1 else r['e2']
            origin = r['o1'] if axis == 1 else r['o2']; mdl = h(model[axis], phi) - h(model[axis], origin)[0]
            for k in range(0, len(phi), 3):
                w.writerow([axis, r['start'].day, round(phi[k] * 360 / LSB_PER_REV, 2), y[k], round(mdl[k], 2), round(y[k] - mdl[k], 2)])

# --- firmware table (DRV8825.cydsn/enc_comp.h, used when ENC_COMP=1): corrected = reading + h(reading) -----------------
# h was fitted against the reading itself (err = command - measured = h(reading) - h(origin)), so no inversion is needed.
table = {a: np.round(h(model[a], np.arange(LSB_PER_REV))).astype(int) for a in (1, 2)}
assert all(np.abs(t).max() <= 127 for t in table.values())
hdr = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'DRV8825.cydsn', 'enc_comp.h')
with open(hdr, 'w', newline='\n') as fh:
    fh.write('/* Generated by thesis/enc_error_study.py - do not edit.\n'
             ' * Per-revolution AS5045 error of each motor, identified from the 22 circle runs of 2026-10-14 (switch homing):\n'
             ' * corrected angle = raw + ENC_COMP_LSB[axis][raw], LSB of the 12-bit single-turn angle.\n')
    for a in (1, 2):
        c = model[a]
        fh.write(f' * axis {a - 1}: first harmonic {np.hypot(c[0], c[1]):.1f} LSB, second {np.hypot(c[2], c[3]):.1f} LSB, range {table[a].min()}..{table[a].max()}\n')
    fh.write(' * Valid only for these magnets as mounted: re-identify after any change to an encoder or magnet. */\n')
    fh.write('static const int8 ENC_COMP_LSB[2][4096] = {\n')
    for a in (1, 2):
        vals = [str(v) for v in table[a]]
        fh.write('{' + ',\n'.join(','.join(vals[i:i + 32]) for i in range(0, LSB_PER_REV, 32)) + '}' + (',\n' if a == 1 else '\n'))
    fh.write('};\n')
print('wrote', hdr)
