"""Hardware validation of the compensated firmware, 2026-10-23 (thesis Section 6.6 and Appendix C).

Research build -enccomp-TESTDROP: the AS5045 error table is applied in the firmware, the following check during PTP and
trajectories is 32 / 40 LSB, and every even trajectory run since power-up swallows 8/16/32 microsteps 3 s after it starts.
Inputs: connection logs and experiment records of 10-23 (%LOCALAPPDATA%/TwoLinkCommissioning) up to the reflash of the
release firmware. Telemetry of this build is already compensated, so slip_motor_lsb is the residual error.
Outputs: figdata/hw_runs.csv (every clean run and move), figdata/hw_injections.csv (every injected run).
"""
import csv, glob, json, os, re
from datetime import datetime
import numpy as np

ROOT = os.path.expandvars(r'%LOCALAPPDATA%\TwoLinkCommissioning')
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'figdata')
FW = 'guards2-enccomp-TESTDROP'
THR = {1: 32, 2: 40}
SHAPES = {'3': 'Circle', '1': 'Line', '2': 'Rectangle', '6': 'Rounded rectangle', '8': 'Pentagon', '7': 'Figure-8', '0': 'Point'}

# --- logs of the test build: injections, stops, the time span of the build ---------------------------------------------
ts = lambda day, s: datetime.strptime(day + s[:8], '%Y%m%d%H:%M:%S').timestamp() + float('0.' + s[9:])
drops, stops, span = [], [], [None, None]
for log in sorted(glob.glob(os.path.join(ROOT, 'Logs', 'stage1-20261023-*.log'))):
    day, test_build = os.path.basename(log)[7:15], False
    for line in open(log, encoding='utf-8', errors='replace'):
        m = re.match(r'^\[(\d\d:\d\d:\d\d\.\d+)\] (.*)', line.rstrip())
        if not m: continue
        t, msg = ts(day, m[1]), m[2]
        if msg.startswith('LINK ') and ' fw=' in msg: test_build = msg.split(' fw=')[1].split()[0].endswith(FW)
        if not test_build: continue
        span[0] = span[0] or t; span[1] = t
        if (d := re.match(r'TEST_DROP run=(\d+) axis=(\d) usteps=(\d+)', msg)):
            drops.append(dict(t=t, run=int(d[1]), axis=int(d[2]), usteps=int(d[3])))
        elif msg.startswith('FAULT_SNAPSHOT '):
            kv = dict(x.split('=', 1) for x in msg.split()[1:] if '=' in x)
            stops.append(dict(t=t, dev_ms=int(kv['t']), phase=kv['phase'], err1=int(kv['err']), err2=int(kv['err2']), reason=None))
        elif msg.startswith('STOP: ') and stops and stops[-1]['reason'] is None and t - stops[-1]['t'] < 1:
            stops[-1]['reason'] = msg[6:].split(';')[0]

# --- records of the test build -------------------------------------------------------------------------------------------
recs = []
for d in sorted(glob.glob(os.path.join(ROOT, 'Experiments', '20261023-*'))):
    start = datetime.strptime(os.path.basename(d)[:15], '%Y%m%d-%H%M%S').timestamp()
    if not (span[0] <= start <= span[1]): continue
    meta = json.load(open(os.path.join(d, 'metadata.json'), encoding='utf-8-sig'))
    st = dict(kv.split('=', 1) for kv in meta['settings'].split('; ') if '=' in kv)
    summ = os.path.join(d, 'summary.json')
    outcome = json.load(open(summ))['outcome'] if os.path.exists(summ) else 'no summary'
    rows = [r for r in csv.DictReader(open(os.path.join(d, 'feedback.csv'), encoding='utf-8-sig'))
            if r['phase'] in ('drawing', 'point') and r['slip_motor_lsb'] and r['slip2_motor_lsb']]
    e1 = np.array([float(r['slip_motor_lsb']) for r in rows]); e2 = np.array([float(r['slip2_motor_lsb']) for r in rows])
    q1 = np.array([float(r['q1_error_deg']) for r in rows]); el = np.array([float(r['elbow_error_deg'] or 'nan') for r in rows])
    recs.append(dict(name=os.path.basename(d), start=start, dev_ms=np.array([int(r['device_ms']) for r in rows]), e1=e1, e2=e2, kind=meta['kind'], shape=SHAPES.get(st['shape'], st['shape']),
                     v=(st['v1'], st['v2']), outcome=outcome, n=len(rows),
                     max1=np.abs(e1).max() if len(rows) else np.nan, max2=np.abs(e2).max() if len(rows) else np.nan,
                     rms_q1=np.sqrt(np.mean(q1 ** 2)) if len(rows) else np.nan, peak_q1=np.abs(q1).max() if len(rows) else np.nan,
                     rms_el=np.sqrt(np.nanmean(el ** 2)) if len(rows) else np.nan, peak_el=np.nanmax(np.abs(el)) if len(rows) else np.nan))

# trajectory runs since power-up, in order; the drop line of run k is logged about 3 s after its record starts
traj = [r for r in recs if r['kind'] == 'trajectory']
for k, r in enumerate(traj, 1): r['run'] = k
for dr in drops:
    match = [r for r in traj if 0 <= dr['t'] - r['start'] < 10]
    assert len(match) == 1 and match[0]['run'] == dr['run'], f"run numbering disagrees with the log at run {dr['run']}"
    nxt = [s for s in stops if s['t'] >= dr['t']]
    end = next((r['start'] for r in recs if r['start'] > dr['t']), span[1])          # the next record starts after re-homing
    s = nxt[0] if nxt and nxt[0]['t'] < end and nxt[0]['phase'] == 'traj_run' else None
    dr.update(rec=match[0], stop=s, shape=match[0]['shape'], detected=s is not None, latency_s=round(s['t'] - dr['t'], 3) if s else None,
              err_at_stop=(s['err1'] if dr['axis'] == 1 else s['err2']) if s else None, reason=s['reason'] if s else None)

clean = [r for r in traj if r['run'] % 2 == 1]
points = [r for r in recs if r['kind'] == 'point']
for r in points: r['kind'] = 'E8' if r['start'] > traj[-1]['start'] else 'move to start'   # the ladder came after the last run
e8 = [r for r in points if r['kind'] == 'E8']
false_alarms = [s for s in stops if s['reason'] and 'encoder following' in s['reason']
                and not any(0 <= s['t'] - d['t'] < 60 for d in drops)]
other_stops = [s for s in stops if s['reason'] and 'encoder following' not in s['reason']]

os.makedirs(OUT, exist_ok=True)
with open(os.path.join(OUT, 'hw_runs.csv'), 'w', newline='') as fh:
    w = csv.writer(fh)
    w.writerow(['record', 'kind', 'run', 'shape', 'v1', 'v2', 'outcome', 'samples', 'max_err1_lsb', 'max_err2_lsb',
                'rms_q1_deg', 'peak_q1_deg', 'rms_elbow_deg', 'peak_elbow_deg'])
    for r in clean + points:
        w.writerow([r['name'], r['kind'], r.get('run', ''), r['shape'], *r['v'], r['outcome'], r['n'], f"{r['max1']:.0f}",
                    f"{r['max2']:.0f}", f"{r['rms_q1']:.4f}", f"{r['peak_q1']:.4f}", f"{r['rms_el']:.4f}", f"{r['peak_el']:.4f}"])
with open(os.path.join(OUT, 'hw_injections.csv'), 'w', newline='') as fh:
    w = csv.writer(fh)
    w.writerow(['run', 'shape', 'axis', 'microsteps', 'full_steps', 'detected', 'latency_s', 'err_at_stop_lsb'])
    for d in drops:
        w.writerow([d['run'], d['shape'], d['axis'], d['usteps'], d['usteps'] / 8, int(d['detected']), d['latency_s'] or '', d['err_at_stop'] or ''])

# figure data: a clean circle and the following four-step injection, per axis (telemetry, plus the error in the stop snapshot)
with open(os.path.join(OUT, 'hw_trace.csv'), 'w', newline='') as fh:
    w = csv.writer(fh); w.writerow(['axis', 'run', 'injected', 't_s', 'err_lsb', 'at_stop'])
    for axis, run in ((1, 10), (2, 12)):
        d = next(d for d in drops if d['run'] == run and d['axis'] == axis)
        for r, inj in ((traj[run - 2], 0), (d['rec'], 1)):
            t0, e = r['dev_ms'][0], r['e1'] if axis == 1 else r['e2']
            for tm, v in zip(r['dev_ms'], e): w.writerow([axis, r['run'], inj, f'{(tm - t0) / 1000:.3f}', f'{v:.0f}', 0])
            if inj: w.writerow([axis, r['run'], 1, f"{(d['stop']['dev_ms'] - t0) / 1000:.3f}", d['err_at_stop'], 1])

# --- report --------------------------------------------------------------------------------------------------------------
print(f"test build {datetime.fromtimestamp(span[0]):%H:%M}-{datetime.fromtimestamp(span[1]):%H:%M}: {len(traj)} trajectory runs, "
      f"{len(points)} point moves, {len(drops)} injections")
print(f"clean runs: {len(clean)}, completed {sum(r['outcome'] == 'complete' for r in clean)}; "
      f"point moves completed {sum(r['outcome'] == 'complete' for r in points)}/{len(points)}")
print(f"false alarms (encoder following stops outside injected runs): {len(false_alarms)}; "
      f"other stops: {[(datetime.fromtimestamp(s['t']).strftime('%H:%M:%S'), s['phase'], s['reason']) for s in other_stops]}")
for shape in dict.fromkeys(r['shape'] for r in clean):
    g = [r for r in clean if r['shape'] == shape]
    print(f"  {shape:17s} n={len(g):2d}  RMS q1 {np.mean([r['rms_q1'] for r in g]):.4f}  peak q1 {max(r['peak_q1'] for r in g):.4f}  "
          f"RMS elbow {np.mean([r['rms_el'] for r in g]):.4f}  peak elbow {max(r['peak_el'] for r in g):.4f} deg;  "
          f"max |err| {max(r['max1'] for r in g):.0f} / {max(r['max2'] for r in g):.0f} LSB (thresholds 32 / 40)")
circ = [r['rms_q1'] for r in clean if r['shape'] == 'Circle']
print(f"  circle RMS q1 over {len(circ)} runs: mean {np.mean(circ):.4f}, SD {np.std(circ, ddof=1):.4f} deg")
starts = [r for r in points if r['kind'] == 'move to start']
print(f"  moves to start: {len(starts)}, max |err| {max(r['max1'] for r in starts):.0f} / {max(r['max2'] for r in starts):.0f} LSB")
for v in dict.fromkeys(r['v'] for r in e8):
    g = [r for r in e8 if r['v'] == v]
    print(f"  E8 v={v}: {len(g)} moves, completed {sum(r['outcome'] == 'complete' for r in g)}, "
          f"max |err| {max(r['max1'] for r in g):.0f} / {max(r['max2'] for r in g):.0f} LSB, peak q1 {max(r['peak_q1'] for r in g):.4f}, "
          f"peak elbow {max(r['peak_el'] for r in g):.4f} deg")
for fs in (1, 2, 4):
    g = [d for d in drops if d['usteps'] == fs * 8]
    print(f"{fs} full step(s): detected {sum(d['detected'] for d in g)}/{len(g)};  " +
          ', '.join(f"run {d['run']} arm{d['axis']} {d['shape']}: " + (f"{d['latency_s']} s, {d['err_at_stop']} LSB" if d['detected'] else 'not detected') for d in g))
