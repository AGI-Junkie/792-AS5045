"""Acceptance check for host logs (%LOCALAPPDATA%\\TwoLinkCommissioning\\Logs\\stage1-*.log).

  python tools/check-log.py --last          newest log
  python tools/check-log.py --all           every log
  python tools/check-log.py PATH [PATH...]  given logs
  python tools/check-log.py --self-test     synthetic cases, no hardware, no logs needed

Exit code 0 = no FAIL, 1 = at least one FAIL.  Counters in the firmware are cumulative since power-up, so link errors
are judged by their increase inside the log; key_max_gap_ms is judged only after the log's first ENABLE (the firmware
resets it there, so earlier values belong to a previous run).
"""
import argparse
import os
import re
import sys
from pathlib import Path

KEY_GAP_MAX_MS = 100   # WORKBENCH_HARDENING.md: acceptance is key_max_gap_ms <= 100 (0.4 x the 250 ms firmware deadman)
LEASE_WARN_MS = 150    # host UI lease gap; the host stops heartbeating at 200 ms (LeaseStopMs)


def analyse(text):
    fails, warns, info = [], [], []
    fw = sorted(set(re.findall(r"fw=(stage2-[a-z0-9-]+)", text)))
    info.append("固件 " + (", ".join(fw) if fw else "（日志里没有 fw= 标识）"))
    if not fw:
        warns.append("日志里没有固件版本标识，无法确认测的是哪一版")

    stops = [s.strip() for s in re.findall(r"STOP: ([^;\r\n]+)", text)]
    protective = [s for s in stops if not s.startswith("operator stop")]
    if protective:
        fails.append("保护性停机 %d 次：%s" % (len(protective), "；".join(sorted(set(protective)))))
    info.append("人为停止 %d 次" % (len(stops) - len(protective)))

    links = re.findall(r"rx_overflow=(\d+) rx_error=(\d+) tx_dropped=(\d+)", text)
    if len(links) >= 2:
        first, last = [tuple(map(int, m)) for m in (links[0], links[-1])]
        for name, a, b in zip(("rx_overflow", "rx_error"), first, last):
            if b > a:
                fails.append("%s 在本次日志内增加 %d" % (name, b - a))
        if last[2] > first[2]:
            warns.append("tx_dropped 增加 %d（状态/回执可能被丢）" % (last[2] - first[2]))

    enable = text.find("ENABLE OK")
    if enable < 0:
        info.append("本次日志没有使能，不评估心跳余量")
    else:
        gaps = [int(v) for v in re.findall(r"key_max_gap_ms=(\d+)", text[enable:])]
        if gaps and max(gaps) > KEY_GAP_MAX_MS:
            fails.append("key_max_gap_ms=%d 超过验收线 %d" % (max(gaps), KEY_GAP_MAX_MS))
        elif gaps:
            info.append("key_max_gap_ms 最大 %d（线 %d）" % (max(gaps), KEY_GAP_MAX_MS))

    leases = [int(v) for v in re.findall(r"HOST UI lease gap max=(\d+)", text)]
    leases += [int(v) for v in re.findall(r"worstUiLeaseGap=(\d+)ms", text)]
    if leases and max(leases) > LEASE_WARN_MS:
        warns.append("上位机 UI 租约间隔最大 %d ms（超过 %d）" % (max(leases), LEASE_WARN_MS))

    for err, limit in re.findall(r"PTP done seq=\d+;.*?error_lsb=(-?\d+) final_limit_lsb=(\d+)", text):
        if abs(int(err)) > int(limit):
            fails.append("终点误差 %s LSB 超过限 %s" % (err, limit))
    done = len(set(re.findall(r"TRAJ_STATE state=4 [^\n]*?seq=(\d+)", text)))
    info.append("完整执行轨迹 %d 条" % done)
    return fails, warns, info


def report(name, text):
    fails, warns, info = analyse(text)
    print("%s  %s" % ("FAIL" if fails else ("WARN" if warns else "PASS"), name))
    for tag, items in (("  FAIL", fails), ("  warn", warns), ("  info", info)):
        for item in items:
            print("%s: %s" % (tag, item))
    return not fails


def self_test():
    base = [
        "[10:00:00.000] LINK uart=hw baud=115200 rx_overflow=3 rx_error=129 tx_dropped=5 fw=stage2-v7-x short_hb=1",
        "[10:00:00.500] TIMING loop_max_ms=4 tick_lag_max_ms=3 clock=hardware wdt=256-384ms key_age_ms=1 key_max_gap_ms=183",
        "[10:00:01.000] ENABLE OK: no automatic movement",
        "[10:00:02.000] TIMING loop_max_ms=4 tick_lag_max_ms=3 clock=hardware wdt=256-384ms key_age_ms=1 key_max_gap_ms=61",
        "[10:00:03.000] PTP done seq=2; arm1 encoder checked, arm2 commanded only; error_lsb=-47 final_limit_lsb=228",
        "[10:00:03.100] TRAJ_STATE state=4 count=3 total=3 index=3 seq=4 epoch=2",
        "[10:00:04.000] LINK uart=hw baud=115200 rx_overflow=3 rx_error=129 tx_dropped=5 fw=stage2-v7-x short_hb=1",
        "[10:00:05.000] STOP: operator stop; drivers OFF",
    ]
    def verdict(*extra, replace=None):
        lines = [replace.get(l, l) if replace else l for l in base] + list(extra)
        fails, warns, _ = analyse("\n".join(lines))
        return bool(fails), bool(warns)
    assert verdict() == (False, False), "cumulative counters and a pre-ENABLE key gap must not fail"
    assert verdict("[10:00:06] STOP: DEADMAN host heartbeat timeout; position invalid")[0], "protective stop must fail"
    assert verdict(replace={base[6]: base[6].replace("rx_error=129", "rx_error=130")})[0], "rx_error increase must fail"
    assert verdict(replace={base[3]: base[3].replace("key_max_gap_ms=61", "key_max_gap_ms=101")})[0], "gap over the line must fail"
    assert verdict(replace={base[4]: base[4].replace("error_lsb=-47", "error_lsb=-229")})[0], "final error over the limit must fail"
    assert verdict(replace={base[6]: base[6].replace("tx_dropped=5", "tx_dropped=6")}) == (False, True), "tx_dropped increase must only warn"
    assert verdict("[10:00:06] HOST UI lease gap max=208 ms") == (False, True), "long UI lease gap must only warn"
    assert analyse("no firmware lines at all")[1], "missing fw id must warn"
    print("check-log self-test passed")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("paths", nargs="*")
    ap.add_argument("--last", action="store_true")
    ap.add_argument("--all", action="store_true")
    ap.add_argument("--self-test", action="store_true")
    args = ap.parse_args()
    if args.self_test:
        return self_test()
    if args.last or args.all:
        directory = Path(os.environ.get("LOCALAPPDATA", ".")) / "TwoLinkCommissioning" / "Logs"
        logs = sorted(directory.glob("stage1-*.log"))
        args.paths = [str(p) for p in (logs if args.all else logs[-1:])]
    if not args.paths:
        ap.error("give a log path, --last, --all or --self-test")
    ok = True
    for path in args.paths:
        ok &= report(Path(path).name, Path(path).read_text(encoding="utf-8", errors="replace"))
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
