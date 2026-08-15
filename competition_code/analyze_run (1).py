"""
analyze_run.py -- diagnose a ROAR Monza run from debugData.json

Usage:
    python analyze_run.py <your_debugData.json> [baseline_debugData.json]

Put it in competition_code/ and run:
    python analyze_run.py debugData/debugData.json

Every tick is 0.05 s of SIMULATED time (world.set_control_steps(0.05, 0.005)),
so total_ticks * 0.05 == the number the competition scores you on.
"""

import json
import sys
import numpy as np

TICK_DT = 0.05
RESPAWN_JUMP_M = 25.0   # a position delta bigger than this in one tick == teleport
STUCK_KMH = 20.0


def load(path):
    with open(path) as f:
        d = json.load(f)
    ks = sorted(int(k) for k in d)
    loc = np.array([d[str(k)]["loc"] for k in ks], dtype=float)
    spd = np.array([d[str(k)]["speed"] for k in ks], dtype=float)
    thr = np.array([d[str(k)]["throttle"] for k in ks], dtype=float)
    brk = np.array([d[str(k)]["brake"] for k in ks], dtype=float)
    lap = np.array([d[str(k)]["lap"] for k in ks], dtype=int)
    return dict(t=np.array(ks), loc=loc, spd=spd, thr=thr, brk=brk, lap=lap)


def report(r, name):
    n = len(r["t"])
    print(f"\n=== {name} ===")
    print(f"ticks            {n}")
    print(f"SIM TIME         {n * TICK_DT:.2f} s")
    if n == 0:
        print("\n  !! EMPTY -- the race loop never ran a single tick.")
        print("  The atexit hook still fires on a crash, so it wrote '{}'.")
        print("  Almost always: CARLA wasn't up when the client connected")
        print("  (RuntimeError: time-out of 5000ms), or startup threw.")
        print("  Check the .log from that run for the traceback.")
        return None
    print(f"speed  mean {r['spd'].mean():6.1f}   max {r['spd'].max():6.1f} km/h")

    # ---- teleports (respawns) -------------------------------------------
    step = np.linalg.norm(np.diff(r["loc"], axis=0), axis=1)
    jumps = np.where(step > RESPAWN_JUMP_M)[0]
    print(f"\nRESPAWNS / teleports: {len(jumps)}")
    for j in jumps:
        print(f"  tick {r['t'][j]:6d} ({r['t'][j]*TICK_DT:7.1f}s)  "
              f"({r['loc'][j][0]:8.1f},{r['loc'][j][1]:8.1f}) -> "
              f"({r['loc'][j+1][0]:8.1f},{r['loc'][j+1][1]:8.1f})  "
              f"jump {step[j]:.0f} m   speed before {r['spd'][j]:.0f} km/h")

    # ---- crawling -------------------------------------------------------
    slow = r["spd"] < STUCK_KMH
    print(f"\nticks under {STUCK_KMH:.0f} km/h: {slow.sum()} "
          f"({slow.mean()*100:.1f}%) = {slow.sum()*TICK_DT:.1f} s lost")
    # contiguous slow runs longer than 1 s
    runs, start = [], None
    for i, s in enumerate(slow):
        if s and start is None:
            start = i
        elif not s and start is not None:
            if i - start > 20:
                runs.append((start, i))
            start = None
    for a, b in runs[:15]:
        print(f"  slow {r['t'][a]*TICK_DT:7.1f}s -> {r['t'][b]*TICK_DT:7.1f}s "
              f"({(b-a)*TICK_DT:5.1f}s)  at ({r['loc'][a][0]:.0f},{r['loc'][a][1]:.0f})")

    # ---- distance / laps ------------------------------------------------
    dist = step[step < RESPAWN_JUMP_M].sum()
    print(f"\ndistance driven  {dist/1000:.2f} km   (3 clean laps ~= 17.4 km)")
    for l in np.unique(r["lap"]):
        m = r["lap"] == l
        print(f"  lap counter {l}: {m.sum()} ticks = {m.sum()*TICK_DT:6.1f} s")
    return dict(jumps=jumps, dist=dist)


def compare(a, b):
    """How far off the baseline path did we get, per unit of track progress?"""
    print("\n=== path divergence vs baseline ===")
    tree = b["loc"]
    idx = np.linspace(0, len(a["loc"]) - 1, min(400, len(a["loc"]))).astype(int)
    devs = []
    for i in idx:
        d = np.linalg.norm(tree - a["loc"][i], axis=1).min()
        devs.append(d)
    devs = np.array(devs)
    print(f"median off-baseline-path distance {np.median(devs):.2f} m, "
          f"90th pct {np.percentile(devs, 90):.2f} m, max {devs.max():.2f} m")
    bad = idx[devs > 8]
    if len(bad):
        print(f"first big departure at tick {a['t'][bad[0]]} "
              f"({a['t'][bad[0]]*TICK_DT:.1f}s) loc "
              f"({a['loc'][bad[0]][0]:.0f},{a['loc'][bad[0]][1]:.0f})")
    else:
        print("car stayed on the reference path the whole run")


if __name__ == "__main__":
    mine = load(sys.argv[1])
    report(mine, sys.argv[1])
    if len(sys.argv) > 2:
        base = load(sys.argv[2])
        report(base, sys.argv[2] + "  (BASELINE)")
        compare(mine, base)
