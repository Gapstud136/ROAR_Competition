# ROAR Simulation Racing Series — Summer 2026

Monza Map v1.1 · Best clean time **320.19s** (3 laps, zero respawns)

---

## Overview

This submission adds three things to the inherited solution: a recovery system
for post-collision respawns, a fix to section-boundary detection, and a
continuous (trail) braking controller replacing binary brake control.

Alongside the code, this repository documents a diagnostic finding that
explains the solution's dominant failure mode — crashes at Lesmo are caused by
discontinuous control branches amplifying simulator nondeterminism, not by
exceeding available grip.

---

## Contributions

### 1. Respawn recovery — `submission.py`

The competition runner teleports the car to the spawn point after a major
collision and zeroes its own lap progress, but leaves the solution's internal
state untouched. In the inherited code this meant that after any crash:

- `WaypointLine.prev_index` was stale, and `find_closest_index()` searches only
  ±50 indices around it — so every snap-to-line steering target was garbage
- `current_section` kept its pre-crash value, applying the wrong
  `steerMultiplier` indefinitely

The result was an unrecoverable death spiral. One measured run produced
**13 respawns with a perfectly periodic 84-tick crash loop**: respawn, drive
76m, hit the same wall at the same speed, repeat.

`FIX_RESPAWN_RESET` detects a single-tick position delta above 20m and
re-seeds `current_waypoint_idx`, `current_section`, `waypoint_line.prev_index`,
`previous_brake` and `s3_mult`. It scans the full racing line for the nearest
index rather than calling the windowed `find_closest_index()`, which has a
latent bug: on failure it returns a *location* where the caller expects an
integer index.

One subtlety worth recording: `previous_location` must be stored with
`np.array(..., copy=True)`. The location sensor returns the same underlying
buffer each tick, so storing a reference makes the computed delta identically
zero and detection silently never fires. The inherited `total_dist`
accumulator has this bug and reads zero.

**Validated:** in one run the car crashed, recovered, and completed three
clean laps at normal pace (2094 and 2100 ticks) with no further teleports.

### 2. Section detection by index range — `submission.py`

Section transitions were detected with a proximity window:

```python
if abs(self.current_waypoint_idx - section_ind) <= 2:
```

Waypoints average 2.12m apart. At 159 km/h the car covers 2.21m per tick —
**1.04 waypoints per tick** — and `filter_waypoints()` returns the *first*
waypoint within 3m scanning forward, so the index can step past the 5-wide
window entirely. The section then never updates and the previous section's
steering multiplier stays applied.

Section 4 uses `steerMultiplier * 1.65`; section 5 uses `* 1.1`. Two runs
crashed at waypoints 1313 and 1315 — both within four waypoints of the
section 4→5 boundary at 1317, both at ~160 km/h.

`FIX_SECTION_RANGE` replaces the window with a range lookup mapping any index
to its owning section, including the wrap past the lap boundary. Verified
against all ten section starts. Cost on clean laps: ~0.2s.

### 3. Trail braking — `ThrottleController.py`, branch `robustness`

*Implemented and tested; not enabled in the submitted configuration.*

The inherited brake signal is binary — telemetry across a full run contains
only 0.0 and 1.0. Under full braking essentially the whole grip budget goes to
deceleration, so the car cannot turn while slowing and must finish braking
before turn-in.

Brake pressure is instead scaled by remaining excess speed:

```
excess = current_speed / recommended_speed_now - 1
brake  = clamp(excess / TRAIL_GAIN, TRAIL_BRAKE_MIN, 1.0)
```

With `TRAIL_GAIN = 0.12` and `TRAIL_BRAKE_MIN = 0.45`: 12% over target gives
full brake, 6% over gives 0.5, at target gives the floor. Post-change
telemetry shows 49 distinct brake values spanning 0.45–1.0.

The inherited throttle-during-braking recovery is suppressed while brake is
modulated, since feeding throttle against reduced brake pressure
under-decelerates the car into the corner.

---

## Findings

### Discontinuous control amplifies simulator nondeterminism into crashes

This is the main diagnostic result, and it explains the dominant failure mode.

Two runs of identical code were compared tick by tick — one crashed at Lesmo
on lap 1, one completed cleanly. **They are bit-identical through tick 503**
(position deviation 0.003m). Speed then drifts by fractions of a km/h, which
is expected: CARLA's PhysX solver is multithreaded, so contact-batch summation
order varies between runs.

Normally that is irrelevant. Here it is not:

| Tick 509 | Speed | Brake |
|---|---|---|
| Crashed run | 181.1 km/h | **0.0** |
| Clean run | 183.0 km/h | **0.6** |

A **1.9 km/h** difference flipped a discrete branch in
`speed_data_to_throttle_and_brake`, producing a step change in brake pressure.
At tick 521 a second flip: brake 0.1 against 1.0. By tick 530 the crashed run
carried 1.6 km/h more and was steering −0.065 against −0.047. Lateral
separation reached **0.66m** by tick 550, and the car hit the wall at 554.

The control law is a tree of hard thresholds, so it amplifies numerical noise
rather than damping it. Trail braking makes the brake channel continuous —
a 1.9 km/h difference produces a slightly different pressure, not a binary
flip — which is the motivation for contribution 3 above.

### Lesmo failures are lateral, not grip-limited

Crashes at waypoint 472 occurred at 36, 37, 44, 86, 119, 120, 123, 123 and
127 km/h. The clean baseline passes at 119–129 km/h **with zero braking across
all 47 ticks** in that window. A car that hits the wall at 36 km/h in a corner
the model targets at 138 km/h is not exceeding traction — it is in the wrong
place laterally. The inherited racing line runs under a metre from the inside
wall there, so sub-metre error is sufficient.

This is why lowering μ does not help: at waypoint 472 the radius is 84m, the
μ=3.0 target is 179 km/h, and the car is only doing 130. It is still
accelerating out of the previous braking zone and never reaches the target,
so the μ formula does not set its speed there.

### The friction table is at a local optimum

Systematic testing found no headroom in the sections carrying the most time.

**Section 9** (10.2s/lap, the slowest corner at 87 km/h) is the only section
achieving more grip than configured — 2.58 against a set 2.10.

| Section 9 μ | Result |
|---|---|
| 2.10 | 320.85s clean |
| 2.30 | 321.50s clean, **slower** |
| 2.50 | **Crashes at waypoint 2596**, deterministic |

Waypoint 2596 has a 29m radius, the tightest on the track. At waypoint 2569
the car already runs 104 km/h against a 90 km/h target, so raising μ means
braking even less into a corner it is already entering over the model's limit.
The inherited 2.10 is calibrated, not conservative.

**Section 5** behaves the same way, and the braking mode is what decides it:

| Section 5 μ | Braking | Result |
|---|---|---|
| 2.75 | bang-bang | 320.50s clean |
| 3.00 | bang-bang | **Crashes at waypoint 1338**, 45m into the corner — 592s |
| 3.00 | trail | 320.80s clean |

Same μ, same corner, opposite outcome. Under bang-bang braking the car commits
its entire grip budget to deceleration before turn-in and has nothing in
reserve; trail braking keeps grip available through the corner.

### There is no time available on the straights

Telemetry across a full run: **94.2% of ticks at full throttle, 0.0s of
coasting, maximum speed 257.6 km/h against a `max_speed` setting of 305.**
The car is engine-limited, not target-limited, so `max_speed` is not a binding
parameter and sections 3, 8 and 0 cannot be improved by speed targets. All
remaining time is in braking and corner entry.

### Trail braking is an enabler, not a gain by itself

Applied without raising μ, trail braking is time-neutral to slightly slower
(320.85s against 320.19s). The controller is a feedback loop targeting
`√(μ·g·r)`; softer braking means more ticks to reach the same target speed.
It changes *how* the corner speed is reached, not what it is. Its value is
that it permits higher targets and smooths the control discontinuity.

---

## Results

| Configuration | Best clean time | Notes |
|---|---|---|
| **Respawn recovery + section fix** | **320.19s** | **Submitted** |
| + trail braking, all sections | 320.85s | Crash rate not established |
| + trail braking, section 9 μ=2.3 | 321.50s | Slower |
| + trail braking, section 9 μ=2.5 | — | Crashes at waypoint 2596 |
| Section 5 μ=3.00, bang-bang | 592s | Crashes at waypoint 1338 |

Clean times on the submitted configuration across eight runs: 320.19, 320.20,
320.30, 320.35, 320.45, 320.45, 320.50, 320.60. Laps 1 and 2 are frequently
tick-identical between runs; the simulator is near-deterministic absent a
collision.

---

## Known limitations

**Crash rate ≈38%** (3 of 8 runs), concentrated at waypoint 472 (Lesmo,
section 1) and waypoints 1313–1315 (section 4→5 boundary). Because
`respawn()` zeroes `furthest_waypoints_index`, a crash forces three laps to be
re-driven and costs 170–270s — so under single-run scoring, reliability
dominates lap-time tuning in expected value. Trail braking is the candidate
mitigation but was not validated to significance before the deadline.

**Respawn recovery is not always sufficient.** The spawn point sits ~7.3m off
the racing line, and section 0's steering authority is
`steerMultiplier = speed/120` — about 0.67 at recovery speeds. That is weak
correction for a 7m lateral offset before the track turns at waypoint 5.
Recovery succeeds in most cases but can fail if the car cannot rejoin the line
in time.

---

## Tooling

`analyze_run.py` reads per-tick JSON telemetry from `debugData/` and reports
teleport events with timestamps and coordinates, crawl segments, distance
driven, and per-lap tick counts. Teleport coordinates were mapped back to
waypoint indices, sections and local track radius to identify crash sites; the
tick-level run comparison in Findings was built on the same data.

Note it reports tick count × 0.05s, while the runner reports accumulated
`world.last_tick_elapsed_seconds`. The latter is marginally lower because
physics steps are 0.005s and the final tick lands mid-step. Quote the runner's
figure.

---

## Attribution

Built on University High School's Spring 2026 ROAR submission
(https://github.com/advaybansal/roar_feb), which was itself developed from the
pure-pursuit baseline provided by the competition.

Inherited and unmodified: the pure-pursuit lateral controller, the racing line
(`waypointsPrimary.npz`, `WaypointLine.py`), the ten-section track division,
and the per-section friction coefficient table. Reuse was disclosed to and
approved by the ROAR organizers before submission.

---

## Running

Start the CARLA Monza v1.1 server, confirm exactly one `CarlaUE4.exe` process,
then:

```
cd competition_code
python competition_runner.py
```

Set `useDebug = True` in `submission.py` to write telemetry. The save runs in
an `atexit` hook — closing the terminal skips it; close the pygame viewer
window instead.
