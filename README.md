# ROAR_Competition

Submission for ROAR Competition Summer 2026

# ROAR Simulation Racing Series — Summer 2026

Monza Map v1.1 · Best clean time **320.19s** (3 laps, zero respawns)

---

## Provenance

This solution uses code from University High School's Spring 2026 first-place submission:
**https://github.com/advaybansal/roar_feb**

That solution was itself built on the pure-pursuit baseline provided by the
competition, which completed three laps in 321.0s. The lineage is:

| Stage | Time | Files changed |
|---|---|---|
| Competition-provided pure pursuit baseline | 321.0s | — |
| University HS, Spring 2026 winner | 320.4s | `ThrottleController.py`, `submission.py` |
| This fork | 320.19s | `ThrottleController.py`, `submission.py` |

Inherited from the baseline and not my work: the pure-pursuit lateral
controller, the racing line (`waypointsPrimary.npz`, `WaypointLine.py`), the
ten-section track division, and the per-section friction coefficient table.

Reuse of the prior solution was disclosed to and approved by the ROAR
organizers before submission.

---

## My contributions

### 1. Respawn recovery — `submission.py`

The competition runner teleports the car to the spawn point after a major
collision and zeroes its own lap progress, but leaves the solution's internal
state untouched. In the original code this meant that after any crash:

- `WaypointLine.prev_index` was stale, and `find_closest_index()` only searches
  ±50 indices around it — so every snap-to-line steering target became garbage
- `current_section` retained its pre-crash value, applying the wrong
  `steerMultiplier` indefinitely

The result was an unrecoverable death spiral. One measured run produced
**13 respawns with a perfectly periodic 84-tick crash loop** — the car
respawned, drove 76m, hit the same wall at the same speed, and repeated.

`FIX_RESPAWN_RESET` detects a single-tick position delta above 20m and
re-seeds `current_waypoint_idx`, `current_section`, `waypoint_line.prev_index`,
`previous_brake` and `s3_mult`. It scans the full racing line for the closest
index rather than calling the windowed `find_closest_index()`, which has a
latent bug: on failure it returns a *location* where the caller expects an
integer index.

One subtlety worth recording: `previous_location` must be stored with
`np.array(..., copy=True)`. The location sensor returns the same underlying
buffer each tick, so storing a reference makes the computed delta identically
zero and the detection silently never fires. The original `total_dist`
accumulator has this bug and reads zero.

**Validated:** after a crash, the car recovered and completed three clean laps
with no further teleports.

### 2. Section detection by index range — `submission.py`

Section transitions were detected with a proximity window:

```python
if abs(self.current_waypoint_idx - section_ind) <= 2:
```

Waypoints average 2.12m apart. At 159 km/h the car covers 2.21m per tick —
**1.04 waypoints per tick** — and `filter_waypoints()` returns the *first*
waypoint within 3m scanning forward, so the index can step past the 5-wide
window entirely. When that happens the section never updates and the previous
section's steering multiplier stays applied.

This is not hypothetical. Section 4 uses `steerMultiplier * 1.65`; section 5
uses `* 1.1`. Two separate runs crashed at waypoints 1313 and 1315 — both
within four waypoints of the section 4→5 boundary at 1317, both at ~160 km/h.

`FIX_SECTION_RANGE` replaces the window with a range lookup mapping any index
to its owning section, including the wrap past the lap boundary. Verified
against all ten section starts. Cost on clean laps: ~0.2s.

### 3. Trail braking — `ThrottleController.py`, branch `trail-braking`

*Not enabled in the submitted configuration. See "Findings" below.*

The original brake signal is binary — telemetry across a full run contains
only the values 0.0 and 1.0, never anything between. Under full braking
essentially the whole grip budget goes to deceleration, so the car cannot turn
while slowing and must finish braking before turn-in.

Trail braking scales brake pressure by how much excess speed remains:

```
excess = current_speed / recommended_speed_now - 1
brake  = clamp(excess / TRAIL_GAIN, TRAIL_BRAKE_MIN, 1.0)
```

With `TRAIL_GAIN = 0.12` and `TRAIL_BRAKE_MIN = 0.45`: 12% over target gives
full brake, 6% over gives 0.5, at target gives the floor. Post-change telemetry
shows 49 distinct brake values spanning 0.45–1.0.

The existing throttle-during-braking recovery is suppressed while brake is
modulated, since feeding throttle in against reduced brake pressure
under-decelerates the car into the corner.

---

## Findings

### Bang-bang braking caps how far corner speeds can be tuned

Advay's writeup notes that several sections were left at the conservative
default of μ = 2.75 but does not explain why raising them fails. A controlled
experiment identifies the mechanism.

Section 5 was set to μ = 3.00, changing only the braking mode between runs:

| Braking | μ (section 5) | Result |
|---|---|---|
| Bang-bang | 3.00 | **Crashed at waypoint 1338**, 45m inside the corner — 592s |
| Trail braking | 3.00 | **Clean** — 320.80s |

Under bang-bang braking the car commits its entire grip budget to deceleration
before turn-in; arriving too fast leaves nothing in reserve. Trail braking
keeps grip available through the corner and the same μ becomes survivable.

This is consistent with the reference solution's remaining untuned sections:
they were left at default because, with binary braking, raising them crashed.

### Trail braking is an enabler, not a gain by itself

Applied without raising μ, trail braking is time-neutral (320.65s vs a
320.50s baseline). The controller is a feedback loop targeting
`√(μ·g·r)`; softer braking means the car takes more ticks to reach the same
target speed. It changes *how* the corner speed is reached, not what it is.
Its value is that it permits higher targets.

### The car is engine-limited on straights, not target-limited

Telemetry across a full run: **94.2% of ticks at full throttle, 0.0s of
coasting, maximum speed 257.6 km/h against a `max_speed` setting of 305.**
There is no time available on the straights, and `max_speed` is not a binding
parameter. All remaining time is in braking and corner entry.

### Achieved grip runs at 75–80% of configured μ

Lateral acceleration derived from the position trace (`a_lat = v·ω`, μ = a_lat/g)
lands consistently at 0.75–0.80 of each section's configured μ. The offset is
systematic, so the ratio is the useful signal.

Section 9 is the outlier: configured at 2.10 but achieving **2.58** — the only
section exceeding its own setting. It is also the slowest corner (87 km/h) and
the third-largest time pool at 10.2s/lap. Something other than the speed target
binds there. Unresolved, and the most promising lead for future work.

---

## Results

| Configuration | Time | Notes |
|---|---|---|
| Reference solution, as inherited | 320.35s | Reproduced |
| **+ respawn recovery + section fix** | **320.19s** | **Submitted** |
| + trail braking (μ unchanged) | 320.65s | Time-neutral |
| + trail braking, section 5 μ=3.00 | 320.80s | Clean; bang-bang crashed |
| Section 5 μ=3.00, bang-bang | 592s | Crashed at waypoint 1338 |

Six clean runs on the submitted configuration, spread ≈0.5s. Laps 1 and 2 are
frequently tick-identical across runs; the simulator is close to deterministic
when no collision occurs.

---

## Known limitations

**Respawn recovery is not always sufficient.** The runner's spawn point sits
~7.3m off the racing line, and section 0's steering authority is
`steerMultiplier = speed/120` — about 0.67 at recovery speeds. That is weak
correction for a 7m lateral offset before the track turns at waypoint 5.
Recovery succeeds in most cases but can fail if the car cannot rejoin the line
in time. This limitation predates this fork; the original code fails worse in
the same situation, with no recovery at all.

**Crash rate ≈20% across all configurations tested**, concentrated at waypoint
470 (section 1) and waypoints 1313–1315 (section 4→5 boundary). Both corners
are marginal independent of the changes here. A crash costs 170–270s because
`respawn()` zeroes `furthest_waypoints_index`, forcing three laps to be
re-driven — so reliability dominates lap-time tuning in expected value.

---

## Tooling

`analyze_run.py` reads the per-tick JSON telemetry in `debugData/` and reports
teleport events with timestamps and coordinates, crawl segments, distance
driven, and per-lap tick counts. Teleport coordinates were mapped back to
waypoint indices and sections to identify crash sites.

Note that it reports tick count × 0.05s. The runner reports accumulated
`world.last_tick_elapsed_seconds`, which is marginally lower because physics
steps are 0.005s and the final tick lands mid-step. Quote the runner's figure.

---

## Running

Start the CARLA Monza v1.1 server, confirm exactly one `CarlaUE4.exe` is
running, then:

```
cd competition_code
python competition_runner.py
```

Set `useDebug = True` in `submission.py` to write telemetry. The save runs in
an `atexit` hook — closing the terminal skips it; close the pygame viewer
instead.
