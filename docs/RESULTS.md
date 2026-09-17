# Results & Validation

Everything below was produced by the commands shown, on the committed code, with fixed
seeds. Re-running them regenerates the same numbers — nothing here is hand-copied from an
older run.

Reference machine for the timings: 8-core x86-64 laptop, Linux, CPU only, Python 3.14,
MuJoCo 3.10.0.

---

## 1. Headline acceptance table

Targets are from the [Design Review §7](./Autonomous_Drone_Interceptor_Design_Review.md)
(each already carries a 5% margin). Measured values are the canonical randomized batch:
`--trials 100 --seed 0`.

| KPI | Target | Measured | Verdict |
| :--- | :--- | :--- | :---: |
| **Mission success rate** (interception) | ≥ 90% | **95%** (95/100) | ✅ |
| **Max target speed** intercepted | ≥ 83.6 km/h | **89.7 km/h** | ✅ |
| **Miss distance** `R_miss` | ≤ 1.05 m | **95%** of trials; median hit **0.024 m** | ✅ |
| **Z-axis overshoot** | ≤ 0.5 m | **98%** compliance; median **0.002 m** | ✅ |
| **Time-to-intercept** | static < 10 s, moving < 20 s | **95%** compliance — *every* successful intercept was inside budget | ✅ |
| **Command saturation** | ≤ 5% of flight time | **77%** compliance; median **1.3%** | ⚠️ tail, see §4 |

75 of 100 trials passed *every* KPI simultaneously; the gap between that and the 95%
mission-success figure is almost entirely the saturation tail on very short, very fast
engagements.

## 2. Randomized Monte-Carlo batch

```bash
python scripts/run_montecarlo.py --trials 100 --seed 0 --report --results-dir results/montecarlo
```

~1 min 50 s. The harness samples a seeded 3D threat envelope — engagement geometry in a
frontal cone, a weighted target family, family parameters, and a weighted wind preset —
and flies every draw through the same closed loop the named scenarios use.

**By target family**

| Family | Interception |
| :--- | ---: |
| static | 21/21 (100%) |
| linear | 32/32 (100%) |
| sinusoidal (evasive) | 34/35 (97%) |
| varying-speed (to 90 km/h) | 8/12 (67%) |

**By wind preset**

| Wind | Interception |
| :--- | ---: |
| calm | 58/62 (94%) |
| moderate | 23/23 (100%) |
| gusty | 14/15 (93%) |

Wind robustness is confirmed: interception is flat across calm / moderate / gusty — with
the modeled lumped drag the disturbance is a slow bias the dual-loop controller absorbs.
All five misses in the batch are fast, off-axis `varying_speed`/`sinusoidal` draws (§4),
not wind failures.

Artifacts written by `--report`:

- `results/montecarlo/batch_kpis.csv` — the per-trial dataset (one row per trial: family,
  wind, seed, miss, time, Z-overshoot, saturation, target speed, pass flags).
- `results/montecarlo/batch_manifest.json` — master seed, git hash, KPI targets, committed
  tuning, and every headline verdict.
- `results/montecarlo/batch_distributions.png` — miss histogram, miss-vs-speed scatter with
  the KPI lines, interception-by-family bars, saturation histogram.
- `results/montecarlo/trial_XXXX/` — full run log + config snapshot for each trial.

## 3. Named scenario suites

### Static & linear geometries — 11/11 pass every KPI

```bash
python scripts/run_scenarios.py scenarios/ --report --results-dir results/scenarios
```

~21 s.

| Scenario | Class | R_miss (m) | t_int (s) | Z-over (m) | Sat % | Pass |
| :--- | :--- | ---: | ---: | ---: | ---: | :---: |
| linear_approaching | moving | 0.032 | 4.78 | 0.021 | 1.1 | ✅ |
| linear_climbing | moving | 0.011 | 4.13 | 0.001 | 0.0 | ✅ |
| linear_crossing | moving | 0.016 | 5.23 | 0.004 | 0.3 | ✅ |
| linear_diagonal | moving | 0.008 | 5.62 | 0.005 | 0.2 | ✅ |
| linear_receding | moving | 0.014 | 4.27 | 0.010 | 0.5 | ✅ |
| static_descend | static | 0.013 | 2.28 | 0.000 | 1.3 | ✅ |
| static_diagonal | static | 0.037 | 4.59 | 0.006 | 1.2 | ✅ |
| static_far | static | 0.003 | 7.70 | 0.000 | 0.0 | ✅ |
| static_high | static | 0.004 | 3.71 | 0.001 | 0.0 | ✅ |
| static_lateral | static | 0.013 | 2.23 | 0.037 | 2.7 | ✅ |
| static_near_level | static | 0.009 | 2.71 | 0.004 | 1.5 | ✅ |

### Stress probes: evasive / high-speed / wind — 11/11 intercept, 9/11 pass every KPI

```bash
python scripts/run_scenarios.py scenarios/stress --results-dir results/stress
```

~14 s.

| Scenario | R_miss (m) | t_int (s) | Z-over (m) | Sat % | Max spd (km/h) | Pass |
| :--- | ---: | ---: | ---: | ---: | ---: | :---: |
| sinusoidal_3d_spiral | 0.058 | 2.86 | 0.039 | 0.5 | 13.3 | ✅ |
| sinusoidal_fast_juke | 0.654 | 4.37 | 0.373 | 10.3 | 17.6 | ❌ saturation |
| sinusoidal_lateral_weave | 0.041 | 5.76 | 0.034 | 0.8 | 9.0 | ✅ |
| sinusoidal_vertical_bob | 0.177 | 4.39 | 0.000 | 1.5 | 9.8 | ✅ |
| varying_speed_crossing_84kmh | 0.325 | 2.06 | 0.000 | 5.8 | 78.0 | ❌ saturation |
| varying_speed_headon_90kmh | 0.112 | 2.08 | 0.000 | 3.4 | 72.2 | ✅ |
| varying_speed_quartering_86kmh | 0.127 | 2.11 | 0.000 | 3.0 | 70.3 | ✅ |
| wind_evasive_moderate | 0.079 | 4.81 | 0.000 | 1.3 | 13.8 | ✅ |
| wind_linear_gusty | 0.017 | 4.19 | 0.000 | 1.2 | 7.2 | ✅ |
| wind_static_gusty | 0.032 | 4.76 | 0.011 | 0.9 | 0.0 | ✅ |
| wind_static_moderate | 0.005 | 5.74 | 0.002 | 0.0 | 0.0 | ✅ |

Every stress scenario **intercepts** (all misses ≤ 1.05 m). The two ❌ rows are saturation
breaches, not misses. "Max spd" is the target's speed *at the intercept instant* — the
90 km/h ramps are caught before reaching peak speed, which is the point of leading a
target rather than chasing it.

### A single interception

```bash
python scripts/run_intercept.py --target 8 3 6 --seconds 9
```

~5 s: `min miss distance: 0.037 m at t=5.12 s  [HIT vs R_miss <= 1.05 m]`.

## 4. Known limitations

Reported rather than papered over; each is reproducible from the seed given.

**L1 — Command saturation on sub-2-second high-speed intercepts.** 23% of randomized
trials exceed the 5%-of-flight-time saturation budget. The regime is specific: an
85+ km/h crossing or quartering target engaged **from rest** in ~2 s. The interceptor
still hits, but it spends a large share of that very short flight pinned against the tilt
cap or the mixer's allocation limit. Closing it needs adaptive command authority or
launch-shaping in guidance — a logic change, deliberately out of scope for this
params-only tuning. Note the metric is deliberately strict: saturation counts **limiter ∪
mixer**, so the airframe cannot be pinned while the number stays green.

**L2 — Fast off-axis targets are the physical degradation edge.** The `varying_speed`
family carries 4 of the batch's 5 misses. A from-rest interceptor cannot always lead a
strongly crossing target above ~85 km/h — the geometry, not a defect. The candidate
improvement is a target-acceleration feed-forward (augmented ZEM), which is implemented
but **gated off**: with a relative-state EKF it feeds the interceptor's own maneuver back
as positive feedback, so it stays a candidate rather than a shipped feature.

**L3 — Far-static engagements are tight against the 10 s static budget.** Static targets
beyond ~12 m are intercepted, but the time metric can slip against the deliberately tight
static budget. In the canonical batch every successful intercept was inside budget; the
five `time` failures are exactly the five misses (no intercept ⇒ no time).

## 5. Reproducing everything

```bash
python scripts/check_env.py                                  # environment doctor
pytest                                                       # 221 tests, ~80 s
ruff check src tests scripts                                 # clean

python scripts/run_scenarios.py scenarios/ --report --results-dir results/scenarios
python scripts/run_scenarios.py scenarios/stress --results-dir results/stress
python scripts/run_montecarlo.py --trials 100 --seed 0 --report --results-dir results/montecarlo
```

Determinism is what makes this reproducible: all randomness flows through one seeded RNG
factory, every run writes `run_config.json` (seed + resolved params + git hash), and every
batch writes `batch_manifest.json` (master seed + git hash). Identical seed + identical
config ⇒ byte-identical run log, and it is enforced by a test
(`tests/integration/test_montecarlo_batch.py::test_batch_is_reproducible`).

Small differences in floating-point libraries across machines can move the last digits of
a miss distance; the pass/fail verdicts and headline rates are stable.
