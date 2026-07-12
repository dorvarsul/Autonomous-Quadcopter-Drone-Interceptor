# Phase 3 — Progress Report

> **Status: COMPLETE.** All exit criteria met. Driven as Role 5 (Test, Validation & KPI),
> with two user-approved tuning changes filed to Roles 3/4. This phase builds the
> KPI-measurement + declarative-scenario tooling on top of the Phase 2 closed loop, then
> uses it to bring the system from "functioning" to **"meets spec on static and linear
> targets"**: all four Phase 3 KPIs (`R_miss`, time-to-intercept, Z-overshoot, command
> saturation) pass across a seeded spread of **11** static and linear geometries with OGL.

> **Role-5 discipline (this phase):** Role 5 built the tooling and *measured*; it did not
> hand-edit estimation/guidance/control internals or relax any target. The one systemic KPI
> miss it surfaced (command saturation) was **filed as a finding with a validated,
> params-only fix and confirmed with the user before committing** (AGENTS.md → Workflow).
> No physical constant in `config/constants.py` was changed; the airframe/motor model is
> untouched.

This report maps every Phase 3 task (`docs/phase3.md`, T3.1–T3.9) to what was built, where
it lives, and how its Definition of Done was verified.

---

## How to reproduce the verification

```powershell
.\.venv\Scripts\Activate.ps1
$env:PATH = "C:\Dev\Libraries\mujoco\bin;$env:PATH"   # so mujoco.dll / glfw3.dll resolve

pytest                     # 178 passed (up from 149; +29 Phase 3 tests)
ruff check src tests scripts   # All checks passed

# Run the full seeded KPI suite headless, print the pass/fail table, and write plots:
python scripts/run_scenarios.py scenarios/ --report
#   -> results/phase3/kpi_summary.csv + per-scenario PNGs

# Single scenario (moving target), and the static demo entry point still works:
python scripts/run_scenarios.py scenarios/linear_crossing.yaml
python scripts/run_intercept.py --target 8 3 6      # reuses the new KPI module
```

**Observed results (this machine):**

- `pytest` → **178 passed** in ~11 s. New: 7 KPI unit tests, 9 scenario-parsing unit tests,
  11 parametrized scenario-KPI regressions + a determinism + a b-ablation integration test.
- `ruff check` → **All checks passed**.
- **11/11 scenarios meet every KPI** with the approved tuning (table below). A fixed seed +
  scenario reproduces a **byte-identical** `run_log.csv`.

### Final KPI table (seed 0, OGL, approved tuning)

| Scenario | Class | R_miss (m) | t_int (s) | Z-over (m) | Sat % | Pass |
| :--- | :--- | ---: | ---: | ---: | ---: | :---: |
| static_near_level | static | 0.018 | 2.96 | 0.014 | 4.2 | ✅ |
| static_high | static | 0.007 | 3.95 | 0.005 | 0.2 | ✅ |
| static_lateral | static | 0.013 | 2.75 | 0.441 | 4.9 | ✅ |
| static_diagonal | static | 0.011 | 5.53 | 0.008 | 0.8 | ✅ |
| static_far | static | 0.028 | 8.35 | 0.018 | 0.6 | ✅ |
| static_descend | static | 0.009 | 3.60 | 0.000 | 0.7 | ✅ |
| linear_approaching | moving | 0.059 | 4.74 | 0.041 | 1.5 | ✅ |
| linear_crossing | moving | 0.013 | 5.56 | 0.008 | 0.7 | ✅ |
| linear_receding | moving | 0.019 | 5.82 | 0.015 | 0.7 | ✅ |
| linear_climbing | moving | 0.063 | 4.82 | 0.024 | 1.3 | ✅ |
| linear_diagonal | moving | 0.005 | 5.68 | 0.003 | 0.0 | ✅ |

KPI targets: `R_miss ≤ 1.05 m`, static `t_int < 10 s` / moving `< 20 s`, `Z-over ≤ 0.5 m`,
`Sat ≤ 5 %`.

---

## Exit criteria checklist (from phase3.md)

- [x] KPI module computes every metric in the success table with the 5 % margin (pulled from
      `config/constants.py`).
- [x] Scenario runner executes declarative, seeded YAML configs and logs full config + seed +
      git hash per run.
- [x] **Static** trials meet `R_miss ≤ 1.05 m`, `t_int < 10 s`, `Z-over ≤ 0.5 m`,
      `Sat ≤ 5 %` with OGL (6 geometries).
- [x] **Linear** trials meet `R_miss ≤ 1.05 m`, `t_int < 20 s`, `Z-over ≤ 0.5 m`,
      `Sat ≤ 5 %` with OGL (5 geometries).
- [x] OGL characterized quantitatively on identical seeds, including the `b = 0.1` vs `b = 0`
      ablation on Z-overshoot.
- [x] Passing scenarios captured as reproducible regression tests.

---

## Contract & interface changes (this phase)

Small, additive, and confined to Role 6 wiring / Role 5 measurement — no pipeline boundary
was crossed (guidance still sees estimates, control still sees limited acceleration).

- **`PipelineComponents.build_intercept(...)`** (`pipeline/orchestrator.py`) — a general
  factory that accepts *any* `TargetTrajectory`, so the same closed loop flies static, linear,
  and (Phase 4) evasive targets. `phase2_intercept` is now a thin static-target wrapper over
  it, so all existing callers/tests are unchanged (Open/Closed).
- **`StubOrchestrator.run(..., extra_metadata=None)`** — lets the scenario runner persist the
  scenario name + resolved spec into the reproducibility snapshot alongside the params/seed/
  git-hash the orchestrator already wrote.
- **Z-overshoot metric definition** — "altitude above target" is measured *sign-aware*: for
  the usual climbing intercept it is how far the interceptor rises **above** the target; for a
  descending intercept (interceptor starting above) it is how far it sinks **below**. This
  fixes a metric artifact where a descending engagement counted its benign initial altitude
  gap as 3.4 m of "overshoot" (`analysis/kpis.py`).

---

## Task-by-task

### T3.1 — KPI measurement module ✅  (Role 5)
- **`src/interceptor/analysis/kpis.py`** — `load_run_trace` (typed view over `run_log.csv`) +
  `compute_kpis` → a frozen `KpiRecord`: miss distance (min range), time-to-intercept (first
  frame within `R_MISS_MAX_M`, `inf` if never), sign-aware Z-overshoot (to closest approach),
  command-saturation fraction (over the terminated engagement), max target speed (finite
  difference), and per-KPI pass/fail + overall `success`. Every threshold is pulled from
  `config/constants.py` (no inline KPI numbers). The static/moving `t_int` bound is supplied
  by the caller. `scripts/run_intercept.py` now reuses this module, removing its duplicated
  inline min-range logic (DRY).
- **DoD:** KPIs verified against **hand-computed** synthetic traces — a hit with known
  overshoot + saturation, a no-intercept (t_int = ∞), finite-difference target speed, the
  post-intercept overshoot exclusion, the descending-intercept case, the "1"/"0" boolean
  parsing, and fail-loud on an empty log (`test_kpis.py`, 7 tests). ✔

### T3.2 — Scenario runner & declarative configs ✅  (Role 5)
- **`src/interceptor/analysis/scenarios.py`** — a `Scenario` dataclass parsed from a YAML
  schema (`name`, `seed`, `interceptor.start_m`, `target.type` + params, `time_limit_s`,
  `guidance_law`, optional `params` override, `target_class`). `build_trajectory` maps the
  `target` block onto the existing Phase 1 generator families (reuse only); params overrides
  go through the same `_merge_into` deep-merge `load_params` uses (DRY). `run_scenario` flies
  it via `build_intercept`, terminates at closest approach, and measures KPIs; `run_suite`
  batches a directory headlessly in deterministic order. Fails loud on unknown trajectory
  types, missing keys, a non-OGL law, or a malformed vector.
- **`scenarios/*.yaml`** — the declarative library (below).
- **DoD:** parsing/trajectory-build/override-merge/fail-loud covered in `test_scenarios.py`
  (9 tests); re-running a scenario yields a **byte-identical** log
  (`test_scenario_suite.py::test_scenario_run_is_deterministic`). ✔

### T3.3 — Static-target trials ✅  (Role 5 measure → Role 3/4 tune)
- **`scenarios/static_*.yaml`** — 6 geometries spanning short/level, steep climb, cross-range,
  3D-diagonal, long-range, and a descending intercept (interceptor starting *above* the
  target). All meet the static KPIs with OGL (table above).
- **DoD:** static KPIs met across the spread; the descending case exercised (and drove) the
  sign-aware Z-overshoot fix. ✔

### T3.4 — Linear moving-target trials ✅  (Role 5 measure → Role 3/4 tune)
- **`scenarios/linear_*.yaml`** — 5 constant-velocity geometries: crossing, tail-chase
  (receding), head-on (approaching), climbing, and 3D-diagonal, at slow/moderate speeds. All
  intercept in ≤ 5.8 s (well under the 20 s moving KPI) with saturation ≤ 1.5 %.
- **DoD:** linear KPIs met (`t_int < 20 s`, others within target) with OGL. ✔

### T3.5 — EKF tuning ✅ (no change needed)  (Role 2)
- Measured outcome: the EKF's estimate quality is **not** the bottleneck on the static/linear
  suite — miss distances are 0.005–0.063 m against the 0.30 m range-noise / 0.02 s latency
  profile, i.e. guidance is fed a clean, lag-compensated LOS. No `Q`/`R` change was required
  or made; the Phase 2 9-state constant-acceleration filter stands.
- **DoD:** estimate error low enough that guidance is not the limiting factor; documented. ✔

### T3.6 — Guidance tuning ✅ (user-approved)  (Role 3)
- **`GuidanceParams.reference_closing_speed_m_s`: 5.0 → 3.5 m/s** (`config/params.py`). Role 5
  found command saturation exceeded the 5 % KPI on 9/11 scenarios (6–11 %), and pinpointed the
  cause as the **from-rest launch transient**: with no true closing speed at rest, OGL
  synthesizes `t_go` from this reference, and at 5 m/s the launch command over-drove the tilt
  bound and saturated the first ~20 % of frames (mid-flight saturation was 0 %). Softening the
  reference cuts that transient. `3.5` was chosen over `2.5` because `2.5` slowed the farthest
  static target (12.4 m) past its 10 s budget — a soft launch and a fast far-range intercept
  trade off under a fixed reference speed.
- The time-varying `N'(t_go/T)` schedule and `b`/`T` were left at their Phase 2 values (the
  ablation, T3.8, showed no `b` retune was warranted on this suite).
- **DoD:** tuning documented; the saturation improvement is attributable to this specific
  change. Confirmed with the user before committing (AGENTS.md → Workflow). ✔

### T3.7 — Control / limiter tuning ✅ (user-approved)  (Role 4)
- **`LimiterParams.max_tilt_rad`: 0.6109 (≈35°) → 0.7854 (45°)** (`config/params.py`). The
  horizontal acceleration authority is `g·tan(max_tilt)`, so 35° capped it at ~6.87 m/s² and
  the cross-range dashes clamped against it. 45° raises the authority to `g` = 9.81 m/s²,
  which (paired with the softened launch) keeps the aggressive geometries — notably the pure
  cross-range `static_lateral` — inside the ≤ 5 % saturation KPI while remaining conservative
  for a quadrotor. Inner/outer PID gains were left unchanged (stable, no oscillation observed).
- **DoD:** command saturation ≤ 5 % on every static/linear scenario; no instability. Confirmed
  with the user before committing. ✔

### T3.8 — OGL characterization & `b`-penalty ablation ✅  (Role 5)
- **`src/interceptor/analysis/reporting.py`** (matplotlib **Agg**, headless) — writes the KPI
  summary (`kpi_summary.csv` + a Markdown table) and per-scenario diagnostic plots (X-Y
  geometry, altitude-vs-time with the Z-overshoot band, range + command-effort with saturated
  frames shaded), plus a b=0.1-vs-b=0 altitude-overlay ablation plot. Artifacts land in
  `results/phase3/`.
- **Ablation finding (reported faithfully):** on this control architecture the b-penalty's
  effect on Z-overshoot is **negligible** — b=0.1 vs b=0 differ by < 0.005 m on the diagonal
  and steep-climb geometries, and even a purpose-built near-vertical climb overshoots < 0.002 m
  regardless of `b` (swept 0 → 1.0). The differential-flatness outer loop (`f = a_cmd + g·ẑ`)
  converges to the target altitude *without* the altitude overshoot the rejected PN/APN
  baselines exhibited — which is the overshoot the Design Review's `b` penalty was introduced
  to tame. The only measurable effect of a larger `b` here is a *slight slowing* of vertical
  closing (higher `t_int`). `b` is therefore retained at its default 0.1 as cheap insurance
  and to keep the OGL cost faithful to the Design Review, with its role to be re-evaluated
  under Phase 4's higher-rate evasive/high-speed geometries.
- **DoD:** a reproducible report (`results/phase3/`) quantifies OGL performance and the
  b-penalty effect; PN/APN cited as documentary reference only (not implemented). ✔

### T3.9 — Regression test suite ✅  (Role 5)
- **`tests/integration/test_scenario_suite.py`** (marked `mujoco`) — parametrizes over every
  `scenarios/*.yaml`, flies it through the real loop, and asserts each KPI **separately** so a
  regression pinpoints the offending metric. Plus a scenario-level determinism test and a
  guard that the b=0 ablation control still intercepts. Locks the user-approved tuning.
- **DoD:** suite runs headless and green; failures name the failing KPI. ✔

---

## Deliverables produced

- `analysis/{kpis,scenarios,reporting}.py`; `scripts/run_scenarios.py`.
- `scenarios/`: 6 static + 5 linear declarative YAMLs, plus `scenarios/ablation/` (2 × b=0
  controls).
- Tuned `config/params.py`: `reference_closing_speed_m_s` 5.0→3.5 (T3.6),
  `max_tilt_rad` 0.6109→0.7854 (T3.7), each with an inline rationale referencing this report.
- `pipeline/orchestrator.py`: `build_intercept` factory + `extra_metadata` snapshotting.
- `analysis/kpis.py`: sign-aware Z-overshoot metric; `scripts/run_intercept.py` reuses it.
- OGL characterization + b-ablation report + plots in `results/phase3/`.
- New tests: `tests/unit/test_kpis.py`, `tests/unit/test_scenarios.py`,
  `tests/integration/test_scenario_suite.py` (**+29** tests; 149 → 178).

## Notes, decisions & deviations

- **Saturation was a launch transient, not a terminal spike.** Per-third analysis showed
  saturation concentrated in the *first* ~20 % of frames (from rest) with 0 % mid-flight —
  redirecting the fix from the terminal `1/t_go²` term (raising the `t_go` floor actually made
  it *worse*) to the from-rest launch shaping. This is why `reference_closing_speed` was the
  effective lever.
- **The far-range/soft-launch trade-off is real.** No single `reference_closing_speed` both
  keeps the near geometries under 5 % saturation *and* the 12.4 m static target under 10 s;
  raising the tilt authority (45°) was what dissolved the tension, letting a higher reference
  speed pass both. This is documented rather than papered over.
- **The b-penalty ablation returned a null result — reported as such.** Rather than manufacture
  a `b` effect, T3.8 documents that the flatness-based controller doesn't overshoot altitude on
  the static/linear suite, so `b` has little to do. This is the faithful Role 5 finding.
- **Augmented ZEM still gated off; OGL still the sole law.** Unchanged from Phase 2; the
  relative-state target-acceleration feed-forward remains Phase 4 work for evasive targets.

## Ready for Phase 4

The KPI + scenario tooling now measures any engagement declaratively and reproducibly, and the
static/linear suite is locked green as regressions. Phase 4 can reuse the exact same runner for
the **sinusoidal (evasive)**, **varying-speed (≥ 83.6 km/h)**, and **wind/gust** families
(already wired in `build_trajectory`), add the `≥ 90 %` randomized-trial mission-success KPI,
and revisit the augmented-ZEM target-acceleration term and the `b`-penalty under the higher
vertical rates those targets impose.
