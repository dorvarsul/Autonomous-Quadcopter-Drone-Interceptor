# Phase 3 — Tuning & Static / Linear Trials

> **Roadmap window:** Jul 16 – Aug 5.
> **Primary role:** Role 5 — Test, Validation & KPI Engineer (drives); owning roles
> (2/3/4) perform the actual tuning of their own layers when a finding is filed.
> **Pipeline stages touched:** All (measured end-to-end); no new stages added.
> **Goal:** Build the KPI-measurement and scenario-running tooling, then bring the
> system from "functioning" (Phase 2) to "meets spec on static and linear targets."
> Characterize OGL quantitatively (including the `b`-penalty ablation) against the KPI
> targets on identical seeded scenarios. Lock passing results as regression tests.

> **Role-5 boundary:** Role 5 measures and **files findings**; it does **not** edit
> estimation/guidance/control logic to force a pass. Tuning is performed by the owning
> role, and scenarios/targets are **never** relaxed to manufacture a pass.

---

## Entry Criteria

- Phase 2 complete: full pipeline wired; OGL is the sole guidance law; static
  interception works headless and deterministically.

## Exit Criteria (Definition of Done for the phase)

- [ ] KPI module computes every metric in the success table with the 5% margin.
- [ ] Scenario runner executes declarative, seeded configs and logs full config + seed
      + git hash per run.
- [ ] **Static** trials meet: `R_miss ≤ 1.05 m`, `t_int < 10 s`, `Z-overshoot ≤ 0.5 m`,
      `saturation ≤ 5%` with OGL.
- [ ] **Linear** trials meet: `R_miss ≤ 1.05 m`, `t_int < 20 s`, `Z-overshoot ≤ 0.5 m`,
      `saturation ≤ 5%` with OGL.
- [ ] OGL's behavior is documented quantitatively on identical seeds (interception time,
      overshoot, effort), including the `b = 0.1` vs `b = 0` ablation on Z-overshoot.
- [ ] Passing scenarios are captured as reproducible regression tests.

---

## Tasks

### T3.1 — KPI measurement module
**Role:** 5 · **Depends on:** Phase 2

- [ ] Implement `analysis/kpis.py` computing, from a run log:
  - [ ] **Miss Distance `R_miss`** — minimum interceptor–target range over the run.
  - [ ] **Time-to-Intercept** — time to reach the miss-distance threshold.
  - [ ] **Z-Axis Overshoot** — max altitude above the target during approach.
  - [ ] **Command Saturation** — fraction of flight time the Limiter/Mixer reported
        saturation.
  - [ ] **Max Target Speed handled** and **per-trial success/fail** vs the targets,
        each with the 5% margin from `config/constants.py`.
- [ ] Pull thresholds from constants (no inline numbers); output a structured KPI
      record per run.
- [ ] **DoD:** KPI values verified against hand-computed cases on synthetic logs.

### T3.2 — Scenario runner & declarative configs
**Role:** 5 · **Depends on:** T3.1

- [ ] Implement `analysis/scenarios.py` + YAML scenario schema specifying: seed,
      target trajectory + params, interceptor initial state, sensor/wind profile, active
      guidance law, time limit.
- [ ] Persist the resolved config + seed + git hash with each run to `results/`
      (reproducibility contract).
- [ ] Support headless batch execution of a scenario list.
- [ ] **DoD:** re-running a scenario file reproduces identical logs and KPIs.

### T3.3 — Static-target trials
**Role:** 5 (measure) → Roles 2/3/4 (tune) · **Depends on:** T3.1, T3.2

- [ ] Define a spread of static-target geometries (varied 3D offsets, initial ranges,
      bearings).
- [ ] Run with OGL; measure all KPIs; file any failing KPI to the owning role.
- [ ] Confirm `b = 0.1` eliminates Z-axis overshoot in practice (cross-check with
      `b` disabled to demonstrate the effect).
- [ ] **DoD:** static KPIs met across the geometry spread with OGL; the `b` effect is
      documented.

### T3.4 — Linear moving-target trials
**Role:** 5 (measure) → Roles 2/3/4 (tune) · **Depends on:** T3.3

- [ ] Define constant-velocity scenarios at a range of slow/moderate closing speeds and
      crossing geometries.
- [ ] Measure tracking quality (EKF), interception time, overshoot, saturation.
- [ ] **DoD:** linear KPIs met (`t_int < 20 s`, others within target) with OGL.

### T3.5 — EKF tuning
**Role:** 2 · **Depends on:** filed findings from T3.3/T3.4

- [ ] Tune process/measurement covariances (`Q`/`R`) against the Phase 1 noise/latency
      profiles so the **LOS rate** delivered to Guidance is clean and lag-compensated.
- [ ] Document the covariance choices and assumptions; keep them in `config/params.py`.
- [ ] **DoD:** estimate error and LOS-rate noise are low enough that guidance is not the
      bottleneck; documented.

### T3.6 — Guidance tuning
**Role:** 3 · **Depends on:** filed findings from T3.3/T3.4

- [ ] Tune the time-varying `N'` schedule, the altitude penalty `b`, and the tilt
      time-constant `T` used in the OGL lag model.
- [ ] Verify the qualitative claims from the Design Review: OGL reaches the target
      quickly (the Design Review cites ≈ 12× faster than the rejected PN/APN baselines)
      with no Z-axis overshoot.
- [ ] **DoD:** OGL tuning documented; KPI improvements attributable to specific changes.

> Any change to KPI-affecting tuning constants is confirmed with the user before being
> committed (AGENTS.md → Workflow).

### T3.7 — Control tuning
**Role:** 4 · **Depends on:** filed findings from T3.3/T3.4

- [ ] Tune inner (400 Hz) and outer (50 Hz) PID gains for stable, responsive tracking
      without excessive oscillation, keeping **command saturation ≤ 5%**.
- [ ] Verify the Limiter bounds are consistent with achievable motor authority.
- [ ] **DoD:** stable aggressive maneuvers; saturation KPI within target on static/linear
      scenarios.

### T3.8 — OGL characterization & `b`-penalty ablation
**Role:** 5 · **Depends on:** T3.3–T3.7

- [ ] Run OGL on a spread of **identical seeded scenarios**, and repeat with the altitude
      penalty disabled (`b = 0`) to isolate its effect on Z-overshoot.
- [ ] Tabulate KPIs; generate plots (trajectories, altitude vs time, command effort, miss
      distance) to `results/`.
- [ ] Report faithfully; contextualize against the Design Review's cited PN/APN numbers
      (documentary reference only — PN/APN are not implemented).
- [ ] **DoD:** a reproducible report quantifies OGL performance and the `b`-penalty effect.

### T3.9 — Regression test suite
**Role:** 5 · **Depends on:** T3.3, T3.4

- [ ] Capture the passing static/linear scenarios as seeded regression tests asserting
      KPI pass/fail, so future changes can't silently regress them.
- [ ] **DoD:** regression suite runs headless and green; failures pinpoint the offending
      KPI.

---

## Deliverables

- `analysis/{kpis,scenarios,reporting}.py`; scenario YAMLs under `scenarios/`.
- Tuned `config/params.py` (EKF `Q`/`R`, PID gains, `N'`, `b`, `T`) with rationale.
- OGL characterization + `b`-ablation report + plots in `results/`.
- Static/linear regression tests.

## KPIs Targeted This Phase

| Metric | Target (this phase's scenarios) |
| :--- | :--- |
| `R_miss` | ≤ 1.05 m (static + linear) |
| Time-to-Intercept | Static < 10 s; Linear < 20 s |
| Z-Axis Overshoot | ≤ 0.5 m |
| Command Saturation | ≤ 5% |

(Max target speed and ≥90% mission success are stressed in Phase 4.)

## Risks & Mitigations

- **Tuning masks a structural defect** → Role 5 files findings; owning role fixes the
  cause, not the scenario.
- **Overfitting to a few geometries** → use a spread of seeded static/linear cases;
  lock them as regressions (T3.9).
- **Non-reproducible results** → enforce config+seed+git-hash logging (T3.2).

## Boundaries (do not cross)

- Role 5 does not edit estimation/guidance/control internals; never relax targets or
  hand-tune a scenario to pass.

## References

- Design Review §6 (algorithm comparison — OGL selected; PN/APN rejected), §7
  (scenarios + KPIs), §8 Phase 3.
- Thesis summary Ch. 5 (OGL outperforms; ~12× faster; no overshoot).
- AGENTS.md → Role 5, KPI table, Testing, Workflow.
