# Phase 4 — Randomized Evasive / Windy Trials & Final Data

> **Roadmap window:** Aug 6 – Aug 20.
> **Primary role:** Role 5 — Test, Validation & KPI Engineer (drives); owning roles
> (1/2/3/4) receive filed findings and fix within their own layers.
> **Pipeline stages touched:** All (full-system stress testing); no new stages.
> **Goal:** Stress the tuned system against the hardest scenarios — sinusoidal evasive,
> varying speed up to 90 km/h, and wind/gust disturbance — then run **randomized 3D
> Monte-Carlo trials** for statistical robustness and compile the **final performance
> dataset**. The phase passes only when every KPI in the success table is met with its
> 5% margin, including **≥ 90% mission success** and **≥ 83.6 km/h** max target speed.

> **Role-5 boundary:** measure and **file findings**; never tune scenarios or relax
> targets to manufacture a pass. Fixes go to the owning role.

---

## Entry Criteria

- Phase 3 complete: KPI + scenario tooling in place; static and linear KPIs met with
  OGL; PN/APN/OGL benchmark documented; regression suite green.

## Exit Criteria (Definition of Done for the phase)

- [ ] Sinusoidal (evasive), varying-speed (to 90 km/h), and wind/gust scenarios each
      run and are measured against the KPI table.
- [ ] Randomized 3D Monte-Carlo harness runs a statistically meaningful seeded batch.
- [ ] **All KPIs met with 5% margin**, including `R_miss ≤ 1.05 m`,
      `Z-overshoot ≤ 0.5 m`, `saturation ≤ 5%`, **max target speed ≥ 83.6 km/h**, and
      **mission success ≥ 90%**.
- [ ] Final KPI dataset, dashboard/report, and plots generated to `results/`.
- [ ] A reproducibility package lets anyone regenerate the final dataset from configs +
      seeds.

---

## Tasks

### T4.1 — Sinusoidal (evasive) trials
**Role:** 5 (measure) → owning roles (fix) · **Depends on:** Phase 3

- [ ] Build sinusoidal-evasion scenarios over a range of amplitudes/frequencies in 3D,
      seeded.
- [ ] Stress the EKF tracking and OGL responsiveness; measure all KPIs.
- [ ] File any failing KPI (e.g., EKF lag, overshoot) to the owning role.
- [ ] **DoD:** evasive-target KPIs measured and reported; failures attributed to a layer.

### T4.2 — Varying-speed trials up to 90 km/h
**Role:** 5 (measure) → owning roles (fix) · **Depends on:** Phase 3

- [ ] Scenarios scaling target speed up to and beyond 90 km/h (25 m/s) to probe the
      `MAX_TARGET_SPEED_MIN_KMH = 83.6` km/h requirement.
- [ ] Identify the maximum reliably intercepted speed; confirm it clears 83.6 km/h.
- [ ] **DoD:** documented max intercept speed ≥ 83.6 km/h with KPIs within target;
      degradation curve beyond that recorded.

### T4.3 — Wind & gust robustness trials
**Role:** 5 (measure) → owning roles (fix) · **Depends on:** Phase 3

- [ ] Run static/linear/evasive engagements under the Phase 1 wind/gust presets
      (moderate, gusty), seeded.
- [ ] Measure control-loop robustness: saturation %, overshoot, miss distance under
      disturbance.
- [ ] **DoD:** KPIs hold (within targets) under wind; any robustness gap filed to Role 4.

### T4.4 — Randomized 3D Monte-Carlo trial harness
**Role:** 5 · **Depends on:** T4.1–T4.3

- [ ] Build a batch harness that samples **seeded** randomized initial geometry,
      target trajectory family + parameters, speed, and disturbance — across the full
      scenario space.
- [ ] Each trial logs its full config + seed + git hash and a per-trial KPI record.
- [ ] Ensure headless, non-interactive, parallel-friendly execution.
- [ ] **DoD:** a single command runs the full randomized batch reproducibly and emits a
      per-trial KPI table.

### T4.5 — Mission success-rate aggregation
**Role:** 5 · **Depends on:** T4.4

- [ ] Aggregate per-trial pass/fail into the **Mission Success Rate**; verify
      `≥ MISSION_SUCCESS_MIN (90%)`.
- [ ] Break results down by scenario family and condition to expose weak regimes.
- [ ] **DoD:** success rate computed over the randomized batch with a per-family
      breakdown; ≥ 90% overall (or failures filed if not).

### T4.6 — Final KPI dashboard & report
**Role:** 5 · **Depends on:** T4.4, T4.5

- [ ] Produce the consolidated final report: every KPI vs its target (5% margin),
      success rate, max speed, and representative plots (trajectories, altitude, command
      effort, miss-distance distributions).
- [ ] Include the Phase 3 PN/APN/OGL comparison context for completeness.
- [ ] **DoD:** a single authoritative report in `results/` summarizes final performance
      faithfully, surfacing any KPI that still misses.

### T4.7 — Failure-mode analysis & findings
**Role:** 5 · **Depends on:** T4.4

- [ ] Catalogue failed/marginal trials; classify root cause by owning layer
      (sensor/EKF/guidance/control); file actionable findings.
- [ ] Do **not** fix other layers' logic; hand off to the owning role.
- [ ] **DoD:** a triaged findings list mapped to roles, with reproduction seeds.

### T4.8 — Reproducibility package
**Role:** 5 · **Depends on:** T4.6

- [ ] Bundle the scenario configs, seeds, environment manifest, and run instructions
      needed to regenerate the final dataset and report from scratch, headlessly.
- [ ] **DoD:** following the instructions reproduces the headline KPIs within run-to-run
      determinism.

### T4.9 — Final documentation update
**Role:** 6 (coordinates) · **Depends on:** T4.6

- [ ] Update `README.md` and the docs with final results, how to reproduce, and known
      limitations.
- [ ] Confirm the implementation matched the Classical Hierarchical architecture
      throughout (no DRL crept in).
- [ ] **DoD:** docs reflect the delivered system and final numbers.

---

## Deliverables

- Sinusoidal / varying-speed / wind scenario suites under `scenarios/`.
- `analysis/` Monte-Carlo harness + aggregation + final reporting.
- Final KPI dataset, dashboard/report, and plots in `results/`.
- Triaged failure-mode findings; reproducibility package; updated docs.

## KPIs Targeted This Phase (full table — final acceptance)

| Metric | Success Target |
| :--- | :--- |
| Miss Distance `R_miss` | ≤ 1.05 m |
| Time-to-Intercept | Static < 10 s; Moving < 20 s |
| Z-Axis Overshoot | ≤ 0.5 m above target |
| Command Saturation | ≤ 5% of total flight time |
| Max Target Speed | ≥ 83.6 km/h |
| Mission Success Rate | ≥ 90% over randomized 3D trials |

## Risks & Mitigations

- **Tracking breaks under fast evasion** → isolate via T4.1/T4.2 and file to EKF/Guidance
  rather than over-tuning a single scenario.
- **Wind pushes saturation over 5%** → file to Role 4 (limiter/control), not a scenario
  relaxation.
- **Monte-Carlo not reproducible** → strict seed + config + git-hash logging (T4.4).
- **Cherry-picking results** → report the full distribution and the per-family
  breakdown, including failures (T4.5–T4.7).

## Boundaries (do not cross)

- Role 5 measures and files findings only; never edits other layers' logic, never tunes
  scenarios or relaxes targets to force a pass. Architecture stays Classical
  Hierarchical — no DRL substitution.

## References

- Design Review §7 (scenarios + KPI table), §8 Phase 4.
- Thesis summary Ch. 5 (moving/high-speed/robustness results).
- AGENTS.md → Role 5, Role 6, KPI table, Determinism & reproducibility.
