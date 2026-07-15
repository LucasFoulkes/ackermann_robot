> **HISTORICAL** (frozen 2026-07-15) — superseded by `stack-review-2026-07-15.md` and `how-the-learner-works.md`; kept for provenance. Verdicts here may be stale.

# Stage F design brief — throttle-map identification by probe excitation

Basis: adversarially verified deep-research pass, 2026-07-13 (105 agents;
votes noted). Full trace in the session workflow journal. This brief is the
design authority for the probe scheduler and throttle-map learner.

## 1. Verified theoretical basis

1. **Ordinary closed-loop driving cannot identify the throttle map**
   (15-0, Forssell & Ljung 1999; Van den Hof 1998; Gevers et al.). Under
   feedback, input correlates with disturbances; estimates converge toward a
   blend of the plant and the inverse controller (−1/C) — the structural
   bias behind our historical wrong-signed slopes. An injected exogenous
   probe added to the controller output is the canonical remedy and
   automatically satisfies the informativity condition.
2. **Only probe-injected energy buys accuracy** (6-0): estimation accuracy
   is set by the noise-to-signal ratio against the probe part of the input
   spectrum alone; feedback-originated input energy contributes zero
   information. Probe amplitude must therefore be sized against the 10 Hz
   lidar-odometry noise floor, not against the actuator resolution.
3. **Estimator: instrumental-variable / correlation keyed to the KNOWN
   probe** (9-0). Direct fitting is optimal only if the noise dynamics are
   correctly modeled — any misfit biases the map itself. IV against the
   known probe is consistent without a noise model and valid under our
   nonlinear feedforward-dominant loop. This also honors the standing
   RLS-windup warning.
4. **Lock-in (synchronous) demodulation** (9-0): modulate the probe at a
   known frequency; multiply measured speed by the probe reference and
   low-pass. Rejects 1/f terrain/battery drift and recovers responses below
   the raw noise floor. Choose a frequency whose odd harmonics avoid known
   periodic disturbances.
5. **Waveform: triangular envelope, not square taps** (9-0), when the probe
   must also dither the ~2-tick quantizer/deadband: square dither has
   impulsive amplitude density and provably fails to smooth nonsmooth
   nonlinearities; triangular/sawtooth smooths with error proportional to
   dither period. (A separate claim that square needs the minimum
   quenching amplitude was REFUTED 0-3 — do not cite it.)
6. **Hammerstein caution** (6-0): our structure (static map + first-order
   lag) sits near the classical marginal sampling regime (10 Hz, 1-2 sample
   delay) where routine-data identification converges to an INCORRECT
   model — reinforcing probes over passive data.

## 2. Refuted / unanswered — treat as open engineering, not literature

- All quantitative minimum-probe-amplitude claims: REFUTED. The amplitude
  ladder must be determined empirically on this robot.
- Shipped-system precedents (ArduPilot AUTOTUNE, OEM pedal-map learning):
  nothing survived verification. Do not argue from folklore.
- Certification protocol without encoders and below-floor pulse-and-coast
  control: unanswered; design ours from first principles (tape-measure +
  held-out, per the MuSHR-style method already adopted for Stage G).

## 3. The design (to implement)

- **Probe**: triangular-envelope throttle offset at a fixed known frequency
  in 0.5–1.5 Hz (pick after checking the odd harmonics against measured
  periodic disturbances in existing bags), amplitude ladder starting at
  ±4 µs and escalating (±8, ±12, …) until the lock-in SNR over N cycles
  crosses a detection threshold; hold at the minimal detectable amplitude.
- **Detection/estimation**: lock-in demodulation of lidar speed against the
  probe reference → local slope Δspeed/Δpulse at the current operating
  pulse; accumulate slopes into per-region bins (same binned + isotonic
  machinery as the steering map); shared multiplicative effort-scale stays
  the battery/surface term (multiplicative decomposition).
- **Gating** (all existing): healthy odometry, valid path, clearances,
  steady rolling above observation floor, small tracking errors, no cusp or
  stop nearby, away from actuator bounds, no recent safety event, strict
  path-error budget, sign alternation + cooldown, auto-disable on measured
  tracking harm.
- **Certification**: held-out bag prediction + physical tape-measure runs
  (numeric threshold), champion/challenger vs. the measured anchor map
  before any authority transfer; §16.2 no-inversion criterion under
  terrain/closed-loop confounding.
- **Identifiability limits accepted**: the map is learned only over the
  naturally exercised speed range; below-floor behavior remains a separate
  explicit mode; battery state and map shape are separated only up to the
  multiplicative model's validity.

## 3b. Bench results (2026-07-13, `experiments/lockin_bench.py`) — §4.1 DONE

Offline bench against REAL lidar-odometry noise (1,021 rolling residual
samples, sd 42 mm/s) with synthetic triangular probes through a 0.25 s ESC
lag at the measured local slope 0.030 m/s/µs:

- **Working point: 0.9 Hz, ±4 µs, 10 cycles (11 s) → 11% slope sd** ✓
  (stealth option: ±2 µs, 20 cycles → ~13%; high-precision: ±4 µs,
  20 cycles → 7%). 0.9 Hz beats the quieter 1.3 Hz band because ESC-lag
  attenuation dominates (gain 0.58 vs 0.44).
- Speed ripple at ±4 µs is 0.059 m/s peak (±2 µs: 0.029 — below the
  odometry noise floor). Estimator unbiased at all tested points.
- **1/f drift rejection proven**: adding a battery-sag ramp changed the
  estimate sd by <0.5% at every operating point.
- Empirical ladder for the scheduler: start ±2 µs/20 cyc; escalate to
  ±4 µs/10 cyc if SNR insufficient; never exceed ±8 µs.

## 4. Implementation order

1. Bench the lock-in offline: replay existing bags, inject synthetic probes
   into the model, verify the demodulator recovers known slopes.
2. Probe scheduler in the controller (shadow: inject + measure + log; no
   map authority), gated as §3.
3. One armed session: verify probes are invisible in tracking KPIs and the
   lock-in SNR ladder converges at some amplitude.
4. Slope accumulation → challenger throttle curve → held-out + tape-measure
   certification → Stage D-style promotion with rollback.
