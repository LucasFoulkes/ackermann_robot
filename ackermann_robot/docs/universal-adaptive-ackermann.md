# Universal Adaptive Ackermann Learning Node

## Goal, current baseline, target architecture, and proof plan

Status: governing design document, written 2026-07-12.

Reference baseline: commit `c61d041`, tag
`experiment-20260712-135900`, drive log
`adaptive_drive_20260712_135900`.

This document defines what this project is trying to build. Older documents
remain valuable experiment records, but where their proposed architecture
conflicts with this document, this document is authoritative.

## 1. North-star goal

Build one stateful ROS 2 control-and-learning system that can be installed on
another Ackermann vehicle in the supported hardware family, given only facts
that cannot be inferred safely, and then learn how that vehicle drives while
executing ordinary Nav2 goals.

The owner supplies the vehicle geometry, sensor mounting, actuator interface,
neutral commands, and safe electrical pulse bounds. The robot learns the
relationship between those commands and actual motion. Command polarity is not
supplied: the sign of the first confirmed displacement and of the first
curvature response identifies it trivially, so it is learned during bootstrap
like everything else the robot can observe for itself.

It must learn without a scripted calibration course, a special floor area,
wheel encoders, a steering-angle sensor, a battery-voltage sensor, or manual
gain tuning. Normal navigation is the training activity. Small, safety-gated
identification probes may be added to ordinary motion when natural driving
does not provide enough independent information; those probes must not choose
the route or require a special path.

The intended user experience is:

1. Install the package.
2. Provide the birth certificate.
3. Place the robot in a supervised, reasonably clear environment.
4. Send ordinary Nav2 goals.
5. The robot begins cautiously, learns its actuation and steering response,
   expands its trusted envelope as evidence accumulates, and retains that
   knowledge across restarts.
6. No configuration constants are manually changed between ordinary drives.

The system is done only when it passes both of these exams:

- **Amnesia exam:** back up and remove all learned memory on this robot. From
  its birth certificate and normal Nav2 goals, it must recover useful driving
  without restoring this robot's measured maps.
- **Second-vehicle exam:** install the same software on a materially different
  Ackermann chassis in the supported family. Given that chassis's birth
  certificate, it must learn its own models and reach the required navigation
  quality without copying this robot's learned state.

## 2. Supported portability domain

“Any Ackermann vehicle” has a practical boundary. The first universal version
targets vehicles with:

- Ackermann-like steering and a known wheelbase;
- a PWM hobby steering servo with supplied neutral and safe pulse range;
- the same ESC command protocol and forward/reverse operating mode used by
  this project, with supplied throttle channel, neutral, and safe pulse
  range;
- lidar odometry or another source capable of estimating planar pose, signed
  longitudinal speed, and yaw rate at roughly 10 Hz or better;
- a known rigid footprint and lidar transform;
- Nav2 goals and paths expressed in the robot's normal odometry frame.

Different mass, gearing, tires, servo linkage, steering ratio, battery state,
floor, temperature, left/right asymmetry, and forward/reverse behavior are
inside the learning problem.

A vehicle outside this interface—for example hydraulic steering, skid steer,
an ESC with a different arming protocol, or no reliable planar odometry—needs
an adapter or a different model family. The software must detect unsupported
observations rather than silently applying a bad Ackermann model.

## 3. Facts supplied once: the birth certificate

The birth certificate contains physical/interface facts, not learned driving
performance. A proposed schema is:

```yaml
schema_version: 1

vehicle:
  wheelbase_m: 0.2775
  footprint:
    front_x_m: 0.40
    rear_x_m: -0.10
    left_y_m: 0.16
    right_y_m: -0.16

lidar:
  frame_id: laser
  x_m: 0.237
  y_m: 0.0
  yaw_rad: 3.141592653589793

actuators:
  throttle:
    channel: 14
    neutral_us: 1500.0
    safe_min_us: ...
    safe_max_us: ...
    esc_mode: forward_reverse
  steering:
    channel: 12
    approximate_neutral_us: ...
    safe_min_us: ...
    safe_max_us: ...

odometry:
  topic: /odom
  base_frame: base_link
  odom_frame: odom
```

Safe electrical bounds are required. With no steering-angle, servo-current,
or force sensor, the robot cannot safely discover that an arbitrary PWM value
is mechanically binding the linkage. A human feels a steering stop; this
robot does not. Everything inside the supplied safe range may be explored and
learned.

The birth certificate must not contain:

- throttle or steering polarity (one clean observed motion identifies each);
- a calibrated steering PWM-to-curvature table;
- a measured minimum turning radius;
- a throttle speed map;
- a breakaway pulse;
- a steering-delay constant;
- direction-specific steering gain;
- a manually tuned planner radius.

Those belong in learned memory.

## 4. What exists today

### 4.1 Current runtime architecture

```text
Nav2 goal
  -> Smac Hybrid-A* Reeds-Shepp path
  -> chronological cusp dispatcher
       - one forward/reverse segment at a time
       - segment progress and path-validity checks
       - trackability scoring
  -> Regulated Pure Pursuit
  -> executable Ackermann command boundary
  -> Collision Monitor
  -> adaptive actuator node
  -> PCA9685 steering and ESC pulses

lidar -> MOLA odometry -> pose, speed, yaw rate
                         -> controller feedback and learning evidence
```

The dispatcher prevents RPP from pruning across Reeds-Shepp cusps, explicitly
dwells between direction changes, cancels a segment after six seconds without
0.05 m of new chronological progress, and checks the next 1.50 m of the
committed segment through Nav2's footprint-aware path-validity service.

The actuator node applies measured direction-specific throttle and steering
maps, learns several corrections, enforces the physical command envelope, and
logs a 50 Hz CSV. Bags contain the sensor, plan, segment, command, costmap,
debug, and learning topics.

### 4.2 Current persistent state

Two files currently hold learned state:

- `~/.robot/adaptive_ackermann_runtime.yaml`
  - four shadow steering RLS branches;
  - steering-delay candidate weights;
  - forward/reverse breakaway models;
  - forward/reverse minimum-speed floor observations;
  - session throttle trims;
  - a pooled effort-scale estimate.
- `~/.robot/planner_trackability.yaml`
  - forward/reverse and positive/negative curvature branches;
  - eligible observation counts;
  - pass/failure history and streaks;
  - branch curvature estimates;
  - planner-radius source and confidence.

State is written atomically. Every launch creates a new timestamped CSV in
write mode; it does not append to the previous run.

### 4.3 Honest current learning inventory

| Quantity | Current status | Applied to control? | Portable today? |
|---|---|---:|---:|
| directional breakaway pulse | learned from motion-onset events | yes | partly |
| session throttle trims | learned online | yes | partly |
| minimum sustainable speed | observed and persisted | configured floor still authoritative | partly |
| pooled effort scale | learned online | shadow/limited influence | not certified universally |
| steering RLS gain/bias | learned from closed-loop samples | no, shadow | no |
| steering response time/distance | measured constants plus shadow candidate estimator | fixed measured model applied | no |
| planner trackability by four branches | learned from eligible segments | next launch | partly |
| forward/reverse throttle maps | measured on this robot | yes | no |
| forward/reverse dense steering maps | measured on this robot | yes | no |
| maximum physical curvature | measured/certified on this robot | yes | no |
| initial planner radius, 1.30 m | fixed bootstrap prior | yes | no |

### 4.4 The current dense steering table

The configuration contains historically named “24-point” forward and reverse
steering maps. They are not 24 discrete steering positions: the controller
interpolates between curvature/PWM knots. They therefore produce a continuous
piecewise-linear command.

The problem is not command discretization. The problem is provenance: those
knots were measured through scripted experiments on this chassis and remain
the authoritative feedforward map. A different steering linkage would inherit
the wrong curve until a human replaced it.

The target system may still store knots internally—monotone splines and
piecewise-linear functions are practical on a Pi—but the knots must be learned
evidence with uncertainty, not hand-carried calibration.

### 4.5 Current validated baseline

The reference run `adaptive_drive_20260712_135900` established a stable point
from which to build the learner:

- 15 chronological segments across seven global plans;
- no chronological-progress, wrong-direction, or path-invalidity aborts;
- effectively no terminal pulse-and-coast intervention;
- completed cusp endpoints commonly around 0.056--0.067 m;
- no post-monitor speed or curvature mutation;
- maximum observed launch peak 0.365 m/s, with none above 0.40 m/s;
- steering delay estimator near 0.30 s with high internal confidence;
- effort scale about 1.004 with standard deviation about 0.018;
- planner evidence confidence advanced to 0.67;
- planner radius remained at the 1.30 m bootstrap prior;
- accepted curvature reached the configured 1.148 1/m physical envelope for
  about 31% of nonzero command rows.

This is a working vehicle controller, not yet the universal learner. Future
work must preserve this baseline and prove each learned replacement before it
becomes authoritative.

## 5. What is wrong with the current abstraction

### 5.1 A fixed delay is not steering dynamics

The current controller approximates steering response with direction-specific
time and distance offsets. That helps preview, but it treats a gradual
mechanical transition as if a correct steering angle appears after a delay.

The actual sequence is:

```text
PWM target changes
  -> servo shaft moves
  -> saver/linkage loads or unloads
  -> wheel angle changes gradually
  -> tire force develops while rolling
  -> lidar odometry observes yaw response later
```

The 2026-07-11 two-speed experiment found both types of behavior:

- forward response was mostly time-dependent, approximately 0.235 s;
- reverse response included a time component near 0.168 s plus roughly
  0.033 m of rolling-distance dependence.

Those measurements disprove both extremes: steering is neither instantaneous
nor adequately described by one pure delay.

### 5.2 Fixed terminal modes defeated adaptation

The failed `adaptive_drive_20260712_134816` run demonstrated a general rule.
A fixed remaining-distance threshold replaced continuous adaptive throttle
control with pulse-and-coast. Repeated 0.227 m segments advanced about 0.113 m,
crossed the threshold, and stopped adapting to the remaining error.

That mechanism was removed in `c61d041`. The lesson applies beyond throttle:
a learned loop must not be silently replaced by a fixed mode switch merely
because the robot entered a difficult regime.

### 5.3 Closed-loop samples are not automatically identification data

Naively fitting commanded steering against measured curvature learns the
controller's corrections as much as it learns the vehicle. Likewise, throttle
effort is highest exactly where the surface or battery makes motion difficult.
Historical naive fits produced misleading gains and even wrong-signed slopes.

Ordinary driving remains sufficient, but only with:

- event-based observations where the event has clear meaning;
- settled, correctly time-aligned samples;
- known small excitation independent of the tracking correction;
- instrumental variables derived from planner geometry;
- contamination rejection;
- held-out prediction before promotion.

### 5.4 The planner learns too slowly and only at process launch

The current four-branch estimator is directionally correct, but it injects its
radius into Smac/RPP when the navigation stack starts. Evidence collected in a
drive therefore cannot affect a later goal in the same process.

The 1.30 m value is a cautious starting hypothesis, not a learned fact. It
must become a geometry-scaled initial prior followed by confidence-bounded
updates between goals.

## 6. Target learned state

The universal node should persist one versioned memory document. A conceptual
schema is:

```yaml
schema_version: 1
vehicle_fingerprint: ...
created_at: ...
last_updated_at: ...

throttle:
  polarity: ...            # learned from first confirmed displacement
  forward:
    breakaway_distribution: ...
    sustain_curve: ...
    minimum_sustainable_speed: ...
    response_dynamics: ...
    confidence: ...
  reverse: ...
  shared_effort_state:
    mean: ...
    variance: ...

steering:
  polarity: ...            # learned from first curvature response
  approximate_center_us: ...
  steady_map:
    representation: monotone_spline
    shared_base_knots: ...
    forward_correction: ...
    reverse_correction: ...
    left_correction: ...
    right_correction: ...
    uncertainty_by_region: ...
  dynamics:
    movement_rate_us_per_s: ...
    time_constant_s: ...
    reverse_distance_constant_m: ...
    hysteresis_width_us: ...
    load_dependence: ...
    uncertainty: ...
  reachable_envelope:
    forward_negative: ...
    forward_positive: ...
    reverse_negative: ...
    reverse_positive: ...
  trackable_envelope: ...

planner:
  applied_radius_m: ...
  candidate_radius_m: ...
  branch_confidence: ...
  evidence: ...

promotion:
  champion_model_version: ...
  challenger_model_version: ...
  rollback_reason: ...
  validation_metrics: ...
```

Every learned quantity needs four distinct fields or concepts:

1. **Estimate:** what the evidence currently suggests.
2. **Uncertainty/confidence:** how much relevant evidence supports it.
3. **Applied value:** what control is allowed to use now.
4. **Validity domain:** direction, steering side, speed, surface regime, and
   command history for which the evidence is trustworthy.

An estimator is not allowed to call itself learned merely because it contains
a number.

## 7. Target steering model

### 7.1 Steady steering curve

The base geometric relationship is:

```text
kappa = tan(delta) / wheelbase
```

The node does not observe `delta` directly, so it learns a monotone command
map from PWM to steady curvature:

```text
kappa_steady = f(steering_pwm)
```

The preferred representation is a constrained monotone spline or monotone
piecewise-linear curve. Requirements:

- continuous output throughout the safe pulse range;
- monotonicity unless repeated evidence proves a mechanical anomaly;
- an explicit center and uncertainty around center;
- no forced left/right symmetry;
- no forced forward/reverse separation when a shared base plus correction
  explains the data;
- denser learned knots where curvature changes rapidly or uncertainty is high;
- saturation represented as a plateau only after repeated evidence;
- reversible storage and human-readable diagnostics.

The model should start simple and add complexity only when residuals justify
it. A three-knot cautious prior is preferable to an invented dense curve.

### 7.2 Dynamic effective steering state

The controller must maintain an estimate of the steering state that exists
between commanded PWM and observed vehicle curvature. One practical family is:

```text
target_kappa = f(steering_pwm)

effective_kappa_next = effective_kappa_now
  + bounded_rate_toward(target_kappa, dt)
  + direction_distance_term(ds)
  + hysteresis_term(command_history)

measured_kappa = yaw_rate / signed_speed
```

Equivalent wheel-angle state may be used internally. The important behavior
is gradual evolution, not the exact equation chosen initially.

Candidate dynamic terms are:

- a time-domain movement rate or first-order settling constant;
- a rolling-distance term, especially in reverse;
- different load response when entering versus leaving high curvature;
- center/backlash state based on the direction from which the servo arrived;
- confidence decay after a long power-off interval or hardware change.

Model order must be selected from held-out prediction. If hysteresis or speed
dependence is smaller than repeatability noise, it stays out of the applied
model.

### 7.3 Observation and alignment

At sufficient signed speed:

```text
observed_kappa = measured_yaw_rate / measured_longitudinal_speed
```

Samples must be aligned to the command history through the candidate dynamic
model. Reject observations during:

- stale or low-quality odometry;
- speed below the reliable curvature-observation floor;
- direction transitions;
- Collision Monitor intervention;
- local obstacle stops;
- throttle recovery or overspeed;
- excessive cross-track error;
- command saturation without enough settled duration;
- pose jumps or odometry outliers.

The node should log accepted and rejected observations with reasons. Learning
that cannot explain which samples it used is not auditable.

### 7.4 Reachable versus trackable curvature

Two envelopes must remain separate:

- **Reachable curvature:** what the steering mechanism can physically achieve
  inside the supplied safe pulse range.
- **Trackable curvature:** what the complete robot/controller can repeatedly
  follow with acceptable error in each direction and turning side.

A vehicle may reach a tight curvature momentarily but fail to track a planned
arc at that curvature. The planner uses the confidence-bounded trackable
envelope, while the actuator clamp uses the reachable envelope.

The current phrase “certified physical curvature” should be replaced by
evidence semantics. Certification is a state reached by repeatable evidence,
not a hard-coded value.

## 8. Target throttle model

The same ESC protocol reduces interface uncertainty, but it does not make the
vehicle dynamics identical. Mass, gearing, motor, tires, battery, and surface
still change the pulse-to-motion response.

The node must learn, per direction:

- motion-onset/breakaway distribution;
- minimum pulse that sustains rolling;
- minimum sustainable speed;
- steady pulse-to-speed curve over the naturally exercised range;
- launch response and safe drop-through behavior;
- acceleration/deceleration response where observable;
- a shared short-term effort state representing battery/surface/temperature
  without pretending to distinguish them with unavailable sensors.

Breakaway remains event-based. Steady speed-map identification needs small
known excitation or another instrument because closed-loop equilibrium samples
are terrain-biased. Below the learned sustainable floor, bounded pulse-and-
coast is permitted only for an explicit sub-floor Nav2 request; it must never
replace ordinary endpoint progress based on a fixed distance.

## 9. Learning from ordinary Nav2 goals

### 9.1 Natural evidence

Normal routes naturally provide:

- starts for breakaway learning;
- straight motion for steering-center evidence;
- ordinary bends for mid-map curvature;
- tight obstacle avoidance for higher curvature;
- Reeds-Shepp cusps for direction transitions;
- forward and reverse turns for branch coverage;
- stops and slow approaches for sustainable-speed evidence;
- repeated command transitions for steering-dynamics estimation.

The learner must consume what the route actually exercises. It must not claim
knowledge of reverse-right full lock after only forward-left driving.

### 9.2 Invisible identification probes

Natural closed-loop motion does not always identify command response cleanly.
The controller may add a small alternating steering or throttle probe while
Nav2 continues to own the path and destination.

A probe is allowed only when all gates pass:

- localization and odometry are healthy;
- the committed path is valid;
- adequate obstacle and stopping clearance exists;
- the robot is steadily rolling above the observation floor;
- cross-track and heading errors are small;
- no cusp, stop, or direction change is near;
- the actuator is away from its safe electrical bound;
- no recent safety or recovery event occurred;
- predicted probe displacement stays inside a strict path-error budget.

Probe sign alternates, amplitude is bounded, and a cooldown prevents continuous
excitation. The response is correlated with the known probe, not blindly fit
against the closed-loop command. If a probe measurably harms tracking, the
scheduler disables it and records why.

This satisfies the no-scripted-path requirement: the route remains an ordinary
Nav2 route, and identification is a small overlay during uneventful motion.

### 9.3 Evidence lifecycle

```text
sensor + command histories
  -> candidate observation
  -> quality/contamination gates
  -> time/distance alignment
  -> estimator update
  -> challenger-model prediction
  -> held-out comparison against champion
  -> confidence update
  -> safe-boundary promotion or rejection
  -> atomic persistence
```

Contraction after credible failure is faster than outward expansion. Unseen
regions stay conservative. Confidence is branch- and domain-specific.

## 10. Bootstrap behavior on a new vehicle

The robot cannot know a steering map before moving. Like a human entering an
unfamiliar car, it begins with strong generic expectations, limited authority,
and cautious first actions. Unlike a human, it lacks steering feel and must
respect supplied safe electrical bounds.

### 10.1 Before motion

- Validate the birth-certificate schema and pulse ranges.
- Confirm fresh scan, odometry, TF, and actuator ownership.
- Verify neutral commands while stationary.
- Initialize a low-complexity monotone steering prior centered on the supplied
  approximate neutral.
- Derive an initial planner radius from wheelbase and a cautious generic wheel
  angle, not a fixed 1.30 m universal constant:

  ```text
  initial_radius = wheelbase / tan(cautious_initial_angle)
  ```

- Cap speed and steering authority according to uncertainty.

### 10.2 First ordinary motion

- Until throttle polarity is confirmed, require stopping clearance on both
  sides of the robot and treat either direction of motion as possible.
- Use a cautious launch pulse inside the safe ESC range.
- Detect motion onset and begin the directional breakaway distribution; the
  sign of that first confirmed displacement fixes throttle polarity.
- Fix steering polarity from the curvature sign of the first small steering
  offsets; until then, plan as if either sign is possible and keep steering
  authority minimal.
- Use straight/low-curvature evidence to refine steering center.
- Observe actual curvature for the mild steering commands Nav2 requests.
- Keep unobserved steering regions conservative.

### 10.3 Envelope expansion

As ordinary goals require stronger turns:

- explore only a small increment beyond the trusted region;
- compare predicted and observed curvature;
- record tracking error and obstacle contamination;
- expand the reachable estimate after repeatable response;
- expand the planner envelope only after repeatable trackability;
- contract immediately after comparable clean failures;
- update Nav2 only between goals or committed plans.

Rapid learning means learning an exercised operating region in the first few
ordinary goals. It does not mean claiming untested extremes. No algorithm can
learn a maneuver that the vehicle never attempts or observes.

## 11. Nav2 and planner adaptation

### 11.1 Geometry-scaled bootstrap

The current 1.30 m radius corresponds to a cautious hypothesis for this small
wheelbase, but it must not survive as a universal meter value. The initial
radius should scale with wheelbase and an explicitly documented cautious wheel
angle prior.

### 11.2 Four-branch evidence

Maintain at least:

- forward negative curvature;
- forward positive curvature;
- reverse negative curvature;
- reverse positive curvature.

Each branch stores demand, achieved curvature, tracking error, entry quality,
obstacle/safety contamination, endpoint error, and confidence. Shared-model
evidence may transfer cautiously between branches, but one weak branch can
still constrain a Reeds-Shepp planner.

### 11.3 Applying a learned radius

The planner curvature is a lower-confidence bound, not the mean or maximum:

```text
planner_curvature = min(
  trusted_trackable_curvature_forward_negative,
  trusted_trackable_curvature_forward_positive,
  trusted_trackable_curvature_reverse_negative,
  trusted_trackable_curvature_reverse_positive)

planner_radius = 1 / planner_curvature
```

Updates occur at a safe boundary:

- never halfway through a committed direction segment;
- preferably before computing the next global path;
- without requiring a full process restart;
- with the applied model version recorded in the bag and CSV;
- with automatic rollback if the new envelope degrades tracking.

### 11.4 Ownership boundaries

- Nav2 owns the goal, route, and collision-aware speed/curvature request.
- The dispatcher owns chronological direction segments and replan triggers.
- Collision Monitor owns immediate local collision veto.
- The actuator learner owns command-to-physical-motion realization inside the
  safe electrical and learned reachable envelope.
- No downstream layer may secretly raise a monitor-approved speed or alter its
  nonzero curvature.

## 12. Confidence, promotion, and rollback

Every production model uses champion/challenger semantics.

- **Champion:** currently applied, last known-good model.
- **Challenger:** newly learned candidate running in shadow.
- **Promotion:** challenger predicts held-out observations better and passes
  safety/tracking gates across its claimed validity domain.
- **Rollback:** applied model causes a statistically or operationally credible
  regression in tracking, launch peaks, stalls, direction handling, or safety.

Promotion rules:

- occur only while stopped, between goals, or at another explicitly safe
  boundary;
- never expand more than a bounded increment;
- require evidence on the branch/domain being expanded;
- preserve the previous model and reason for promotion;
- reset short-lived confidence after power-off where battery/surface drift is
  relevant;
- retain long-lived geometric evidence unless hardware identity changes.

Rollback must be automatic and must not erase the failed candidate evidence.
Failed models are useful training data.

## 13. Safety invariants

Learning may change performance estimates, but it may not learn away safety.

Hard invariants:

- supplied electrical pulse bounds are never exceeded;
- actuator ownership/watchdog returns throttle to neutral if the owner dies;
- stale odometry, stale commands, or invalid TF cannot produce continued drive
  effort;
- Collision Monitor may always replace motion with zero;
- the local scan guard may always stop for a closer obstacle;
- a learned model cannot mutate a monitor-approved nonzero command downstream;
- new goals serialize backend ownership;
- path invalidity and no-progress events cancel stale committed motion;
- outward envelope expansion is bounded and confidence-gated;
- evidence contaminated by a safety intervention cannot expand capability;
- memory corruption or schema mismatch falls back to cautious bootstrap, not
  unbounded defaults.

The six-second chronological watchdog is a liveness guard, not an actuator
safety controller. Every executable segment of at least 0.18 m is watched. A
persistent lack of 0.05 m new path progress cancels the segment so Nav2 can
replan; it does not force throttle or reversal.

## 14. Observability and audit requirements

Every drive must make these questions answerable from one bag/CSV plus memory:

- What model version generated this command?
- Was a value a prior, estimate, challenger, or applied champion?
- What was its confidence and validity domain?
- Which observations changed it?
- Which observations were rejected, and why?
- Did Nav2, Collision Monitor, or the actuator layer change the command?
- Was steering still moving toward its target?
- What effective steering state did the model predict?
- What curvature did odometry observe?
- Was the planner radius bootstrap, mixed evidence, or learned?
- Why did a model promote, contract, or roll back?

Required telemetry includes:

- commanded PWM and target curvature;
- predicted effective curvature/wheel angle;
- observed curvature and observation quality;
- steering map region, uncertainty, and saturation state;
- dynamic-model residual;
- probe ID, sign, amplitude, and gates;
- applied/challenger model IDs;
- planner branch evidence and applied radius;
- rejection/promotion/rollback reason codes;
- path progress, endpoint error, and obstacle contamination.

## 15. Migration plan from the current baseline

> **Status 2026-07-12 (evening):** Stage A COMPLETE — scorecard tool
> (`experiments/drive_scorecard.py`), train/held-out split and champion
> benchmarks in `~/.robot/scorecards/bag_manifest.yaml`; three autonomous
> coverage sessions recorded via `experiments/auto_coverage_drive.py`.
> Stage B PASSED OFFLINE — monotone challenger
> (`experiments/steering_map_challenger.py`, 14.6k samples, 30 ordinary
> drives) matches the dense map on the baseline bag and beats it on both
> coverage bags; wrong-sign 0.2%. Stage C PASSED OFFLINE —
> first-order gradual model (`experiments/steering_dynamics_challenger.py`,
> fwd tau 0.4 s, rev tau 0.4 s + 0.01 m) beats fixed-delay median on all six
> held-out direction splits. Live bonus: trackable-curvature clamp
> 1.148 -> 0.92 1/m applied from session-1 evidence cut steering error 38%
> and eliminated saturation in sessions 2–3. Stage D COMPLETE (2026-07-12 evening,
> with one caveat): learned monotone map is the applied steering authority
> inside |kappa|<=0.6; dense map remains rollback champion and high-kappa
> authority. Promotion sessions 6 (big room, median 0.105 ~ champion 0.103,
> 9/9), 8 (small room, 0.129, 9/9+skip), 9 (small room, 0.137, 8/8+2 skips)
> — zero rollbacks, 26/26 driven goals, wrong-sign within champion history,
> endpoints <=0.15 m. Deliberate rollback drill PASSED (forced threshold →
> fired, latched, safe champion continuation). Production false-positive
> rollback in session 7 led to attribution fix: rollback now judges
> `applied_error_ema` (error only while the learned map steers), not global
> tracking error. Battery-coupled state (effort_scale, trims) now relaxes
> toward neutral on process start (`battery_state_retention: 0.5`) after the
> sag session proved persistence hazard. CAVEAT: no champion-only control
> session in the small room; the within-session out-of-region driving (67%)
> served as the control. Stage C PREDICTION CRITERIA MET
> (2026-07-12/13 night): transient-weighted refit (fwd τ0.4, rev τ0.4+0.01 m)
> wins ALL transient comparisons — 6/6 offline held-out and 3/3 live drives
> (s10 0.131v0.137, s11 0.127v0.138, s13 0.094v0.101, the last under
> weak-pack stress) — with settled parity (worst −0.007). Next code
> milestone: apply effective-kappa to preview horizon + steering-transition
> speed logic (replace fixed lag constants), then D-style validation.
> Battery lesson codified: pack sags under LAUNCH load while cruise reads
> nominal (effort_scale ~1.01 yet 8 breakaway escalations) — effort-scale
> variance now re-inflates on power-cycle relaxation. Throttle (Stage F)
> remains the dominant KPI gap.
>
> **AMNESIA EXAM PASSED (2026-07-13, sessions 122211/123051/123837):**
> learned memory wiped AND config hand-measured values demoted to
> deliberately-wrong generic guesses (breakaway 1450/1550 vs true
> ~1414/1590; throttle map ~20 µs off; steering fallback geometry-generic;
> lags generic 0.25 s). From the birth certificate alone: **24/26 driven
> goals (92%) across three ordinary sessions — 9/10 → 7/8 → 8/8 perfect
> final round** — cusp endpoints down to 0.069 m, zero config edits, zero
> interventions, on a rug-perturbed floor. Convergence: breakaway walked
> the 36/43 µs lies to within ~5 µs of truth (~30 launches each); steering
> map median 0.078/0.113 1/m vs pre-exam truth (rug-contaminated
> comparison, inside the §16.2 ≤0.15 target), full ±0.6 trust earned after
> one session; cruise anchors, Rprop correction rates, polarities all
> re-learned. Caveats: session-2 steering partly rode the dense fallback
> (leak fixed for session 3 — exam script now demotes steering maps too);
> |κ|>0.6 stays fallback territory. Stage F final design = two-point
> learned line + failure-driven adaptation (2026-07-13 decision; probes
> retired to diagnostics; ArduPilot-verified pattern), validated in
> Session B (10/10 with mid-session rug) and by the exam. Refinement
> queue: launch-tail fixes (target-speed ramp, per-sample ladder cadence,
> prior+spread first rung).
>
> **HARDWARE SPLIT + EFFORT-NATIVE MIGRATION (2026-07-13):** the monolith is
> now four packages — `adaptive_ackermann` (controller + dispatcher + offline
> tools, hardware-free), `adaptive_ackermann_msgs` (`EffortCommand`:
> stamped normalized efforts in [-1,1], sign semantics LEARNED not declared),
> `pca9685_effort_driver` (dumb transport: piecewise-affine effort→µs from
> birth-certificate endpoints, 0.5 s reference-timeout deadman, neutral-on-
> error/shutdown, ~1 ms added latency measured), `ackermann_robot` (vehicle
> bringup: launch, Nav2 configs, birth certificate, `tf_odom_bridge` which
> replaces the controller's internal TF→/odom conversion). The controller
> never sees a pulse unit: all internals, configs, learned memory (runtime
> v8), and CSV columns are effort-denominated (throttle (us−1500)/320,
> steering /463 below / /537 above neutral); the birth certificate's
> actuator section is now driver-only, the controller consumes geometry +
> fingerprint. One-time converter `tools/convert_memory_to_effort.py`
> (round-trip verified ≤0.022 µs vs 1.24 µs tick); pre-v8 memory is
> preserved-aside, never overwritten. Deadman drill PASSED (SIGKILL
> controller → neutral 1463/1500 µs at exactly 0.5 s); esc-watchdog service
> now backstops the DRIVER process. Mirrors the ros2_control
> controller/hardware-interface split (researched: no ROS standard exists
> for signal-level actuator interfaces; theirs assumes calibrated encoder
> joints — exactly what this project cannot assume). Runtime memory restored
> from the 12:21 exam backup (444/145 breakaway obs) after a smoke-test
> save clobbered the live v7 file (hazard now guarded). Nothing else about
> the learning changed. **KPI re-verification PASSED same day (session
> 174238, fresh pack): 8/8 driven goals, endpoints 0.056–0.136 m, both
> multi-cusp reverse flips clean, steering prediction median 0.109 1/m,
> wrong-sign ≤3.1%, zero curvature saturation — no regression.** (An
> earlier attempt stalled at FULL authority — effort 1.0, two clean
> give-ups — which was the dead drive pack, not the split: the identical
> stack drove perfectly after charging, incidentally re-validating the
> behavioral exhaustion path end-to-end.)
>
> **Status 2026-07-13:** weak-plant adaptation VALIDATED (launch patience
> 3×, behavioral pack-exhaustion latch, achievability limiter; sessions
> 16-17 completed 12/13 driven goals on a dying pack that failed sessions
> 13-14). Stage E MECHANISM VERIFIED: between-goal envelope updates fire at
> the goal boundary via parameter services (drilled disarmed), geometry-
> scaled bootstrap replaces the 1.30 constant (wheelbase/tan 12° = 1.306 m).
> Note: four-branch min-trusted envelope currently yields radius ~1.61 m —
> correct §11.3 conservatism; expands with clean high-demand passes. Birth
> certificate implemented (config/birth_certificate.yaml → parameter
> overlay + vehicle fingerprint; learned memory from a different fingerprint
> is refused). Stage C dynamics applied to preview horizon.
>
> **AMNESIA DRESS REHEARSAL PASSED (2026-07-13, steering domain):** learned
> memory wiped (backup ~/.robot_backup_20260713_021344); robot booted on
> birth certificate + prior-only map (bootstrap trust ±0.35, geometric
> planner radius). Session 1 from blank: 7/10 driven goals including a
> multi-cusp flip. Post-session refit from post-wipe CSVs only
> (`steering_map_challenger.py --since 20260713_021344`): 496 samples →
> map within median 0.044 1/m of the 14.6k-sample truth. Session 2 on the
> self-learned map (evidence-shaped trust fwd −0.60..0.35, rev ±0.60):
> previously-failing maneuvers succeeded, 3.5× speedups; second refit
> (1,179 samples) → median 0.042 / p90 <0.10 / max 0.15 1/m vs truth with
> full branch coverage. §16.2 initial learning target (<=15%) met from
> ordinary driving alone. Rehearsal fixes: PAVA-tie knots are spread not
> dropped; sparse-side authority stays with the PRIOR (bootstrap trust),
> never silently the dense map. SCOPE CARVE-OUTS for the full exam:
> throttle maps + steering-lag constants remain config-side (Stage F),
> |kappa|>0.6 remains dense-map authority, polarity discovery uncoded
> (second-vehicle feature). The robot's operating memory is now the
> amnesia-relearned state — not restored from backup.

### Stage A — freeze and replay the baseline

- Keep `c61d041` and tag `experiment-20260712-135900` as the champion.
- Add a reproducible analysis command that scores steering prediction,
  endpoint tracking, launch peaks, saturation, and learner behavior from a bag.
- Build a held-out bag set covering forward/reverse, both turn signs, cusps,
  open space, and obstacle reactions.
- Record the exact current dense-map predictions as the benchmark.

Exit criteria:

- one command reproduces the baseline scorecard;
- no source changes are needed to evaluate a new learner;
- bags are split into estimator-training and held-out validation sets.

### Stage B — continuous steering-map challenger

- Implement a monotone learned steady-curvature curve in shadow mode.
- Seed it with a cautious generic prior, not the production dense map.
- Feed it eligible natural observations and identification probes.
- Compare its held-out curvature predictions against the current dense map.
- Persist estimate, uncertainty, knots, sample provenance, and validity domain.

Exit criteria:

- correct curvature sign across the exercised safe range;
- monotonic learned response;
- held-out median curvature error no worse than the dense-map champion;
- no dependence on scripted-path labels;
- repeatable results from a blank challenger state.

### Stage C — dynamic steering-state challenger

- Replace pure delay prediction in shadow with gradual effective steering
  state.
- Fit time and distance terms from ordinary command transitions and probes.
- Add hysteresis/load terms only if they improve held-out prediction.
- Report predicted steering transition completion rather than one delay time.

Exit criteria:

- better held-out transient-curvature prediction than the fixed-delay model;
- separately explains forward time response and reverse distance response;
- no double compensation between RPP lookahead, preview, and actuator model;
- stable prediction across at least three ordinary drives.

### Stage D — confidence-gated steering promotion

- Promote the learned map/dynamics first inside a well-observed low/mid
  curvature region.
- Keep the dense map as automatic rollback champion.
- Expand the applied region incrementally as evidence accumulates.
- Remove dense-map authority only after full exercised-range certification.

Exit criteria:

- tracking KPIs equal or improve across three comparable sessions;
- no increase in wrong-sign steering, endpoint failures, or saturation;
- rollback is tested deliberately in disarmed/replay conditions;
- no hand editing of steering knots.

### Stage E — learned reachable and planner envelopes

- Derive reachable curvature from the promoted steering model and safe pulse
  bounds.
- Derive trackable curvature from clean segment outcomes.
- replace fixed 1.30 m with wheelbase-scaled bootstrap plus learned envelope;
- apply planner updates between goals in the same process;
- log every plan with its model/envelope version.

Exit criteria:

- planner radius changes only from qualified evidence;
- a clean failure contracts faster than clean passes expand;
- unobserved branches remain conservative;
- no restart is required to use evidence on the next goal;
- automatic rollback restores the last good envelope.

### Stage F — throttle-map portability

- Retain the certified event-based breakaway learner.
- identify direction-specific steady response through small known excitation;
- replace this chassis's measured throttle maps with learned curves;
- preserve a shared short-term effort state for battery/surface/temperature;
- keep explicit sub-floor pulse-and-coast separate from ordinary endpoint
  control.

Exit criteria:

- blank-memory vehicle starts cautiously and learns both directions;
- launch peak and stall KPIs match the current baseline;
- steady-map estimates do not invert under terrain/closed-loop confounding;
- no throttle-map configuration edits across three sessions.

### Stage G — amnesia and second-vehicle exams

- Back up memory, remove it, and run normal supervised goals on this robot.
- Compare relearned maps/dynamics/envelopes with the historical measured truth.
- Install on the second chassis with only its birth certificate.
- Repeat normal goals, confidence expansion, and KPI scoring.

Exit criteria:

- both exams meet the definition of done;
- no scripted calibration route was required;
- no learned state was copied between vehicles;
- any unexercised domain remains explicitly uncertain rather than guessed.

## 16. Acceptance metrics

The universal learner is evaluated on both navigation quality and learning
truthfulness.

### 16.1 Navigation

| Metric | Initial target |
|---|---:|
| goal success rate on representative supervised course | >90% |
| manual interventions/restarts | 0 per accepted session |
| rolling cross-track error p90 | <=0.05 m |
| rolling heading error p90 | <=0.15 rad |
| clean cusp endpoint XY error | <=0.07 m |
| wrong-direction command persistence inside a segment | <0.75 s |
| no chronological progress before replan | <=6.0 s |
| post-monitor nonzero speed/curvature mutation | 0 |
| uncontrolled launch peak above 0.40 m/s at 0.30 request | 0 |

### 16.2 Learning

| Metric | Initial target |
|---|---:|
| steering-map sign errors in trusted region | 0 |
| held-out steady-curvature median error | <=15% initially, then improve |
| dynamic-model transient prediction | better than fixed-delay champion |
| false outward envelope expansion from contaminated evidence | 0 |
| applied values without confidence/provenance | 0 |
| config edits to learned maps between three accepted sessions | 0 |
| next-goal use of newly qualified planner evidence | yes, without restart |
| blank-memory relearning on robot 1 | pass |
| learning on robot 2 without copied memory | pass |

### 16.3 Safety and compute

| Metric | Required |
|---|---:|
| safe PWM bound violations | 0 |
| watchdog neutralization after owner loss | pass every release |
| Collision Monitor authority | never bypassed |
| learner CPU on Pi 5 | bounded; navigation lifecycle remains healthy |
| corrupted/missing memory fallback | cautious bootstrap |

## 17. Explicit design decisions

- No scripted calibration path is part of normal deployment.
- Polarity is not supplied: throttle and steering sign are identified from the
  first clean observed motion, under both-direction clearance until known.
- No end-to-end reinforcement learning.
- No requirement for wheel encoders, steering sensor, or battery sensor.
- No claim that passive closed-loop regression is sufficient.
- Small safety-gated probes during ordinary Nav2 motion are allowed.
- The steering curve is smooth and learned; stored knots are an
  implementation detail, not manual calibration.
- Steering response is a gradual hidden state with time/distance/history, not
  one fixed delay.
- Physical/reachable curvature and reliably trackable curvature are distinct
  learned quantities.
- A planner needs a bootstrap prior, but it is geometry-scaled and explicitly
  uncertain—not a universal 1.30 m constant.
- Learning updates the next safe planning/control boundary, not the middle of
  a committed maneuver.
- Safety limits do not adapt outward without supplied electrical bounds and
  qualified motion evidence.
- The current good controller remains the champion until challengers beat it
  on held-out evidence and real-drive KPIs.

## 18. Immediate next work

The next implementation should not directly replace the production steering
map. It should create the measurement and comparison foundation:

1. Build a repeatable bag-replay steering scorecard.
2. Define the versioned birth-certificate and learned-memory schemas.
3. Implement a monotone steady steering-map challenger in shadow mode.
4. Re-enable only safety-gated, low-amplitude steering probes needed for
   identification during ordinary goals.
5. Fit and compare a gradual time/distance steering-state model against the
   current fixed-delay approximation.
6. Promote nothing until held-out replay and three comparable drives show no
   regression.

This ordering preserves the robot that now drives reasonably well while
turning its current hard-coded knowledge into testable bootstrap evidence. It
also creates the shortest credible path to the actual product: a robot that
learns how its own Ackermann chassis moves by doing the work it was built to
do—navigating to goals.
