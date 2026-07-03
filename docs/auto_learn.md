# Auto-Learn Project — Status & Session Log

*Last updated: 2026-07-02 (evening session). Living document for the
"robot gets more accurate as it drives" effort.*

## Goal

The robot should continuously calibrate itself from normal driving: the
effort→motion map (deadbands, gains), eventually steering, using its own
sensors as ground truth — instead of the hand-tuned constants in
`config/*.yaml` that go stale every time hardware, battery, or terrain
changes. No wheel encoders exist; lidar ICP + SLAM + D435i IMU are the only
feedback.

---

## Where we are: PHASE 1 DEPLOYED (observe-only), 2026-07-02

### What runs now

- **`auto_calib` node** (`ackermann_robot/auto_calib.py`, in `drive.launch.py`
  by default, `auto_calib:=false` to disable, ~2% core). Observe-only:
  - Learns **forward breakaway deadband** from motion-onset events during
    stiction-kick ramps. Reports median + IQR every 15 s.
  - Collects **steady (effort, speed) pairs** per direction, fits a line, but
    publishes a **trust flag** — see "why naive learning fails" below.
  - Publishes `/auto_calib/params`; logs via
    `journalctl -u ackermann-drive | grep auto_calib`.
- **EKF at 30 Hz** (was 20; measures ~27). The EKF predicts between the 10 Hz
  sensors — fresher velocity for the speed PI / watchdog / interlock.
- **Vibration metric** in `cmd_vel_to_effort` (accel-envelope from the 30 Hz
  IMU feed, last field of `/cmd_vel_to_effort/debug`). Observe-only: gathering
  data to pick thresholds for a chassis-is-moving bit.

### First real finding (validated offline on 13 of tonight's logs)

**Forward breakaway ≈ 0.65 raw** (51 events, median 0.65, IQR 0.64–0.68;
odometry lag inflates ~0.05) vs `throttle_deadband_fwd: 0.45` in the yaml.
The feedforward for 0.3 m/s lands at ~0.61 — *at* breakaway — which is why
forward starts stall until the stiction kick rescues them. **Not yet applied
to the yaml** — first action of the next session after confirming with live
auto_calib reports.

### Why naive "learn while driving" fails (measured, not theoretical)

Both failure modes were hit on tonight's real logs before the node design:

1. **Dynamic regression** (fit dv/dt = k1·(u−u0) − k2·v): biased by the
   closed loop — the PI deliberately anti-correlates effort with speed.
   Fits gave u0≈0.84, negative drag. Textbook identification-under-feedback.
2. **Steady-state regression** (fit v = G·(u−u0) on cruise points): biased by
   **terrain selection** — the PI settles at high effort exactly where ground
   is hard, so steady pairs trace the terrain distribution, not the motor
   curve. Fitted G came out *negative* on 91 real pairs.

What survives closed loop: **event-based estimators** (breakaway = motion
onset, an event not an equilibrium) and **known-input correlation** (phase 2).

---

## Roadmap

- **Phase 1 (DONE, observing):** breakaway deadband + steady pairs + trust
  flags + vibration metric. Validate over a few normal drives.
- **Phase 2 (next):** active excitation — a small known feedforward dither
  (±0.02–0.03 effort, sub-Hz) injected by `cmd_vel_to_effort`, and auto_calib
  correlates response against it (lock-in style). This identifies the true
  speed gain G per direction despite the closed loop. Then: vibration-bit
  thresholds → wire into watchdog (fast blind-runaway trip) and kick veto.
- **Phase 3:** close the loop — feedforward params (deadband, max_speed,
  max_speed_rev) derived live from the learned model, bounded, with the
  watchdog underneath. Retire hand tuning.
- **Phase 4 (optional):** persist SLAM maps across sessions (localization
  mode — accuracy accumulates across days); Nav2 MPPI controller with the
  learned model *only if* RPP + good feedforward still isn't enough
  (MPPI is the most expensive Nav2 controller; Pi budget is ~1 spare core).

### Explicitly rejected

- **End-to-end RL:** needs thousands of trials incl. crashes; Pi 5 has no
  compute headroom (DR-SPAAM benchmarked 0.9 Hz here). Model-ID + adaptive
  feedforward gets the benefit without the risk.
- **Visual odometry for more feedback rate:** benchmarked on this Pi
  (2026-06-09): rgbd_odometry caps at 10–12 Hz with quality collapse at 6×
  the CPU of lidar ICP. Camera fps is not the bottleneck; compute is.
  The C1 lidar is hardware-fixed at 10 Hz.

### CPU diet (once auto_calib is trusted)

`drive_logger` (~19% core) → default off; `use_floor_scan:=false` (~30% core)
where low obstacles don't matter. Camera cannot go entirely: the EKF's gyro
fusion and the vibration metric live in it (IMU-only camera ≈ 10%).
icp_odometry also dies without the camera — unresolved bug, see
drive.launch.py comment (2026-06-14).

---

## Session log 2026-07-02 — the ESC reverse saga (context for why auto-learn)

Drivetrain was converted from H-bridge to a Hobbywing QuicRun-1060-style
brushed ESC (ch 14, committed `814fd1d` this session, code from 2026-06-15).
A day of field tests exposed, layer by layer:

1. **Blind reverse runaway** (`9c5c908`): lidar ICP couldn't track fast
   reverse (odom read ~0 / phantom-positive while backing fast) → speed PI +
   stiction kick stacked effort to raw −0.8. Fixes: odometry-distrust
   watchdog (2 s commanded-but-unmeasured motion → neutral) + per-direction
   raw ceilings.
2. **ESC reverse cliff** (`84d8246`): SLAM-ground-truth showed raw −0.61…−0.64
   ≈ 0.25 m/s but −0.69 ≈ 0.7–0.9 m/s. Ceiling calibrated to the good side.
3. **Non-deterministic engagement** (`95009a0`): same −0.6 effort → 0.3 m/s or
   NOTHING, coin-flip. Cause: ESC brake/reverse state machine. Fix: driver
   double-tap (brake tap → neutral gap → reverse). Also fixed watchdog
   latching across stale-cmd gaps (bricked all motion).
4. **True reverse threshold** (`7ab33ce`): raw sweep below the old floor —
   nothing at −0.40, moves at −0.50, and **constant effort = constant
   acceleration** (reverse is torque command: −0.50 held 3 s went 0.2→0.9
   m/s). deadband_rev 0.55 (H-bridge fossil) → 0.45, max_speed_rev → 2.0.
5. **PI wound the wrong way** (`0461d96`): during the double-tap's 300 ms the
   integral wound deeper-into-reverse. Reverse trim is now brake-only, kick
   forward-only, ceiling −0.55.
6. **Watchdog blocked stall escape** (`89e062c`): a forward-stall trip carried
   across Nav2's sign flip to reverse. Sign flip now re-arms. Plus reverse
   ff −0.51 (max_speed_rev 2.6), PI brake authority 0.2.
7. **Self-calibration phase 1** (`07d8095`): this document's subject.

**Hardware TODO:** move the ESC's mode jumper to **Fwd/Rev (single-click)** —
removes the double-tap state machine at the source (manual: no jumper on the
Mode header = single-click). Then set `esc_double_tap: false`. ESC startup
beeps: 2 short = 2S LiPo detected, 1 long = self-test OK (normal, not a fault).

**Also known:** ESC reverse hardware current is half of forward (30 A vs 60 A
cont.); throttle range self-calibrates at power-on (power ESC with driver
already at neutral).

## How to check what it's learning

```bash
journalctl -u ackermann-drive -f | grep auto_calib     # live reports
ros2 topic echo /auto_calib/params                     # machine-readable
```

Report format: `FWD breakaway: n=.. median=..` (want n>20 before trusting),
`FWD/REV steady: n=.. G=.. u0=.. trust=..` (trust LOW until phase-2 dither).
