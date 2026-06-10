# /tune-mechanism — Tune a YAMS mechanism (LQR or PID)

Agent playbook for tuning the mechanism subsystems built on
`org.frc5010.common.mechanisms` (see [docs/mechanisms.md](../../docs/mechanisms.md)).

## Identify the controller first

- `YamsElevator` / `YamsArm` / `YamsPivot` / `YamsFlywheel` → **LQR** —
  tune tolerances under `/Tuning/<name>/lqr_*`.
- `YamsDoubleJointedArm` / `YamsDifferentialMechanism` → **profiled PID** —
  tune gains under `/Tuning/<name>/*_kP|kI|kD`.

## LQR tuning workflow

LQR has no kP/kI/kD. You tune three *physical tolerances*; the regulator computes
optimal gains from the plant model (motor, gearing, mass/MOI):

| NT entry | Meaning | Move it when... |
|---|---|---|
| `lqr_qelmsPosition` | allowed position error (m or rotations) | too sluggish → smaller; oscillates → larger |
| `lqr_qelmsVelocity` | allowed velocity error (m/s or rot/s) | overshoot/ringing → smaller (more damping) |
| `lqr_relms` | allowed control effort, volts | violent/brownouts → smaller; weak → keep 12 |

Procedure (sim: `./gradlew simulateJava`; real robot: tethered, mechanism clear):
1. Open AdvantageScope/Shuffleboard → `/Tuning/<name>/`. Changes apply live — the
   wrapper rebuilds the regulator and restarts the loop at the current state.
2. Command a mid-range setpoint, observe the YAMS telemetry (position vs setpoint).
3. Adjust one weight at a time, factor-of-2 steps. Don't go below ~1 in / ~1° position
   tolerance: the RIO loop has 20–40 ms delay and tighter weights oscillate.
4. If it *never* settles at the target (steady offset): that's gravity, not weights —
   fix `kG` (step below), LQR has no integrator.
5. **Bake the final values into the `Settings`** in the team's subsystem class —
   NT tunables reset on reboot.

### Plant accuracy beats weight tuning
The LQR is only as good as its model. If behavior is wildly off, re-check settings:
gearing stages, carriage mass / arm length+mass / MOI, drum circumference. A 2x mass
error degrades control more than any weight change can fix.

### kG characterization (real robot)
1. Run the wrapper's `sysId()` command (quasistatic+dynamic, logs via WPILib SysId).
2. Load the log in the SysId tool → read kG (elevator) or kG of `ArmFeedforward`.
3. Set `settings.kG`. (kS/kV are informational — the LQR loop provides
   plant-inversion feedforward itself; do NOT add a kV feedforward on top.)

### Profile limits
`maxVelocity` must stay below free speed ÷ gearing × circumference (or the rotational
equivalent). If the mechanism saturates at 12 V chasing the profile, lower
`maxVelocity`/`maxAcceleration` — the symptom is big overshoot at the end of motion
with the LQR weights blameless.

## Profiled PID tuning (DJA / differential)

`/Tuning/<name>/lowerJoint_kP` etc. apply live (config re-applied to the motor on
change — lands onboard for TalonFX). Standard PID workflow: raise kP until slight
overshoot, add kD to damp; kI almost never needed with the trapezoid profile. Bake
final gains into `Settings`.

## Validate

`./gradlew test --tests "frc.robot.mechanisms.YamsMechanismsFunctionalTest"` must stay
green with the baked-in values. If tuning sessions changed defaults, update the
matching tolerance/time budget in the test rather than deleting assertions.
