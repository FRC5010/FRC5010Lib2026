# Mechanisms — YAMS + LQR

This library wraps [YAMS](https://github.com/Yet-Another-Software-Suite/YAMS)
("Yet Another Mechanism System", vendordep `yams.json`, version 2026.4.10.3) to give
teams declarative, simulation-ready mechanism subsystems with **state-space LQR
control** and live NetworkTables tuning.

## Architecture

```
Settings (public fields — robot-specific numbers ONLY)
   │
   ▼
Common wrapper (org.frc5010.common.mechanisms)        Controller
 ├── YamsElevator        ─ ELEVATOR-type LQR (meters) + trapezoid profile + kG FF
 ├── YamsArm             ─ ARM-type LQR (rotations) + trapezoid profile + kG·cos(θ) FF
 ├── YamsPivot           ─ ARM-type LQR (no gravity FF) — turrets, hoods, wrists
 ├── YamsFlywheel        ─ FLYWHEEL-type LQR (velocity, plant-inversion FF built in)
 ├── YamsDoubleJointedArm ─ profiled PID per joint (LQR doesn't model coupled joints)
 └── YamsDifferentialMechanism ─ profiled PID per motor (tilt + twist)
   │
   ▼
YAMS mechanism (Elevator / Arm / Pivot / FlyWheel / DoubleJointedArm / DifferentialMechanism)
 ├── SmartMotorController wrapper (TalonFX preferred; TalonFXS / SparkMax / SparkFlex via MechanismMotor.Vendor)
 ├── RIO-side closed loop in a 20 ms WPILib Notifier (LQR always runs on the RIO)
 └── Built-in physics sim (ElevatorSim / SingleJointedArmSim / DCMotorSim) + Mechanism2d
```

**Common vs robot-specific:** all control logic, LQR construction, and tuning plumbing
live in `org.frc5010.common.mechanisms`. Team code only fills in a `Settings` object —
see the six examples in `src/main/java/frc/robot/mechanisms/` (`ExampleElevator`,
`ExampleArm`, `ExampleTurret`, `ExampleShooter`, `ExampleDoubleJointedArm`,
`ExampleDifferentialWrist`), all configured with Kraken X60s on TalonFX, CAN IDs 21–28.

## Adding a mechanism (short version — see `/new-yams-mechanism` for the playbook)

```java
public class MyElevator extends YamsElevator {
  public MyElevator() { super(settings()); }
  private static Settings settings() {
    var s = new Settings();
    s.name = "Elevator";
    s.canId = 21;                                   // TalonFX CAN ID
    s.motorModel = DCMotor.getKrakenX60(1);
    s.gearReductionStages = new double[] {4, 3};    // 12:1
    s.drumCircumference = Inches.of(0.25 * 22);     // travel per drum rotation
    s.carriageMass = Kilograms.of(6.0);
    s.minHeight = Meters.of(0);  s.maxHeight = Meters.of(1.5);
    s.maxVelocity = MetersPerSecond.of(0.9);        // MUST be physically achievable!
    s.kG = Volts.of(0.19);                          // from sysId() on a real robot
    return s;
  }
}
```

Commands: `goToHeight(Distance)` / `goToAngle(Angle)` / `goToSpeed(AngularVelocity)`,
`setDutyCycle(...)`, `sysId()`; triggers: `isAtHeight` / `isAtAngle` / `isAtSpeed`;
mechanism access: `getMechanism()` / `getMotor()`.

## Why LQR (and what to know)

The YAMS docs and examples mostly show plain/profiled **PID** — those examples are out
of date relative to what the library can do. YAMS ships `yams.math.LQRController`
(WPILib `LinearSystemLoop` = LQR gain + Kalman filter + plant-inversion feedforward),
plugged in via `SmartMotorControllerConfig.withClosedLoopController(LQRController)`.
LQR is tuned with *physical tolerances*, not abstract gains:

| Weight | Meaning | Default |
|---|---|---|
| `qelmsPosition` | position error you tolerate (m or rot). Smaller = more aggressive | 2 in / 1.5° |
| `qelmsVelocity` | velocity error you tolerate (m/s or rot/s) | 0.5 m/s / 20°/s |
| `relms` | control effort you allow (volts). Smaller = gentler | 12 V |

All three are live-tunable under `/Tuning/<name>/lqr_*` (AdvantageScope / Shuffleboard).
On change, the wrapper rebuilds the regulator and restarts the loop at the current
state. See `/tune-mechanism` for the full tuning workflow.

LQR supports exactly three plants — **ELEVATOR**, **ARM** (also used for pivots), and
**FLYWHEEL**. DoubleJointedArm and DifferentialMechanism are coupled multi-motor
systems the LQR types don't model, so those wrappers use profiled PID with
`TunableGains` (`/Tuning/<name>/<joint>_k*`).

## Gotchas (hard-won)

1. **YAMS 2026.4.10.3 ships a broken Kalman filter for ARM/ELEVATOR LQR.**
   `LQRConfig.getKalmanFilter` passes WPILib's 2-output plant with a 1-dim measurement
   noise vector; the native DARE solver reads garbage ("R was not symmetric", or—worse—
   silently useless gains, mechanism barely moves). Fixed upstream on main but not
   released. `MechanismLqrConfig` overrides `getKalmanFilter` with the correct
   `plant.slice(0)`. **Always build LQR configs through `MechanismLqrConfig`**, never
   raw `LQRConfig`, or retuning will go through the broken path. FLYWHEEL (1-state) is
   unaffected.

2. **The LQR closed loop must run in the units of its plant.** The ELEVATOR plant is in
   meters: `withMechanismCircumference(...)` + a linear trapezoid profile (or
   `ElevatorFeedforward`) put the YAMS loop in linear mode. Without it, the loop feeds
   rotations into a meters model and slams the mechanism.

3. **`withClosedLoopController(lqr)` must be LAST in the config chain.** Every
   PID-flavored `withClosedLoopController(...)` overload clears the stored LQR.

4. **Profile cruise velocity must be physically achievable** (below
   free speed ÷ gearing × circumference). The profile doesn't know about saturation; if
   it outruns the mechanism the LQR chases an unreachable reference and overshoots
   badly. Same for arms/pivots in rotational units.

5. **kG feedforward is required for elevators/arms.** The linearized LQR plants have no
   gravity term and LQR has no integrator, so uncompensated gravity = steady-state
   error. Get kG from `sysId()` on the real robot (the examples compute it from motor
   constants for sim). YAMS only applies `ArmFeedforward` when a motion profile is
   configured — keep the trapezoid profile on arms.

6. **No `withClosedLoopTolerance` with LQR** — YAMS throws by design.

7. **Tests: never use synchronous `SimHooks.stepTiming` with YAMS mechanisms.** The
   YAMS closed loop lives in a Notifier; synchronous stepping deadlocks waiting for an
   alarm ack. Pump with `stepTimingAsync(0.02)` + ~10 ms real sleep
   (see `YamsMechanismsFunctionalTest.runScheduledFor`).

8. **Tests: feed the Phoenix enable watchdog.** TalonFX outputs silently neutral (duty
   reads 0, no error) if fresh DS packets stop for ~100 ms of *real* time. Call
   `DriverStationSim.notifyNewData()` + `Unmanaged.feedEnable(...)` every test cycle.

9. **The released jar's API differs from YAMS GitHub main** (e.g. `withSoftLimit` vs
   `withSoftLimits`, `ArmConfig.withHardLimit` vs `withHardLimits`). When in doubt,
   `javap` the jar in the Gradle cache, not the GitHub source.

## Functional tests

`src/test/java/frc/robot/mechanisms/YamsMechanismsFunctionalTest.java` builds each
example subsystem for real (TalonFX wrapper + YAMS sim + closed loop), schedules its
public command, and asserts the mechanism reaches the commanded state — including one
live-retune-over-NT test. They run in the normal `./gradlew test` suite.

## Real-robot bring-up

1. Copy an example, set real CAN IDs / gearing / masses / limits.
2. Verify in sim (`./gradlew test`, or `simulateJava` and watch the Mechanism2d widget
   under SmartDashboard → `<name>/mechanism`).
3. On the robot: run `sysId()` to characterize kG (and kS/kV for reference), update
   the settings.
4. Tune qelms/relms live over NT (`/tune-mechanism`), then bake the final values into
   the settings.
