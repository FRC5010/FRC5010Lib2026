# Motor Controller Calibration

This guide walks students and agents through calibrating the swerve drive motor controllers for accurate velocity control. Follow the steps in order: simulation first, then real hardware.

---

## Why calibrate?

Velocity control on an FRC swerve drive relies on a **feedforward model** that predicts how much voltage (or torque-current) is needed to spin a wheel at a target speed. Without accurate feedforward, the closed-loop PID controller has to do all the work, leading to:

- **Under-shoot**: robot moves slower than commanded; path-following errors grow
- **Oscillation**: PID fights a poorly-tuned feedforward, causing hunting around the setpoint
- **Inconsistency**: gain required at low speeds differs from high speeds, so no single kP works well

Calibration identifies three feedforward constants and suggests a starting kP.

---

## Motor feedforward theory

The voltage-mode feedforward model is:

```
V = kS · sgn(ω) + kV · ω + kA · α
```

| Parameter | Units | Meaning |
|-----------|-------|---------|
| **kS** | V | Minimum voltage to overcome static friction (dead-band) |
| **kV** | V·s/rad | Back-EMF slope — how many extra volts per rad/s of wheel speed |
| **kA** | V·s²/rad | Inertia — extra volts needed to accelerate; often small enough to ignore |
| **kP** | V/(rad/s) | PID proportional gain — corrects steady-state error feedforward leaves |

For a first calibration, identify **kS** and **kV** via the ramp routine below, set **kA = 0**, then tune **kP** on the real robot.

---

## Control modes

### Profiled velocity control (voltage mode)

All supported hardware (TalonFX, SparkMax, SparkFlex) supports voltage-mode velocity control. This is the default and the right starting point.

In `DriveConstants` (for Spark-based modules), the relevant fields are:
```java
public static final double driveKs = 0.0;  // → set this from calibration
public static final double driveKv = 0.1;  // → set this from calibration
public static final double driveKp = 0.0;  // → set this from kP tuning step
```

### Torque-current FOC (TalonFX Kraken X60 / Falcon 500 only)

With `DriveMotorClosedLoopOutput.TorqueCurrentFOC` set in TunerX, the drive request switches from `VelocityVoltage` to `VelocityTorqueCurrentFOC`. The constants change meaning slightly:

| TunerX Slot0 field | Voltage mode | TorqueCurrent mode |
|--------------------|-------------|-------------------|
| kS | Static friction volts | Slip current (A) — threshold below which the motor doesn't move |
| kV | V·s/rad (back-EMF) | Same V·s/rad — the firmware converts internally |
| kA | V·s²/rad | Same V·s²/rad |
| kP | V/(rad/s) | A/(rad/s) — typically 5–20× larger than voltage mode |

**Recommendation**: identify kV and kS in voltage mode using the steps below, then use them as starting values for torque-current mode. Expect to re-tune kP because the loop gain is different.

---

## Step 1: Run the ramp calibration in simulation

The `MotorCalibrationRoutine` class applies a sequence of open-loop voltages, waits for the motor to reach steady state, then fits `V = kS + kV · ω` via least-squares regression.

### Reference test (Layer 2 — DCMotorSim)

Open `src/test/java/org/frc5010/common/subsystem/DriveCalibrationTest.java` and run:

```bash
./gradlew test --tests "org.frc5010.common.subsystem.DriveCalibrationTest"
```

The test `driveCalibrationFitProducesPositiveGains` exercises the full routine and checks that kV > 0 and kS ≥ 0. The test `driveKVIsInPhysicallyPlausibleRange` confirms the number is in a realistic range for FRC swerve hardware.

### Physics validation (Layer 3 — IronMaple)

```bash
./gradlew test --tests "org.frc5010.common.subsystem.DriveCalibrationSimPhysicsTest"
```

The key test is `estimatedGainsAchieveTargetVelocity`: it runs the full calibration, computes a feedforward voltage for 5 rad/s, applies it open-loop, and verifies the robot actually reaches within 20% of that speed. If this test passes, the routine is producing usable gains.

### Using the routine in your own code

```java
// inside a test, visual test, or calibration command:
MotorCalibrationRoutine routine = MotorCalibrationRoutine.collectDriveRamp(
    drive,
    new double[]{1.0, 2.0, 3.0, 4.0, 5.0, 6.0},  // voltage steps (V)
    30,                                              // settle cycles per step (30 × 20 ms = 0.6 s)
    () -> { drive.simulationPeriodic(); drive.periodic(); stepOneCycle(); }
);
CalibrationResult result = routine.fit();
System.out.println(result.toConfigString());
// → kS=0.1012 V, kV=0.0934 V·s/rad, kA=0.0000 V·s²/rad  (R²=0.9981)  → suggested kP=1.0706
```

---

## Step 2: Run WPILib SysId on real hardware (for kA)

The ramp routine identifies kS and kV accurately. To also identify **kA** (inertia / acceleration gain), use WPILib's built-in SysId routine which is already wired into `AkitSwerveDrive`:

1. **Deploy** to the robot and connect to Driver Station.
2. **Open Shuffleboard** or SmartDashboard and bind commands to buttons, or trigger via NT4.
3. **Run** `drive.sysIdFullRoutine()` — this executes quasistatic (forward + reverse) then dynamic (forward + reverse) tests, logging all data to AdvantageKit.
4. **Capture the log** from `logs/` on the RoboRIO.
5. **Open** [WPILib SysId Analyzer](https://github.com/wpilibsuite/sysid) and load the `.wpilog`.
6. **Select** the drive mechanism and read off kS, kV, kA, and R².

> The `sysIdFullRoutine()` has a built-in 1-second coast-down pause before each test segment, 10-second quasistatic ramps, and 4-second dynamic steps. Make sure the field is clear for ~2 robot lengths in each direction.

---

## Step 3: Apply gains to the motor controller

### SparkMax / SparkFlex (NEO / NEO Vortex)

Update `DriveConstants.java`:
```java
public static final double driveKs = 0.12;   // from SysId / ramp routine
public static final double driveKv = 0.093;  // from SysId / ramp routine
public static final double driveKp = 0.05;   // start here; tune in Step 4
```

Rebuild and deploy. The changes take effect immediately on robot startup.

### TalonFX via TunerX (Falcon 500 / Kraken X60)

1. Open **Phoenix Tuner X** → your robot → Swerve Project.
2. Select the **Drive Motor** slot in your swerve module config.
3. Update **Slot 0**:
   - `kS` ← kS from calibration
   - `kV` ← kV from calibration (in V·s/rad — Tuner X accepts this directly)
   - `kA` ← kA from SysId, or 0 if skipping
   - `kP` ← suggested kP from `CalibrationResult.suggestedKP()` as a starting point
4. Click **Apply** and verify in the **Self Test Snapshot**.

For **torque-current mode**, set `DriveMotorClosedLoopOutput = TorqueCurrentFOC` in TunerX before deploying. Increase kP to 5–20× the voltage-mode value as a starting point.

---

## Step 4: Fine-tune kP with TunableGains

`TunableGains` (already used in the codebase) publishes gains to SmartDashboard and watches for live updates:

```java
// In your module IO or subsystem init:
TunableGains driveGains = new TunableGains("Drive", "driveMotor", kP, 0.0, 0.0, kV);

// In periodic():
if (driveGains.hasChanged()) {
    driveController.setP(driveGains.kP());
    driveController.setFF(driveGains.kFF());
}
```

**Tuning procedure:**
1. Command the robot to drive at a constant speed (e.g. 2 m/s forward).
2. Watch the velocity error on SmartDashboard (or in AdvantageScope from the log).
3. Increase kP until the velocity tracks the setpoint with small steady-state error.
4. If oscillation appears, halve kP.
5. Persist the final value back into `DriveConstants` or TunerX.

---

## Troubleshooting

| Symptom | Likely cause | Fix |
|---------|-------------|-----|
| Ramp routine returns kV = 0 | Robot is disabled; `runCharacterization()` is a no-op | Call `enableTeleop()` before the test / enable DS |
| kV is extremely large (> 5) | Velocity samples are in motor-shaft units, not wheel units | Check `Module.getDriveFFCharacterizationVelocity()` — it returns `inputs.driveVelocityRadPerSec` at the wheel |
| R² < 0.8 | Not enough settle time; motor still in transient | Increase `SETTLE_CYCLES` to ≥ 40 |
| `estimatedGainsAchieveTargetVelocity` fails | kA is significant (heavy robot) | Add a small kA term and use 50+ cycles for the feedforward test |
| SysId produces inconsistent kV vs ramp routine | Battery voltage varied during SysId test | Re-run SysId with a fresh battery; battery compensation improves repeatability |
| Oscillation at high speed only | kP too high relative to kV | Reduce kP by 30%; the feedforward should carry most of the load |
| Robot drifts left/right during straight-line drive | Module-to-module kV variation | Characterise each module individually (use per-module IO directly) |

---

## Related slash commands

- `/calibrate-drive` — agent-guided step-by-step calibration session
- `/diagnose-log` — analyze `.wpilog` for anomalies after a SysId run
- `/validate-replay` — confirm replay fidelity after updating gain constants
