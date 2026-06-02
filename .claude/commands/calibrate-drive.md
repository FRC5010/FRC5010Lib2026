# /calibrate-drive — Motor Calibration Agent Playbook

Use this playbook when a team needs to calibrate swerve drive motor feedforward gains (kS, kV, kA) for profiled velocity control or torque-current FOC control.

---

## Step 0 — Identify the target hardware

Determine which module type is in use from `SwerveConstants`:

| `ModuleType` | Gain storage | Control modes |
|---|---|---|
| `SIM` / `SPARK_MAX` / `SPARK_TALON` | `DriveConstants.java` | Voltage only |
| `TALON_FX` | TunerX-generated `SwerveModuleConstants` | Voltage or TorqueCurrentFOC |

---

## Step 1 — Run the simulation ramp calibration

Execute the Layer 2 calibration tests to confirm the routine works:

```bash
./gradlew test --tests "org.frc5010.common.subsystem.DriveCalibrationTest"
./gradlew test --tests "org.frc5010.common.subsystem.DriveCalibrationSimPhysicsTest"
```

Read the console output for the `toConfigString()` values. The physics test
(`estimatedGainsAchieveTargetVelocity`) must pass before proceeding to hardware.

---

## Step 2 — Identify the current gain storage location

**Spark-based modules**: open `DriveConstants.java` and locate:
```java
public static final double driveKs = ...;
public static final double driveKv = ...;
public static final double driveKp = ...;
```

**TalonFX modules**: open the TunerX project JSON in `vendordeps/` or ask the team where
their TunerX-generated constants live. Look for `Slot0kS`, `Slot0kV`, `Slot0kP` inside
`SwerveModuleConstantsFactory`.

---

## Step 3 — Run WPILib SysId on real hardware (for kA)

If the team has access to the physical robot:

1. Trigger `drive.sysIdFullRoutine()` via SmartDashboard or a button binding.
2. Capture the `.wpilog` from `logs/` on the RoboRIO.
3. Load it in the [WPILib SysId Analyzer](https://github.com/wpilibsuite/sysid).
4. Note kS, kV, kA and their R² values.

If kA < 0.02 V·s²/rad, it is safe to leave it at 0 for most use cases.

---

## Step 4 — Patch gain constants

For `DriveConstants.java`, use the Edit tool to update the three fields:
```java
public static final double driveKs = <kS from calibration>;
public static final double driveKv = <kV from calibration>;
public static final double driveKp = <suggestedKP from CalibrationResult>;
```

For TunerX, provide the updated values to the team to enter in Tuner X GUI —
agents cannot interact with Phoenix Tuner X directly.

---

## Step 5 — Verify with full test suite

```bash
./gradlew test
```

All tests must pass. The calibration tests (`DriveCalibrationTest`,
`DriveCalibrationSimPhysicsTest`) verify the routine itself. The existing
`AkitSwerveDriveTest` and `AkitSwerveDriveSimPhysicsTest` tests verify that
no regressions were introduced.

---

## Step 6 — Torque-current FOC (TalonFX only, optional)

If the team wants `VelocityTorqueCurrentFOC`:
1. In TunerX, set `DriveMotorClosedLoopOutput = TorqueCurrentFOC`.
2. Keep kV and kA values from Step 3.
3. Change kS from volts to amps (typically 2–8 A for FRC drive motors).
4. Increase kP by 5–20× relative to the voltage-mode value.
5. Validate with a straight-line drive test: velocity error should be < 5% at full speed.

See `docs/calibration.md` for the full background on torque-current vs voltage mode.

---

## Troubleshooting quick reference

| Problem | Fix |
|---------|-----|
| Tests disabled-robot fail | Add `enableTeleop()` before the ramp loop |
| R² < 0.85 | Increase `SETTLE_CYCLES` to 40+ and re-run |
| `estimatedGainsAchieveTargetVelocity` fails | Check that `simulationPeriodic()` is called before `periodic()` in the Layer 3 cycle step |
| kV unchanged after editing `DriveConstants` | Rebuild with `./gradlew build`; the constant is read at module construction time |
