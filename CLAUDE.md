# FRC5010Claude — Claude Code Project Briefing

## What this project is
WPILib 2026.2.1 FRC robot **swerve drive library**. Teams configure one `SwerveConstants` record, call `SwerveFactory.build()`, and get a fully wired `AkitSwerveDrive` subsystem that works in REAL, SIM, and REPLAY modes with AdvantageKit.

**Build system:** Gradle + GradleRIO 2026.2.1 · **Java 17** · `./gradlew test` (Windows: `gradlew.bat test`)

---

## Architecture in one diagram

```
SwerveConstants (immutable record, Builder)
        │
        ▼
SwerveFactory.build()              ← SIM: IronMaple physics (SwerveDriveSimulation)
SwerveFactory.buildWithoutPhysics() ← SIM: WPILib DCMotorSim (no YAGSL overhead)
        │
        ▼
AkitSwerveDrive (SubsystemBase)
 ├── GyroIO  ──► GyroIOSim (kinematics integration, buildWithoutPhysics)
 │               GyroIOSimPhysics (reads GyroSimulation, build)
 │               GyroIONavX / GyroIOPigeon2 (REAL)
 └── Module[4]
      └── ModuleIO ──► ModuleIOSim (DCMotorSim, buildWithoutPhysics)
                       ModuleIOSimPhysics (IronMaple GenericMotorController, build)
                       ModuleIOTalonFXReal / ModuleIOSparkTalon (REAL)
```

**Critical distinction — `instanceof GyroIOSim` in `AkitSwerveDrive.periodic()`:**
- `buildWithoutPhysics()` uses `GyroIOSim` → the kinematics-fallback branch fires.
- `build()` uses `GyroIOSimPhysics` (a separate class) → that branch does NOT fire. Heading comes from the physics engine's `GyroSimulation`.

---

## Test pyramid (48/48 passing as of 2026-05-24)

| Layer | Class | Factory method | IO impl | Tests |
|-------|-------|----------------|---------|-------|
| 1 — unit | `SwerveConstantsTest`, `SwerveFactoryModeTest`, `TunableGainsTest` | — | — | 33 |
| 2 — subsystem sim | `AkitSwerveDriveTest` | `buildWithoutPhysics()` | `ModuleIOSim` | 8 |
| 3 — physics integration | `AkitSwerveDriveSimPhysicsTest` | `build()` | `ModuleIOSimPhysics` | 7 |

All tests extend `SimTestBase` (deterministic FPGA clock via `SimHooks`).

---

## Per-cycle call order — Layer 3 tests (IronMaple)

```java
drive.runVelocity(speeds);   // 1. queue voltage commands to physics controllers
drive.simulationPeriodic();  // 2. advance dyn4j world: 5 sub-ticks × 4 ms = 20 ms
drive.periodic();            // 3. read updated module caches → pose estimator
stepOneCycle();              // 4. advance FPGA clock 20 ms
```

**Wrong order = stale data.** `periodic()` reads IronMaple module position caches. Those caches are only refreshed by `simulationPeriodic()` sub-ticks. If you call `periodic()` first, it reads the zero-filled initial caches and no motion appears.

Layer 2 tests (`buildWithoutPhysics`) don't need `simulationPeriodic()` — `ModuleIOSim.updateInputs()` calls `driveSim.update(0.02)` internally.

---

## SimulatedArena singleton — test isolation

`SimulatedArena` is a static singleton. Every `SwerveFactory.build()` call registers a new `SwerveDriveSimulation` body into the current arena. Without cleanup, each test accumulates stale bodies.

**Required teardown pattern in every Layer 3 `@AfterEach`:**
```java
SimulatedArena.getInstance().shutDown();         // removes all dyn4j bodies
java.lang.reflect.Field f = SimulatedArena.class.getDeclaredField("instance");
f.setAccessible(true);
f.set(null, null);                               // null the singleton → next test gets fresh Arena2026Rebuilt
```

---

## Key gotchas (hard-won debugging lessons)

### 1. `physicsMotionRequiresSimulationPeriodic` is the Layer 3 contract test
`SwerveModuleSimulation` pre-fills its position caches with `SIMULATION_SUB_TICKS_IN_1_PERIOD` (= 5) copies of the initial zero position at construction. Without `simulationPeriodic()`, those 5 zeros are re-read each cycle. Pose stays at origin even when `runVelocity()` is called. This is the fundamental difference between Layer 2 and Layer 3.

### 2. DCMotorSim coasts after `setPose()`
`ModuleIOSim` uses WPILib `DCMotorSim` which has its own internal velocity state. Calling `drive.setPose(Pose2d.kZero)` re-anchors the odometry estimator but does **not** stop the DCMotorSim. The motor coasts for v₀·τ ≈ 1 m/s × 0.1 s ≈ 0.1 m. Use tolerance `< 0.15 m` and 50 coast cycles, not 5.

### 3. Strafe threshold is lower than forward
Modules start facing forward (0°). A forward command works immediately. A strafe command requires modules to rotate 90° first, consuming several of the 50 test cycles. Strafe threshold: `> 0.05 m`; forward threshold: `> 0.1 m`.

### 4. Initial heading has physics noise
After one sub-tick, `initialPoseIsAtOrigin` sees a heading of ~1.5e-6 rad (sub-micro-radian numerical noise from dyn4j). Use tolerance `1e-4`, not `1e-6`.

### 5. Gradle UP-TO-DATE silently skips tests
Without `outputs.upToDateWhen { false }` in `build.gradle`, Gradle considers `:test` UP-TO-DATE when no source files changed. The WPILib "Test Robot Code" button does nothing. **Already fixed** — the line is present in `build.gradle`.

### 6. Running tests from Windows vs WSL
There is **no WSL access** to `C:\workspace`. Always use `gradlew.bat` via `PowerShell`:
```powershell
cd C:\workspace\FRC5010Claude
.\gradlew.bat test
```
`./gradlew test` via Bash will fail (`/mnt/c` not mounted).

### 7. REAL mode factory throws by design
`SwerveFactory.build()` and `buildWithoutPhysics()` throw `UnsupportedOperationException` for `TALON_FX`/`SPARK_TALON` in REAL mode. Teams must instantiate `ModuleIOTalonFXReal`/`ModuleIOSparkTalon` directly with their CTRE TunerX `SwerveModuleConstants`. This is intentional — the factory can't construct motor specs without full TunerX gear/gain configuration.

---

## Key file locations

| What | Where |
|------|-------|
| Swerve config record | `src/main/java/org/frc5010/common/drive/swerve/SwerveConstants.java` |
| Factory (build/buildWithoutPhysics) | `src/main/java/org/frc5010/common/drive/swerve/SwerveFactory.java` |
| Subsystem (periodic, simulationPeriodic) | `src/main/java/org/frc5010/common/drive/swerve/akit/AkitSwerveDrive.java` |
| Physics module IO | `src/main/java/org/frc5010/common/drive/swerve/akit/ModuleIOSimPhysics.java` |
| Physics gyro IO | `src/main/java/org/frc5010/common/drive/swerve/akit/GyroIOSimPhysics.java` |
| DCMotorSim module IO | `src/main/java/org/frc5010/common/drive/swerve/akit/ModuleIOSim.java` |
| Odometry timestamp helper | `src/main/java/org/frc5010/common/drive/swerve/akit/util/PhoenixUtil.java` |
| Sim test base class | `src/test/java/org/frc5010/common/util/SimTestBase.java` |
| Layer 2 tests | `src/test/java/org/frc5010/common/subsystem/AkitSwerveDriveTest.java` |
| Layer 3 tests | `src/test/java/org/frc5010/common/subsystem/AkitSwerveDriveSimPhysicsTest.java` |
| IronMaple sources (read-only reference) | `yagsl_src_tmp/swervelib/simulation/ironmaple/` |

---

## SwerveConstants physics fields (added for IronMaple)

```java
new SwerveConstants.Builder()
    .moduleType(ModuleType.SIM)
    .gyroType(GyroType.SIM)
    .robotMassKg(45.0)           // 10–80 kg, default 45.0
    .bumperLengthMeters(0.76)    // 0.5–1.5 m, default 0.76
    .bumperWidthMeters(0.76)     // 0.5–1.5 m, default 0.76
    .build();
```

These are passed to `DriveTrainSimulationConfig` in `SwerveFactory.buildWithPhysicsSim()`.

---

## CI / devcontainer

- **CI:** `.github/workflows/ci.yml` — `./gradlew test` on every push/PR to `main`
- **Codespaces:** `.devcontainer/` — Java 17 bookworm + xvfb; `postCreateCommand` pre-warms Gradle; forwards ports 5810 (NT4), 5800, 1735
- **Sim sharing:** `xvfb-run ./gradlew simulateJava` in Codespace → VS Code auto-forwards port 5810 → AdvantageScope connects live

---

## Slash commands available

- `/new-sim-test` — step-by-step playbook for adding a Layer 2 or Layer 3 sim test
