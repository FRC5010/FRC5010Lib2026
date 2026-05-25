# Testing

The project uses a four-layer test pyramid. Layers 1–3 are automated and run in CI; Layer 4 is visual and run manually.

---

## Test pyramid

| Layer | What | Factory | IO impl | Count |
|-------|------|---------|---------|-------|
| 1 — unit | `SwerveConstantsTest`, `SwerveFactoryModeTest`, `TunableGainsTest` | — | — | 33 |
| 2 — subsystem sim | `AkitSwerveDriveTest` | `buildWithoutPhysics()` | `ModuleIOSim` (DCMotorSim) | 8 |
| 3 — physics integration | `AkitSwerveDriveSimPhysicsTest` | `build()` | `ModuleIOSimPhysics` (IronMaple) | 7 |
| 4 — visual / interactive | `RobotContainer` visual-test sequence | `build()` | `ModuleIOSimPhysics` | visual |

**Total: 48/48 passing.**

---

## Running tests

```powershell
# Windows PowerShell — always use this; WSL cannot reach C:\workspace
cd C:\workspace\FRC5010Claude
.\gradlew.bat test

# Force a re-run even if nothing changed
.\gradlew.bat cleanTest test
```

HTML report: `build/reports/tests/test/index.html`

VS Code shortcut: **Ctrl+Shift+P → Tasks: Run Test Task → Run Unit Tests**

---

## Layer 1 — Unit tests

Test `SwerveConstants` builder validation, `SwerveFactory` mode-selection logic, and `TunableGains`. No simulation infrastructure needed — these are plain JUnit 5 tests.

Representative assertions:
```java
// Builder accepts typed units
SwerveConstants c = new SwerveConstants.Builder()
    .trackWidth(Inches.of(22.75))
    .robotMass(Pounds.of(125))
    .build();
assertEquals(22.75, c.trackWidth.in(Inches), 1e-6);

// Factory throws in REAL mode for hardware types
RobotMode.set(Mode.REAL);
assertThrows(UnsupportedOperationException.class, () -> SwerveFactory.build(TALON_CONSTANTS));
```

---

## Layer 2 — Subsystem sim tests

`buildWithoutPhysics()` uses WPILib `DCMotorSim` — no dyn4j, no IronMaple overhead. All Layer 2 tests extend `SimTestBase`, which initialises HAL, pauses the FPGA clock, and cleans up `CommandScheduler` between tests.

### Test skeleton

```java
@Test
void myBehavior() {
    enableTeleop();
    for (int i = 0; i < 50; i++) {          // 50 × 20 ms = 1 simulated second
        drive.runVelocity(new ChassisSpeeds(vx, vy, omega));
        drive.periodic();                     // reads DCMotorSim → odometry
        stepOneCycle();                       // advance FPGA clock 20 ms
    }
    Pose2d pose = drive.getPose();
    // assert on pose.getX(), pose.getY(), pose.getRotation()
}
```

No `drive.simulationPeriodic()` — `ModuleIOSim` calls `driveSim.update(0.02)` internally during `periodic()`.

### Thresholds (Layer 2)

| Motion | Expected after 50 cycles (1 s at 1 m/s or π rad/s) |
|--------|-----------------------------------------------------|
| Forward 1 m/s | X > 0.1 m |
| Strafe 1 m/s | Y > 0.1 m |
| Rotate π rad/s | \|heading\| > 0.1 rad |
| After `setPose()` coast | < 0.15 m from target (use 50 coast cycles, not 5) |

`setPose()` re-anchors odometry but does not stop the DCMotorSim — the motor coasts for v₀·τ ≈ 0.1 m.

---

## Layer 3 — Physics integration tests

`build()` uses IronMaple (dyn4j). Every Layer 3 test must follow the strict per-cycle call order:

```java
drive.runVelocity(speeds);    // 1. queue voltage commands
drive.simulationPeriodic();   // 2. advance dyn4j: 5 sub-ticks × 4 ms = 20 ms
drive.periodic();             // 3. read updated module caches → odometry
stepOneCycle();               // 4. advance FPGA clock 20 ms
```

The `step()` helper in `AkitSwerveDriveSimPhysicsTest` wraps steps 2–4.

**Wrong order = stale data.** `periodic()` reads IronMaple module position caches. Those caches are only filled by `simulationPeriodic()` sub-ticks. Calling `periodic()` first reads the initial zero-filled caches and no motion appears.

### Teardown — `SimulatedArena` singleton

`SimulatedArena` is a static singleton. Every `build()` call registers a physics body into it. Without cleanup, bodies accumulate across tests.

```java
@AfterEach
public void simTeardown() {
    SimulatedArena.getInstance().shutDown();
    java.lang.reflect.Field f = SimulatedArena.class.getDeclaredField("instance");
    f.setAccessible(true);
    f.set(null, null);     // null → next test gets a fresh Arena2026Rebuilt
    RobotMode.resetForTesting();
    super.simTeardown();
}
```

### Thresholds (Layer 3)

| Motion | Expected after 50 cycles (1 s at 1 m/s or π rad/s) |
|--------|-----------------------------------------------------|
| Forward 1 m/s | X > 0.1 m |
| Strafe 1 m/s | Y > 0.05 m (modules must rotate 90° first — costs ~10 cycles) |
| Rotate π rad/s | \|heading\| > 0.05 rad |
| Initial pose heading | tolerance 1e-4 rad (not 1e-6 — dyn4j has sub-micro-radian noise) |

Strafe threshold is lower than forward because modules start facing forward (0°) and must steer to 90° before contributing lateral motion.

---

## Layer 4 — Visual test

Runs as a full robot program with the automated `SwerveVisualTest` command sequence:

1. Drive forward 1 m (1 s at 1 m/s)
2. Strafe left 0.5 m
3. Rotate 90°
4. Drive in the alliance-correct direction
5. Approach the field boundary and stop

```powershell
.\gradlew.bat simulateJava -PvisualTest
```

Watch the robot in AdvantageScope or the Glass **Field2d** widget. The sequence never runs in CI — it is a manual sanity check.

---

## Adding a new test

See the `/new-sim-test` slash command in Claude Code for a step-by-step playbook with copy-paste skeletons for both Layer 2 and Layer 3.
