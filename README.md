# FRC5010Claude — Swerve Drive Framework

[![CI](https://github.com/clrozeboom/FRC5010Claude/actions/workflows/ci.yml/badge.svg)](https://github.com/clrozeboom/FRC5010Claude/actions/workflows/ci.yml)

A clean, AdvantageKit-compatible swerve drive library for FRC teams using WPILib 2026. Students configure a single `SwerveConstants` record; `SwerveFactory` wires up the correct IO implementations for real hardware, simulation, and log replay automatically.

---

## Quick start

```java
// In your RobotContainer
AkitSwerveDrive drive = SwerveFactory.build(
    new SwerveConstants.Builder()
        .moduleType(ModuleType.TALON_FX)
        .gyroType(GyroType.PIGEON2)
        .trackWidthMeters(0.55)
        .wheelBaseMeters(0.55)
        .build());
```

`SwerveFactory.build()` automatically selects the right IO layer:

| `RobotMode` | What you get |
|-------------|-------------|
| `REAL`      | Real hardware — instantiate `ModuleIOTalonFXReal` directly with your TunerX `SwerveModuleConstants` |
| `SIM`       | Full IronMaple physics simulation (YAGSL), pose published over NT4 |
| `REPLAY`    | No-op IO — feed a `.wpilog` back through AdvantageKit |

For unit tests use `SwerveFactory.buildWithoutPhysics()` — identical except SIM mode uses a lightweight `DCMotorSim` with no YAGSL overhead.

---

## Running tests

```bash
./gradlew test
```

27 tests across three suites: `SwerveConstantsTest`, `SwerveFactoryModeTest`, `TunableGainsTest`.

---

## Sharing the simulation

When the simulation runs, WPILib publishes all robot state over **NetworkTables 4** on port **5810**. [AdvantageScope](https://github.com/Mechanical-Advantage/AdvantageScope) can connect to it from any machine on the same network for a live 3D view.

### Option A — Live connection (same LAN or Codespace)

1. Run the simulation:
   ```bash
   # Local machine
   ./gradlew simulateJava

   # Inside a GitHub Codespace (headless — xvfb provides a virtual display)
   xvfb-run ./gradlew simulateJava
   ```
2. In **AdvantageScope → File → Connect to Robot**, enter:
   - **Same LAN**: your machine's IP address, port `5810`
   - **Codespace**: in VS Code's *Ports* panel, copy the forwarded URL for port `5810`, then paste it into AdvantageScope
3. The robot's pose, module states, and gyro heading appear in real time.

### Option B — Log replay (async, no network required)

AdvantageKit writes a `.wpilog` file to the `logs/` directory every time the sim runs (once your robot project configures the Logger). Share that file with teammates; they open it in AdvantageScope with **File → Open Log** and can scrub through the full run offline.

This is the recommended workflow for reviewing autonomous routines or debugging a specific scenario — no live connection needed, works from anywhere.

---

## Developing in GitHub Codespaces

[![Open in GitHub Codespaces](https://github.com/codespaces/badge.svg)](https://codespaces.new/clrozeboom/FRC5010Claude)

The repo ships a `.devcontainer` that provides Java 17, Gradle, and all vendordep dependencies pre-downloaded. Port 5810 (NT4) is forwarded automatically so AdvantageScope on your laptop can connect to a sim running in the cloud.

**What works in Codespaces:**
- Full edit, build, and test cycle (`./gradlew test`)
- Running the simulation headlessly (`xvfb-run ./gradlew simulateJava`)
- NT4 live connection to AdvantageScope via the forwarded port

**What requires a local laptop:**
- Deploying to the physical RoboRIO (use the WPILib VS Code extension on your local machine with the repo cloned there)

---

## Architecture

```
SwerveConstants          — immutable record, Builder pattern
    └─ SwerveFactory     — build() / buildWithoutPhysics()
           └─ AkitSwerveDrive  — subsystem, AdvantageKit IO abstraction
                  ├─ ModuleIO  — per-module interface
                  │    ├─ ModuleIOTalonFXReal  (REAL, needs TunerX SwerveModuleConstants)
                  │    ├─ ModuleIOSparkTalon   (REAL, needs TunerX SwerveModuleConstants)
                  │    ├─ ModuleIOSimPhysics   (SIM via build() — IronMaple)
                  │    ├─ ModuleIOTalonFXSim   (SIM via build() — IronMaple + TalonFX hardware)
                  │    └─ ModuleIOSim          (SIM via buildWithoutPhysics() — DCMotorSim)
                  └─ GyroIO
                       ├─ GyroIOPigeon2        (REAL)
                       ├─ GyroIONavX           (REAL — stub, needs Kauai vendordep)
                       ├─ GyroIOSimPhysics     (SIM via build() — IronMaple GyroSimulation)
                       └─ GyroIOSim            (SIM via buildWithoutPhysics() — kinematics integration)
```

---

## Vendordeps

| Library | Version | Purpose |
|---------|---------|---------|
| WPILib / GradleRIO | 2026.2.1 | Core robot framework |
| Phoenix 6 | 26.2.0 | TalonFX, CANcoder, Pigeon2 |
| REVLib | latest | Spark MAX / Spark Flex |
| AdvantageKit | latest | IO abstraction + log replay |
| PathPlannerLib | 2026.1.2 | Autonomous path following |
| YAGSL | 2026.4.1 | IronMaple physics simulation engine |
| ReduxLib | 2026.1.1 | YAGSL dependency |
| ThriftyLib | 2026.1.0 | YAGSL dependency |
| PhotonVision | latest | Vision targeting |
