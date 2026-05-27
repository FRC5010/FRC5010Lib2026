# /validate-replay — Validate replay fidelity after a non-trivial logging change

Use this workflow whenever you change `Robot.java` logging setup, `@AutoLog`-annotated inputs
structs, or AdvantageKit data receiver wiring. Replay must produce the same signal values as the
original run; divergence indicates a log-format break or a code path that only fires during replay.

---

## When to run this

Run replay validation after **any** of the following:
- Adding or renaming `@AutoLog` fields in `ModuleIOInputs`, `GyroIOInputs`, or `VisionIOInputs`
- Changing data receivers in `Robot.java` (`WPILOGWriter` path, `NT4Publisher`, etc.)
- Adding new `@AutoLogOutput` signals to `AkitSwerveDrive`, `Vision`, or custom subsystems
- Modifying `LogSummary.java` anomaly detection logic itself

---

## Step 1 — Produce a live log with the visual test

```powershell
.\gradlew.bat simulateJava -PvisualTest -PvisualTestExit
```

- Runs the 6-step visual test sequence (drive forward/back/strafe, rotation, vision correction).
- `-PvisualTestExit` makes the JVM call `System.exit(0)` as soon as the autonomous command
  finishes — so Gradle returns automatically without needing a manual Ctrl-C.
- Writes a `.wpilog` to `logs/` (e.g. `logs/FRC_20260527_143022.wpilog`).
- Note the filename — you need it in Step 2.

Glass will open briefly and close when the test completes (~20–30 seconds).

---

## Step 2 — Replay the log headlessly

```powershell
.\gradlew.bat simulateJava -Plog=logs/<your-log>.wpilog -PvisualTest -PreplayExit
```

- `-Plog=<path>` puts Robot into REPLAY mode: reads sensor inputs from the log, re-runs all
  robot code, writes output to `<original>_sim.wpilog`.
- `-PreplayExit` makes the robot call `System.exit(0)` as soon as the autonomous command
  finishes — so Gradle returns promptly without needing a manual Ctrl-C.
- `-PvisualTest` is required so `autonomousInit()` schedules the visual-test command
  (otherwise autonomous never starts and `-PreplayExit` never fires).

The replay writes `logs/<your-log>_sim.wpilog`.

---

## Step 3 — Analyze the replay log

```powershell
.\gradlew.bat replayValidate
```

This runs `LogSummary` on the most recent `_sim.wpilog` in `logs/`. Check:

1. **Duration** should be similar to the live log (replay runs faster but processes the same data).
2. **Anomaly Flags** — any `[WARN]` in the replay but not the live log indicates divergence.
3. **Vision section** — `max_poses/frame` should be non-zero for accepted observations if the
   live log saw tags during Step 6.

To compare the live log alongside the replay:

```powershell
.\gradlew.bat logSummary -PlogFile=logs/<your-log>.wpilog
.\gradlew.bat replayValidate
```

---

## Step 4 — Interpret results

| Observation | Likely cause |
|-------------|-------------|
| Replay log has no Vision entries | `Vision.periodic()` not called in replay — check CommandScheduler |
| `[WARN] Vision: all rejected` in replay only | Pose filter uses a live-only value (e.g. current time) |
| Replay duration >> live duration | `setUseTiming(false)` not active — check REPLAY mode branch in `Robot.java` |
| `_sim.wpilog` not created | Replay failed to start — check `-Plog=` path and that HAL GUI extensions are disabled |
| `[WARN] Loop overrun` in replay but not live | Slow host machine or Gradle daemon memory pressure — not a code issue |

### Known expected differences between live and replay

These are not bugs — they are inherent limitations of replay mode:

| Warning / difference | Expected? | Explanation |
|----------------------|-----------|-------------|
| `[WARN] Loop overrun: FullCycleMS max=1800+ms` | Yes | First JVM cycle has ~1–2 s startup overhead; this is the live log value replayed. Not a recurring overrun. |
| `[WARN] High current: TurnCurrentAmps max=150 A` | Yes | IronMaple caps turn-motor current at 150 A during rapid rotation. Simulation physics artifact, not real hardware behavior. |
| Step 5 (boundary check) prints `physics_x=-999.0` in replay output | Yes | `drive.getSimulatedPose()` returns `Optional.empty()` in replay (physics engine is not running). The -999 fallback is intentional. |
| Vision `RealOutputs` and `ReplayOutputs` both show `max_poses/frame=1` | Good | Confirms replay re-runs the same vision logic and produces the same accepted observations. |

---

## Reference — key files

| File | Purpose |
|------|---------|
| `Robot.java` | REPLAY mode setup: `setUseTiming(false)`, `WPILOGReader`/`WPILOGWriter`, `replayAutoExit` |
| `build.gradle` | `-PreplayExit` forwarding, `replayValidate` and `logSummary` tasks |
| `LogSummary.java` | Summary + anomaly detection (vision, gyro, current, loop overrun) |
| `SwerveVisualTest.java` | Layer 4 visual-test sequence (Steps 1–6 incl. vision correction) |
