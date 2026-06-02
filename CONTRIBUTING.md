# Contributing

The code, the tests, the docs, and the agent skills must stay in sync. Stale guidance causes the next contributor (or agent) to repeat solved problems.

## Before committing any change to the common library

(`src/main/java/org/frc5010/common/...`)

1. **Run the full test suite** — `.\gradlew.bat test` — all tests must pass. Never weaken an assertion to force a pass; fix the root cause.
2. Update any affected slash command in `.claude/commands/` (e.g. `new-sim-test`, `new-robot-profile`, `diagnose-log`, `validate-replay`).
3. Update the relevant `docs/` page (`configuration`, `architecture`, `testing`, `simulation`, `robot-profiles`, `vision`).
4. Update `CLAUDE.md` if a gotcha, file location, or architecture section is no longer accurate.
5. If a new reusable pattern was introduced, consider whether it warrants a new slash command or docs page.

## Logging changes — validate replay fidelity

Any change to `@AutoLog` fields, `Robot.java` data receivers, or `LogSummary.java` must be validated end-to-end:

```powershell
# 1. Produce a live log (Glass opens, auto-closes when test completes)
.\gradlew.bat simulateJava -PvisualTest -PvisualTestExit
# 2. Replay it headlessly; exits automatically when autonomous completes
.\gradlew.bat simulateJava -Plog=logs/<your-log>.wpilog -PvisualTest -PreplayExit
# 3. Check the replay log for anomalies vs the live log
.\gradlew.bat replayValidate
```

See `/validate-replay` for the full workflow and how to interpret the output.
