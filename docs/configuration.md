# Configuration — SwerveConstants Reference

`SwerveConstants` is an immutable record built via a fluent builder. All fields accept WPILib typed units so you can express measurements in whatever unit is most natural; the library converts internally.

---

## Complete builder example

```java
import static edu.wpi.first.units.Units.*;

SwerveConstants CONSTANTS = new SwerveConstants.Builder()
    // Hardware selection
    .moduleType(ModuleType.TALON_FX)          // see Module types below
    .gyroType(GyroType.PIGEON2)               // or NAVX, SIM
    .gyroCanId(1)

    // Physical geometry
    .trackWidth(Inches.of(22.75))             // left-to-right wheel-centre distance
    .wheelBase(Inches.of(22.75))              // front-to-back wheel-centre distance
    .wheelRadius(Inches.of(2.0))              // actual worn-down radius, not nominal

    // Performance limits
    .maxLinearSpeed(MetersPerSecond.of(4.5))
    .maxAngularSpeed(RadiansPerSecond.of(2 * Math.PI))

    // CAN IDs — [driveId, steerId, encoderId] per module
    .frontLeftIds(1, 2, 3)
    .frontRightIds(4, 5, 6)
    .backLeftIds(7, 8, 9)
    .backRightIds(10, 11, 12)

    // CAN bus
    .canBusName("")                           // "" = RIO bus; CANivore name for Phoenix Pro
    .odometryFrequency(Hertz.of(100))         // 250 Hz for CANivore, 100 Hz for RIO bus

    // Physics simulation (IronMaple)
    .robotMass(Pounds.of(125))                // total weight including bumpers and battery
    .bumperLength(Inches.of(30))              // outside-to-outside front-to-back
    .bumperWidth(Inches.of(30))               // outside-to-outside left-to-right
    .build();
```

---

## Field reference

### Hardware

| Field | Type | Default | Notes |
|-------|------|---------|-------|
| `moduleType` | `ModuleType` | `SIM` | See table below |
| `gyroType` | `GyroType` | `SIM` | `PIGEON2`, `NAVX`, or `SIM` |
| `gyroCanId` | `int` | `0` | Ignored in SIM mode |
| `canBusName` | `String` | `""` | `""` = default RIO bus; CANivore name for Phoenix Pro |

#### Module types

| `ModuleType` | Drive motor | Steer motor | Notes |
|-------------|-------------|-------------|-------|
| `TALON_FX` | TalonFX (Falcon / Kraken) | TalonFX | Needs CTRE TunerX `SwerveModuleConstants` for REAL mode |
| `SPARK_MAX` | SparkMAX (NEO) | SparkMAX (NEO) | |
| `SPARK_TALON` | SparkMAX (NEO) | TalonFX | |
| `SIM` | — | — | Selected automatically in SIM mode regardless of above |

### Physical geometry

| Field | Type | Default | Notes |
|-------|------|---------|-------|
| `trackWidth` | `Distance` | `Inches.of(24)` | Left-to-right wheel-centre distance |
| `wheelBase` | `Distance` | `Inches.of(24)` | Front-to-back wheel-centre distance |
| `wheelRadius` | `Distance` | `Inches.of(2)` | Actual worn radius, not nominal |

`moduleTranslations` (FL, FR, BL, BR) is computed automatically from `wheelBase` and `trackWidth`.

### Performance limits

| Field | Type | Default |
|-------|------|---------|
| `maxLinearSpeed` | `LinearVelocity` | `MetersPerSecond.of(4.5)` |
| `maxAngularSpeed` | `AngularVelocity` | `RadiansPerSecond.of(2π)` |

### CAN IDs

Each module takes `[driveId, steerId]` or `[driveId, steerId, encoderId]`:

```java
.frontLeftIds(1, 2, 3)   // drive=1, steer=2, encoder=3
.frontLeftIds(1, 2)       // drive=1, steer=2 (no separate encoder)
```

Module order: **FL, FR, BL, BR** (matches WPILib convention).

### Odometry

| Field | Type | Default | Notes |
|-------|------|---------|-------|
| `odometryFrequency` | `Frequency` | `Hertz.of(100)` | 250 Hz for CANivore, 100 Hz for RIO bus |

### Physics simulation (IronMaple)

These three fields are passed to `DriveTrainSimulationConfig` and only matter in SIM mode. They should still reflect the real robot's physical properties so the simulation is accurate.

| Field | Type | Default | Valid range |
|-------|------|---------|-------------|
| `robotMass` | `Mass` | `Kilograms.of(45)` | 10–80 kg (FRC weight limits) |
| `bumperLength` | `Distance` | `Meters.of(0.76)` | 0.5–1.5 m |
| `bumperWidth` | `Distance` | `Meters.of(0.76)` | 0.5–1.5 m |

`bumperLength` and `bumperWidth` are outside-to-outside full dimensions (not per-side). Typical FRC bumpers are ~30 in (0.76 m) total.

---

## Accessing fields from `AkitSwerveDrive`

```java
// Typed measures returned directly — no raw-double accessors
LinearVelocity  vMax = drive.getMaxLinearSpeed();
AngularVelocity wMax = drive.getMaxAngularSpeed();

// Extract raw double when a WPILib API needs it
double vMaxMps = drive.getMaxLinearSpeed().in(MetersPerSecond);
```

---

## Validation

`SwerveConstants.Builder.build()` validates all fields and throws `IllegalArgumentException` with a descriptive message if any value is out of range. Common mistakes:

- `wheelRadius` is the *actual worn-down* radius, not the nominal size printed on the wheel. Measure it.
- `bumperLength`/`bumperWidth` must be ≥ 0.5 m (19.7 in). The minimum is there because IronMaple needs a physically plausible body size.
- `robotMass` includes bumpers (~5 lb) and battery (~13 lb). A typical robot comes in at 100–125 lb total.
