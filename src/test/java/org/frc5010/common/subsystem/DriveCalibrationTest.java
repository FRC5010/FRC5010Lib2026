package org.frc5010.common.subsystem;

import static org.junit.jupiter.api.Assertions.*;

import org.frc5010.common.drive.swerve.SwerveConstants;
import org.frc5010.common.drive.swerve.SwerveConstants.GyroType;
import org.frc5010.common.drive.swerve.SwerveConstants.ModuleType;
import org.frc5010.common.drive.swerve.SwerveFactory;
import org.frc5010.common.drive.swerve.akit.AkitSwerveDrive;
import org.frc5010.common.drive.swerve.calibration.CalibrationResult;
import org.frc5010.common.drive.swerve.calibration.MotorCalibrationRoutine;
import org.frc5010.common.robot.Mode;
import org.frc5010.common.robot.RobotMode;
import org.frc5010.common.util.SimTestBase;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * Layer 2 — calibration routine tests using lightweight DCMotorSim modules.
 *
 * <p>These tests verify that {@link MotorCalibrationRoutine} correctly collects
 * open-loop voltage-vs-velocity data and that {@link CalibrationResult} produced by
 * {@link MotorCalibrationRoutine#fit()} has physically plausible gain values.
 *
 * <p>The drive motor model is a Falcon 500 with 6.75:1 reduction; the steer model is a
 * Falcon 500 with ~21.4:1 (150/7) reduction. Both are implemented in {@code ModuleIOSim}.
 */
class DriveCalibrationTest extends SimTestBase {

  private static final SwerveConstants CONSTANTS =
      new SwerveConstants.Builder()
          .moduleType(ModuleType.SIM)
          .gyroType(GyroType.SIM)
          .build();

  /** Voltage steps used across calibration tests (ascending, starting above dead-band). */
  private static final double[] VOLTAGE_STEPS = {1.0, 2.0, 3.0, 4.0, 5.0};

  /** Cycles to settle at each voltage step before sampling steady-state velocity. */
  private static final int SETTLE_CYCLES = 25;

  private AkitSwerveDrive drive;

  @BeforeEach
  @Override
  public void simSetup() {
    super.simSetup();
    RobotMode.set(Mode.SIM);
    drive = SwerveFactory.buildWithoutPhysics(CONSTANTS);
  }

  @AfterEach
  @Override
  public void simTeardown() {
    RobotMode.resetForTesting();
    super.simTeardown();
  }

  // ---------------------------------------------------------------------------
  // Drive motor characterization
  // ---------------------------------------------------------------------------

  @Test
  void driveVoltageRampIncreasesVelocityMonotonically() {
    enableTeleop();
    double[] velocities = new double[VOLTAGE_STEPS.length];

    for (int i = 0; i < VOLTAGE_STEPS.length; i++) {
      for (int j = 0; j < SETTLE_CYCLES; j++) {
        drive.runCharacterization(VOLTAGE_STEPS[i]);
        drive.periodic();
        stepOneCycle();
      }
      velocities[i] = drive.getDriveFFCharacterizationVelocity();
    }

    for (int i = 1; i < velocities.length; i++) {
      assertTrue(
          velocities[i] > velocities[i - 1],
          "Velocity at "
              + VOLTAGE_STEPS[i]
              + " V ("
              + velocities[i]
              + " rad/s) should exceed velocity at "
              + VOLTAGE_STEPS[i - 1]
              + " V ("
              + velocities[i - 1]
              + " rad/s)");
    }
  }

  @Test
  void driveCalibrationFitProducesPositiveGains() {
    enableTeleop();
    MotorCalibrationRoutine routine =
        MotorCalibrationRoutine.collectDriveRamp(
            drive, VOLTAGE_STEPS, SETTLE_CYCLES, () -> { drive.periodic(); stepOneCycle(); });

    CalibrationResult result = routine.fit();

    assertTrue(result.kV() > 0,
        "kV must be positive; got " + result.kV());
    assertTrue(result.kS() >= 0,
        "kS must be non-negative; got " + result.kS());
  }

  @Test
  void driveKVIsInPhysicallyPlausibleRange() {
    enableTeleop();
    MotorCalibrationRoutine routine =
        MotorCalibrationRoutine.collectDriveRamp(
            drive, VOLTAGE_STEPS, SETTLE_CYCLES, () -> { drive.periodic(); stepOneCycle(); });

    double kV = routine.fit().kV();

    assertTrue(kV > 0.01,
        "kV=" + kV + " is too small; likely a data collection error");
    assertTrue(kV < 2.0,
        "kV=" + kV + " is unreasonably large for an FRC swerve drive motor");
  }

  @Test
  void driveRSquaredIndicatesGoodLinearFit() {
    enableTeleop();
    MotorCalibrationRoutine routine =
        MotorCalibrationRoutine.collectDriveRamp(
            drive, VOLTAGE_STEPS, SETTLE_CYCLES, () -> { drive.periodic(); stepOneCycle(); });

    double rSquared = routine.fit().rSquared();

    assertTrue(rSquared > 0.90,
        "R² should indicate a good linear fit (> 0.90); got " + rSquared);
  }

  // ---------------------------------------------------------------------------
  // Steer motor characterization
  // ---------------------------------------------------------------------------

  @Test
  void steerVoltageRampIncreasesVelocityMonotonically() {
    enableTeleop();
    double[] velocities = new double[VOLTAGE_STEPS.length];

    for (int i = 0; i < VOLTAGE_STEPS.length; i++) {
      for (int j = 0; j < SETTLE_CYCLES; j++) {
        drive.runSteerCharacterization(VOLTAGE_STEPS[i]);
        drive.periodic();
        stepOneCycle();
      }
      velocities[i] = drive.getSteerFFCharacterizationVelocity();
    }

    for (int i = 1; i < velocities.length; i++) {
      assertTrue(
          velocities[i] > velocities[i - 1],
          "Steer velocity at "
              + VOLTAGE_STEPS[i]
              + " V ("
              + velocities[i]
              + " rad/s) should exceed velocity at "
              + VOLTAGE_STEPS[i - 1]
              + " V ("
              + velocities[i - 1]
              + " rad/s)");
    }
  }

  @Test
  void steerCalibrationFitProducesPositiveKV() {
    enableTeleop();
    MotorCalibrationRoutine routine =
        MotorCalibrationRoutine.collectSteerRamp(
            drive, VOLTAGE_STEPS, SETTLE_CYCLES, () -> { drive.periodic(); stepOneCycle(); });

    CalibrationResult result = routine.fit();

    assertTrue(result.kV() > 0,
        "Steer kV must be positive; got " + result.kV());
    assertTrue(result.rSquared() > 0.90,
        "Steer R² should indicate a good linear fit (> 0.90); got " + result.rSquared());
  }
}
